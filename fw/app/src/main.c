/*
 * Tram speed and event recorder: Zephyr glue around lib/odo.
 *
 *  - LPUART3: sensor frames in (one per 10 ms sample: encoder captures, IMU, brake input),
 *    commands in, a status frame out every 10th sample. On the target the sample is built
 *    by the 100 Hz timer from the encoder capture (QTMR/ENC) and the SPI IMU; the Renode
 *    model has neither, so the rig (sim/rig.py) sends the sampled front end as frames.
 *  - recorder thread: one odo_unit_tick per sample
 *  - power-fail comparator on GPIO2_IO00 (low = supply lost, level interrupt): the ISR wakes the
 *    highest-priority thread, which writes the power-fail record to FRAM at once
 *  - FRAM: 128 KiB array here (SPI FRAM on the target), accessed through odo_nvm_t
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include "odo/odo.h"

#define USER_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec pfi = GPIO_DT_SPEC_GET(USER_NODE, pfi_gpios);
static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(odo_sensor_uart));

/* ---- FRAM ------------------------------------------------------------------------------ */

uint8_t odo_fram[ODO_SLOTS * ODO_SLOT_BYTES];

#ifdef CONFIG_ODO_TEST_HOOKS
volatile int32_t odo_test_tear = -1; /* >= 0: the next FRAM write stores only this many bytes */
volatile uint32_t odo_test_recover;  /* rig sets 1: run the boot-time recovery on the FRAM */
volatile uint32_t odo_diag_rec_valid;
volatile uint32_t odo_diag_rec_head;
volatile uint32_t odo_diag_rec_type; /* newest valid record after the recovery */
volatile uint32_t odo_diag_rec_s_cm;
#endif

static uint32_t fram_write(void *ctx, uint32_t addr, const uint8_t *src, uint32_t len)
{
	uint32_t n = len;

	ARG_UNUSED(ctx);
#ifdef CONFIG_ODO_TEST_HOOKS
	if (odo_test_tear >= 0) {
		n = ((uint32_t)odo_test_tear < len) ? (uint32_t)odo_test_tear : len;
		odo_test_tear = -1;
	}
#endif
	memcpy(&odo_fram[addr], src, n);
	return n;
}

static void fram_read(void *ctx, uint32_t addr, uint8_t *dst, uint32_t len)
{
	ARG_UNUSED(ctx);
	memcpy(dst, &odo_fram[addr], len);
}

static const odo_nvm_t nvm = {fram_write, fram_read, NULL};

/* ---- Diagnostics, read by the rig through the ELF symbols -------------------------------- */

volatile uint32_t odo_diag_ticks;
volatile uint32_t odo_diag_max_cyc;   /* worst odo_unit_tick, CPU cycles */
volatile uint32_t odo_diag_pf_cyc;    /* power-fail ISR to record in FRAM, CPU cycles */
volatile uint32_t odo_diag_rx_err;
volatile uint32_t odo_diag_rx_overflow;
volatile uint32_t odo_diag_cmds;

static odo_unit_t unit;
static K_MUTEX_DEFINE(unit_lock);

/* ---- UART receive -------------------------------------------------------------------------- */

RING_BUF_DECLARE(rx_ring, 2048);
static K_SEM_DEFINE(rx_sem, 0, 1);

static void uart_isr(const struct device *dev, void *user)
{
	ARG_UNUSED(user);
	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev)) {
		uint8_t buf[32];
		const int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		if (ring_buf_put(&rx_ring, buf, (uint32_t)n) < (uint32_t)n) {
			odo_diag_rx_overflow++;
		}
	}
	k_sem_give(&rx_sem);
}

/* ---- Power fail -------------------------------------------------------------------------- */

static K_SEM_DEFINE(pf_sem, 0, 1);
static volatile uint32_t pf_t0;
static struct gpio_callback pf_cb;

static void pf_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	pf_t0 = k_cycle_get_32();
	/* level interrupt: masked here, re-armed by the thread when the supply is back */
	(void)gpio_pin_interrupt_configure_dt(&pfi, GPIO_INT_DISABLE);
	k_sem_give(&pf_sem);
}

static void pf_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&pf_sem, K_FOREVER);
		k_mutex_lock(&unit_lock, K_FOREVER);
		odo_unit_power(&unit, false);
		k_mutex_unlock(&unit_lock);
		odo_diag_pf_cyc = k_cycle_get_32() - pf_t0;
		/* supply lost: the interrupt stays masked; the return is polled every millisecond */
		while (gpio_pin_get_dt(&pfi) != 0) {
			k_msleep(1);
		}
		k_mutex_lock(&unit_lock, K_FOREVER);
		odo_unit_power(&unit, true);
		k_mutex_unlock(&unit_lock);
		(void)gpio_pin_interrupt_configure_dt(&pfi, GPIO_INT_LEVEL_ACTIVE);
	}
}

K_THREAD_STACK_DEFINE(pf_stack, 2048);
static struct k_thread pf_data;

/* ---- Recorder ------------------------------------------------------------------------------ */

static void command(const odo_cmd_t *c)
{
	odo_diag_cmds++;
	k_mutex_lock(&unit_lock, K_FOREVER);
	if ((c->cmd == ODO_CMD_DIAM0) || (c->cmd == ODO_CMD_DIAM1)) {
		const uint32_t k = (c->cmd == ODO_CMD_DIAM0) ? 0U : 1U;
		const float d = (float)c->arg * 1e-6f;

		if ((d >= ODO_DIAM_MIN) && (d <= ODO_DIAM_MAX)) {
			unit.cfg.diam[k] = d;
			unit.est.wheel[k].edge_m = (ODO_PI * d) / (float)ODO_EDGES_PER_REV;
			odo_cfg_seal(&unit.cfg);
		}
	} else if (c->cmd == ODO_CMD_CAL) {
		(void)odo_unit_cal_start(&unit, c->arg);
	} else {
		/* unknown command: ignored */
	}
	k_mutex_unlock(&unit_lock);
}

static void status_out(void)
{
	uint8_t buf[ODO_MAX_FRAME];
	const uint32_t n = odo_encode_status(&unit, buf, sizeof(buf));

	for (uint32_t i = 0; i < n; i++) {
		uart_poll_out(uart, buf[i]);
	}
}

#ifdef CONFIG_ODO_TEST_HOOKS
static void test_recover(void)
{
	static odo_journal_t j;
	odo_entry_t e;

	odo_journal_recover(&j, &nvm);
	odo_diag_rec_valid = j.valid;
	odo_diag_rec_head = j.head;
	if (odo_journal_get(&j, 0U, &e)) {
		odo_diag_rec_type = e.type;
		odo_diag_rec_s_cm = e.s_cm;
	}
	odo_test_recover = 0U;
}
#endif

static void rec_thread(void *a, void *b, void *c)
{
	static odo_rx_t rx;
	uint8_t chunk[64];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	odo_rx_init(&rx);
	for (;;) {
		uint32_t n;

		(void)k_sem_take(&rx_sem, K_MSEC(5));
#ifdef CONFIG_ODO_TEST_HOOKS
		if (odo_test_recover != 0U) {
			test_recover();
		}
#endif
		while ((n = ring_buf_get(&rx_ring, chunk, sizeof(chunk))) > 0U) {
			for (uint32_t i = 0; i < n; i++) {
				odo_sample_t s;
				odo_cmd_t cmd;
				const odo_rx_kind_t k = odo_rx_byte(&rx, chunk[i], &s, &cmd);

				if (k == ODO_RX_SENSOR) {
					k_mutex_lock(&unit_lock, K_FOREVER);
					const uint32_t t0 = k_cycle_get_32();

					odo_unit_tick(&unit, &s);
					const uint32_t dt = k_cycle_get_32() - t0;

					k_mutex_unlock(&unit_lock);
					if (dt > odo_diag_max_cyc) {
						odo_diag_max_cyc = dt;
					}
					odo_diag_ticks++;
					if ((odo_diag_ticks % 10U) == 0U) {
						status_out();
					}
				} else if (k == ODO_RX_CMD) {
					command(&cmd);
				} else {
					/* byte consumed, no frame yet */
				}
			}
		}
		odo_diag_rx_err = rx.crc_err;
	}
}

K_THREAD_STACK_DEFINE(rec_stack, 4096);
static struct k_thread rec_data;

int main(void)
{
	/* the calibration is kept in FRAM on the target; the test image starts uncalibrated
	 * and receives the diameters by command */
	static const odo_cfg_t none = {{0.0f, 0.0f}, 0U};
	int err = gpio_pin_configure_dt(&pfi, GPIO_INPUT);

	err |= gpio_pin_interrupt_configure_dt(&pfi, GPIO_INT_LEVEL_ACTIVE);
	gpio_init_callback(&pf_cb, pf_isr, BIT(pfi.pin));
	err |= gpio_add_callback(pfi.port, &pf_cb);
	if ((err != 0) || !device_is_ready(uart)) {
		printk("odo: I/O init failed (%d)\n", err);
		return 0;
	}
	odo_unit_init(&unit, &nvm, &none);
	uart_irq_callback_user_data_set(uart, uart_isr, NULL);
	uart_irq_rx_enable(uart);
	k_thread_create(&pf_data, pf_stack, K_THREAD_STACK_SIZEOF(pf_stack), pf_thread, NULL, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	k_thread_name_set(&pf_data, "pfail");
	k_thread_create(&rec_data, rec_stack, K_THREAD_STACK_SIZEOF(rec_stack), rec_thread, NULL, NULL, NULL, 2,
			0, K_NO_WAIT);
	k_thread_name_set(&rec_data, "recorder");
	printk("odo READY\n");
	return 0;
}
