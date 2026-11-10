"""Model checks against the sources and against independent calculations."""

import numpy as np
import pytest
from odo import adhesion, data, holdup


def test_bostrab_table_is_self_consistent():
    # BOStrab Anlage 2 gives a = v^2 / (3.6^2 * 2 s); the table values must satisfy it.
    for v, (a, s) in data.BOSTRAB_EMERGENCY.items():
        assert a == pytest.approx(v * v / (3.6**2 * 2.0 * s), rel=0.04)


def test_polach_table2_values():
    assert data.POLACH["dry"] == {"kA": 1.0, "kS": 0.4, "mu0": 0.55, "A": 0.4, "B": 0.6}
    assert data.POLACH["wet"] == {"kA": 0.3, "kS": 0.1, "mu0": 0.3, "A": 0.4, "B": 0.2}


@pytest.fixture(scope="module")
def k_eps():
    q = data.M_EMPTY * data.G0 / (2 * data.N_AXLES)
    return adhesion.eps_per_creep(q, data.D_NEW / 2.0)


def test_hertz_contact_size(k_eps):
    a = adhesion.hertz_radius(data.M_EMPTY * data.G0 / 12, data.D_NEW / 2)
    assert 3.5e-3 < a < 4.5e-3  # a few millimetres, as for rail wheels


def test_peaks_order_and_leaf_targets(k_eps):
    tab = adhesion.condition_table(k_eps)
    peaks = [adhesion.peak(dict(zip(["kA", "kS", "mu0", "A", "B"], r)), k_eps, 10.0)[0] for r in tab]
    assert peaks[0] > peaks[1] > peaks[2] > peaks[3]
    assert peaks[2] == pytest.approx(data.MU_DRY_LEAF, abs=1e-3)
    assert peaks[3] == pytest.approx(data.MU_WET_LEAF, abs=1e-3)
    assert 0.35 < peaks[0] < 0.55 and 0.2 < peaks[1] < 0.3


def test_slope_is_the_derivative(k_eps):
    p = adhesion.condition_table(k_eps)[1]
    w = np.array([0.001, 0.01, 0.1, 0.5, 1.5])
    f, df = adhesion.creep_coeff_and_slope(w, 8.0, k_eps, *p)
    h = 1e-6
    num = (adhesion.creep_coeff(w + h, 8.0, k_eps, *p) - adhesion.creep_coeff(w - h, 8.0, k_eps, *p)) / (
        2 * h
    )
    assert np.allclose(df, num, rtol=1e-4, atol=1e-6)
    assert np.allclose(f, adhesion.creep_coeff(w, 8.0, k_eps, *p))


def test_creep_inversion(k_eps):
    p = adhesion.condition_table(k_eps)[0]
    fpk, wpk = adhesion.peak(dict(zip(["kA", "kS", "mu0", "A", "B"], p)), k_eps, 10.0)
    w = adhesion.invert_creep(np.array([0.05, 0.2]), 10.0, k_eps, *p, w_peak=wpk)
    assert np.allclose(adhesion.creep_coeff(w, 10.0, k_eps, *p), [0.05, 0.2], atol=1e-6)
    assert adhesion.invert_creep(np.array([0.9]), 10.0, k_eps, *p, w_peak=wpk)[0] == pytest.approx(
        wpk, abs=1e-6
    )


def test_holdup_fram_against_sd():
    s = holdup.summary()
    fram, sd = s["options"]
    assert s["ring_bytes"] == 6000 * 16
    # FRAM: a few hundred microfarads ride through S2; SD needs a supercapacitor
    assert 200.0 < fram["S2"]["C_input_uF"] < 600.0
    assert sd["S2"]["C_supercap_F"] > 0.1
    assert fram["S3"]["C_input_uF"] == pytest.approx(2.0 * fram["S2"]["C_input_uF"], rel=0.05)
