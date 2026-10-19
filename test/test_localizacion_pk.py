"""Pruebas de orientación, fusión temporal y degradación sin GPS."""

import math
from pathlib import Path
import random

import pytest

from puntos_kilometricos.localizacion_pk import (
    Hito, KalmanPK, MotionBuffer, PKTracker, RailwayMap, Settings,
    formatear_pk, pk_a_metros,
)


DEGREES_PER_METER = 180 / (math.pi * 6371000)


def straight_map(track='A', offset=0.0, spacing=500, end=3000):
    return RailwayMap([
        Hito(track, pk, offset * DEGREES_PER_METER, pk * DEGREES_PER_METER)
        for pk in range(0, end + 1, spacing)])


def gps(tracker, time, pk, sigma=1.0):
    return tracker.observe(time, 0.0, pk * DEGREES_PER_METER, sigma)


def oriented(direction=1, signed_speed=10.0, spacing=500):
    motion = MotionBuffer()
    tracker = PKTracker(straight_map(spacing=spacing), motion, Settings(map_sigma=1.0))
    initial_pk = 100 if direction > 0 else 2000
    for t in range(6):
        motion.add(t, signed_speed, 0.01)
        gps(tracker, t, initial_pk + direction * abs(signed_speed) * t)
    assert tracker.state == 'AVANCE'
    return tracker, motion, initial_pk


@pytest.mark.parametrize('text, meters', [
    ('1+500', 1500), ('1+050', 1050), ('0+001', 1), (' 12+000 ', 12000),
])
def test_pk_units(text, meters):
    assert pk_a_metros(text) == meters
    assert formatear_pk(meters) == text.strip()


@pytest.mark.parametrize('text', ['1+50', '1+1000', '1.050', '', None])
def test_invalid_pk(text):
    with pytest.raises(ValueError):
        pk_a_metros(text)


def test_rounding_carries_to_next_kilometer():
    assert formatear_pk(1999.6) == '2+000'
    assert formatear_pk(1050.4) == '1+050'


def test_projection_interpolates_chainage_and_does_not_clamp_endpoints():
    railway = straight_map(end=1000)
    best = railway.candidates(0, 1050 * DEGREES_PER_METER)
    assert best == []
    assert railway.candidates(0, 105 * DEGREES_PER_METER)[0].pk == pytest.approx(105)


@pytest.mark.parametrize('length', [50.0, 5000.0])
def test_extreme_geographic_pk_ratios_preserve_segment_and_interpolation(length):
    railway = RailwayMap([
        Hito('A', 1000, 0, 0),
        Hito('A', 1500, 0, length * DEGREES_PER_METER),
    ])
    candidate = railway.candidates(0, length / 2 * DEGREES_PER_METER)[0]
    assert railway.discarded_segments == 0
    assert candidate.pk == pytest.approx(1250)
    assert candidate.scale == pytest.approx(500 / length)


def test_csv_validation_and_conflicting_duplicates(tmp_path):
    path = tmp_path / 'map.csv'
    path.write_text('nombre_via;PK;latitud;longitud\n'
                    'A;0+000;0;0\n'
                    f'A;0+500;0;{500 * DEGREES_PER_METER}\n'
                    'A;1+000;0;0,01\nA;1+000;0;0,02\n'
                    'A;mal;0;0\nA;2+000;nan;0\n', encoding='utf-8-sig')
    railway, discarded = RailwayMap.from_csv(path)
    assert len(railway.segments) == 1
    assert discarded == 4


def test_repository_csv_loads_without_pandas():
    path = Path(__file__).parents[1] / 'config' / 'Coordenadas_PPKK_AV.csv'
    railway, discarded = RailwayMap.from_csv(path)
    assert len(railway.segments) > 1000
    assert discarded > 0


def test_integration_interpolates_time_and_rejects_duplicate_stamps():
    motion = MotionBuffer()
    assert motion.add(0, 0, 0.01)
    assert motion.add(2, 4, 0.01)
    assert motion.latest.distance == 4
    assert motion.at(1).distance == 1
    assert not motion.add(2, 10, 0.01)
    assert not motion.add(1, 10, 0.01)
    assert not motion.add(3, float('nan'), 0.01)


def test_encoder_gap_does_not_invent_distance_and_inflates_uncertainty():
    motion = MotionBuffer()
    motion.add(0, 10, 0.01)
    motion.add(1, 10, 0.01)
    motion.add(20, 10, 0.01)
    assert motion.latest.distance == 10
    assert motion.latest.noise > 10000
    assert motion.at(10) is None


def test_start_at_rest_without_encoder_does_not_publish_a_direction():
    tracker = PKTracker(straight_map(), MotionBuffer())
    for t in range(10):
        gps(tracker, t, 100)
    assert tracker.state == 'ORIENTANDO'
    assert tracker.estimate() is None


def test_stationary_gps_jitter_does_not_orient():
    motion = MotionBuffer()
    tracker = PKTracker(straight_map(), motion)
    for t in range(20):
        motion.add(t, 0, 0.01)
        gps(tracker, t, 100 + (3 if t % 2 else -3))
    assert tracker.state == 'ORIENTANDO'


@pytest.mark.parametrize('direction,speed', [(1, 10), (-1, 10), (-1, -10), (1, -10)])
def test_orientation_calibrates_encoder_sign_against_pk(direction, speed):
    tracker, motion, initial = oriented(direction, speed)
    motion.add(6, speed, 0.01)
    assert tracker.estimate()[0] == pytest.approx(initial + direction * 60)


def test_orientation_can_cross_many_segments_without_index_lock():
    tracker, _, _ = oriented(spacing=10)
    assert tracker.state == 'AVANCE'


def test_ambiguous_parallel_tracks_wait_or_use_explicit_track():
    hitos = [Hito(track, pk, offset * DEGREES_PER_METER, pk * DEGREES_PER_METER)
             for track, offset in [('A', 0), ('B', 4)] for pk in [0, 500, 1000]]
    railway = RailwayMap(hitos)
    tracker = PKTracker(railway, MotionBuffer())
    gps(tracker, 0, 100)
    assert tracker.state == 'INIT'
    tracker = PKTracker(railway, MotionBuffer(), Settings(initial_track='B'))
    gps(tracker, 0, 100)
    assert tracker.track == 'B'


def test_gps_dropout_predicts_encoder_and_uncertainty_grows():
    tracker, motion, initial = oriented()
    _, sigma_before = tracker.estimate()
    for t in range(6, 101):
        motion.add(t, 10, 0.01)
    pk, sigma_after = tracker.estimate()
    assert pk == pytest.approx(initial + 1000)
    assert sigma_after > sigma_before
    assert gps(tracker, 100, initial + 1000)
    assert tracker.estimate()[1] < sigma_after


def test_delayed_gps_matches_on_time_update_exactly():
    timely, a, _ = oriented()
    delayed, b, _ = oriented()
    for t in range(6, 11):
        a.add(t, 10, 0.01)
        b.add(t, 10, 0.01)
        if t == 7:
            gps(timely, 6.5, 166)
    gps(delayed, 6.5, 166)
    assert delayed.estimate() == pytest.approx(timely.estimate())


def test_outliers_and_old_fixes_do_not_change_estimate():
    tracker, motion, _ = oriented()
    motion.add(6, 10, 0.01)
    before = tracker.estimate()
    assert not gps(tracker, 6, 900)
    assert not gps(tracker, 4, 140)
    assert tracker.estimate() == before


def test_signed_velocity_handles_reversal_without_gps():
    tracker, motion, _ = oriented()
    start, _ = tracker.estimate()
    motion.add(6, -10, 0.01)  # Cruce por cero: desplazamiento neto nulo.
    motion.add(7, -10, 0.01)
    assert tracker.estimate()[0] == pytest.approx(start - 10)


def test_scalar_update_has_known_analytic_solution():
    motion = MotionBuffer(process_noise=0, drift_per_meter=0)
    motion.add(0, 2, 0)
    kalman = KalmanPK(100, 4, motion.latest, 1)
    motion.add(1, 2, 0)
    assert kalman.update(104, 4, motion.latest)
    assert kalman.pk == pytest.approx(103)
    assert kalman.variance == pytest.approx(2)


def test_noisy_gps_and_encoder_bias_fuse_better_than_encoder_alone():
    tracker, motion, _ = oriented()
    rng = random.Random(20)
    fused_errors, encoder_errors = [], []
    for t in range(6, 151):
        motion.add(t, 10.1, 0.01)
        truth = 100 + 10 * t
        gps(tracker, t, truth + rng.gauss(0, 3), sigma=3)
        fused_errors.append((tracker.estimate()[0] - truth) ** 2)
        encoder_errors.append((0.1 * (t - 5)) ** 2)
    assert sum(fused_errors) < sum(encoder_errors) / 2
