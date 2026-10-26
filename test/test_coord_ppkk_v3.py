"""Pruebas de enlaces y cambios de vía de la versión 3."""

import csv
import math
from pathlib import Path

import pytest

from puntos_kilometricos.localizacion_pk import Hito, MotionBuffer, Projection
from puntos_kilometricos.localizacion_pk_v3 import (
    DEFAULT_CSV_V3, PKTrackerV3, RailwayMapV3, SettingsV3,
)


DEGREES_PER_METER = 180 / (math.pi * 6371000)


def branching_map(reverse_pk=False):
    """Construir una vía A recta y una B que nace en el PK 500 de A."""
    return RailwayMapV3([
        Hito('A', pk, 0, pk * DEGREES_PER_METER)
        for pk in (0, 500, 1000)
    ] + [
        Hito('B', 54000 - pk if reverse_pk else pk,
             0.62 * pk * DEGREES_PER_METER,
             (500 + 0.78 * pk) * DEGREES_PER_METER)
        for pk in (0, 100, 200, 300)
    ])


def branch_fix(tracker, time, pk):
    """Observar la posición sobre el ramal B."""
    return tracker.observe(time, 0.62 * pk * DEGREES_PER_METER,
                           (500 + 0.78 * pk) * DEGREES_PER_METER, 1)


def main_fix(tracker, time, pk):
    """Observar la posición sobre la vía principal A."""
    return tracker.observe(time, 0, pk * DEGREES_PER_METER, 1)


@pytest.mark.parametrize('filename', [DEFAULT_CSV_V3, 'Coordenadas_PPKK_AV.csv'])
def test_csv_selects_the_fifteen_tracks_and_only_links_selected_tracks(filename):
    path = Path(__file__).parents[1] / 'config' / filename
    railway, _ = RailwayMapV3.from_csv(path)
    selected = set(railway.hitos_by_track)
    assert {track[:3] for track in selected} == {
        '020', '030', '040', '042', '044', '046', '050', '054',
        '056', '080', '082', '084', '354', '982', 'MAD',
    }
    railway.load_explicit_junctions(path.parent / 'Enlaces_vias_v3.csv')
    assert all(link.source in selected and link.target in selected
               for link in railway.junctions)


def test_simple_csv_points_are_preserved_in_generated_csv(tmp_path):
    from puntos_kilometricos.generar_csv_validos_v3 import main

    source = Path(__file__).parents[1] / 'config' / DEFAULT_CSV_V3
    original = source.read_bytes()
    output = tmp_path / 'validos.csv'
    main(['--entrada', str(source), '--salida', str(output)])
    with source.open(encoding='utf-8-sig', newline='') as stream:
        expected = list(csv.reader(stream, delimiter=';'))
    with output.open(encoding='utf-8-sig', newline='') as stream:
        actual = list(csv.reader(stream, delimiter=';'))
    assert actual == expected
    assert source.read_bytes() == original


def test_v3_defaults_to_simple_csv_and_v2_keeps_its_default():
    from puntos_kilometricos.coord_ppkk_v2 import Coordenadas_a_PPKK as V2
    from puntos_kilometricos.coord_ppkk_v3 import Coordenadas_a_PPKK as V3

    assert V2.default_csv_filename == 'Coordenadas_PPKK_AV.csv'
    assert V3.default_csv_filename == 'Coordenadas_PPKK_Simples.csv'


def test_pending_056_endpoint_cannot_trigger_an_automatic_switch():
    root = Path(__file__).parents[1] / 'config'
    railway, _ = RailwayMapV3.from_csv(root / DEFAULT_CSV_V3)
    railway.load_explicit_junctions(root / 'Enlaces_vias_v3.csv')
    branch = next(t for t in railway.hitos_by_track if t.startswith('056 - '))
    trunk = next(t for t in railway.hitos_by_track if t.startswith('050 - '))
    links = [link for link in railway.junctions_by_track[branch]
             if link.source_pk == 451100]
    assert links
    assert all(not link.switch_enabled for link in links)
    tracker = PKTrackerV3(railway, MotionBuffer())
    tracker.track = branch
    candidate = Projection(trunk, 0, 448300, 0, 1)
    assert tracker._transition_options([candidate], 451100) == []
    railway.build_junctions(100)
    assert all(not link.switch_enabled for link in railway.junctions_by_track[branch]
               if link.source_pk == 451100)


def test_missing_track_lookup_does_not_add_an_empty_track_to_the_map():
    railway = branching_map()
    before = set(railway.hitos_by_track)
    with pytest.raises(ValueError, match='Vía ausente'):
        railway._coordinates_at_pk('absent', 0)
    assert set(railway.hitos_by_track) == before


def test_verified_venta_de_banos_junction_connects_interior_points():
    root = Path(__file__).parents[1] / 'config'
    railway, _ = RailwayMapV3.from_csv(root / DEFAULT_CSV_V3)
    trunk = next(t for t in railway.hitos_by_track if t.startswith('080 - '))
    branch = next(t for t in railway.hitos_by_track if t.startswith('084 - '))
    assert not any(link.target == branch
                   for link in railway.junctions_by_track[trunk])

    railway.load_explicit_junctions(root / 'Enlaces_vias_v3.csv')
    link = next(link for link in railway.junctions_by_track[trunk]
                if link.target == branch)
    assert (link.source_pk, link.target_pk) == (218000, 218000)
    assert link.explicit
    assert not link.switch_enabled
    assert link.distance == pytest.approx(59.7, abs=0.2)
    assert link.reverse() in railway.junctions_by_track[branch]
    tracker = PKTrackerV3(railway, MotionBuffer())
    tracker.track = trunk
    candidate = Projection(branch, 0, 218100, 0, 1)
    assert tracker._transition_options([candidate], 218000) == []
    railway.build_junctions(30)
    assert link in railway.junctions_by_track[trunk]


def test_confirmed_main_junctions_and_toledo_direction_are_loaded():
    root = Path(__file__).parents[1] / 'config'
    railway, _ = RailwayMapV3.from_csv(root / DEFAULT_CSV_V3)
    railway.load_explicit_junctions(root / 'Enlaces_vias_v3.csv')
    links = {(link.source[:3], link.source_pk, link.target[:3], link.target_pk): link
             for link in railway.junctions if link.explicit}
    for key in [('982', 133900, '080', 133974),
                ('046', 461400, '042', 461062),
                ('020', 54000, 'MAD', 54000)]:
        assert links[key].switch_enabled
        assert links[key].distance < 10
    toledo = links['020', 54000, 'MAD', 54000]
    assert toledo.pk_direction == -1
    assert toledo.reverse().pk_direction == -1


def configured_toledo_map(tmp_path, relation=-1):
    """Configurar un ramal descendente cuya escala PK comienza en 54+000."""
    railway = branching_map(reverse_pk=True)
    config = tmp_path / 'enlaces.csv'
    config.write_text(
        'nombre_via_origen;PK_origen;nombre_via_destino;PK_destino;'
        'habilitar_cambio;relacion_sentidos_pk\n'
        f'A;0+500;B;54+000;true;{relation}\n', encoding='utf-8')
    railway.load_explicit_junctions(config)
    return railway


@pytest.mark.parametrize('relation, switches', [(-1, True), (1, False)])
def test_toledo_switch_changes_pk_direction_only_as_configured(tmp_path, relation, switches):
    railway = configured_toledo_map(tmp_path, relation)
    motion = MotionBuffer()
    tracker = PKTrackerV3(railway, motion, SettingsV3(map_sigma=1))
    for time in range(16):
        motion.add(time, 10, 0.01)
        main_fix(tracker, time, 350 + 10 * time)
    assert tracker.filter.direction == 1
    for time in range(16, 21):
        motion.add(time, 10, 0.01)
        branch_fix(tracker, time, 10 * (time - 15))
    if switches:
        assert tracker.track == 'B'
        assert tracker.filter.direction == -1
        assert tracker.estimate()[0] == pytest.approx(53950, abs=1)
        motion.add(21, 10, 0.01)
        assert tracker.estimate()[0] == pytest.approx(53940, abs=1)
    else:
        assert tracker.track == 'A'


def test_return_from_toledo_toward_madrid_decreases_main_line_pk(tmp_path):
    railway = configured_toledo_map(tmp_path)
    motion = MotionBuffer()
    tracker = PKTrackerV3(railway, motion, SettingsV3(map_sigma=1, initial_track='B'))
    for time in range(16):
        motion.add(time, 10, 0.01)
        branch_fix(tracker, time, 150 - 10 * time)
    assert tracker.filter.direction == 1
    for time in range(16, 21):
        motion.add(time, 10, 0.01)
        main_fix(tracker, time, 500 - 10 * (time - 15))
    assert tracker.track == 'A'
    assert tracker.filter.direction == -1
    assert tracker.estimate()[0] == pytest.approx(450, abs=1)
    motion.add(21, 10, 0.01)
    assert tracker.estimate()[0] == pytest.approx(440, abs=1)


def test_external_integer_pk_refinements_between_hitos_use_legacy_csv_format(tmp_path):
    railway = branching_map()
    config = tmp_path / 'enlaces_medidos.csv'
    config.write_text(
        'nombre_via_origen;PK_origen;nombre_via_destino;PK_destino;habilitar_cambio\n'
        'A;0+510;B;0+013;true\n', encoding='utf-8')
    railway.load_explicit_junctions(config)
    link = next(link for link in railway.junctions if link.explicit)
    assert (link.source_pk, link.target_pk) == (510, 13)
    assert link.switch_enabled
    assert link.pk_direction is None
    assert link.distance == pytest.approx(8.06, abs=0.01)


def test_branch_switches_track_and_rebases_pk_after_consistent_gps():
    railway = branching_map()
    assert any(link.source == 'B' and link.source_pk == 0
               and link.target == 'A' and link.target_pk == 500
               for link in railway.junctions)
    motion = MotionBuffer()
    tracker = PKTrackerV3(railway, motion, SettingsV3(map_sigma=1))
    for time in range(16):
        motion.add(time, 10, 0.01)
        main_fix(tracker, time, 350 + 10 * time)
    assert tracker.state == 'AVANCE'
    assert tracker.track == 'A'
    for time in range(16, 21):
        motion.add(time, 10, 0.01)
        branch_fix(tracker, time, 10 * (time - 15))
        if time < 20:
            assert tracker.track == 'A'
    assert tracker.track == 'B'
    assert tracker.estimate()[0] == pytest.approx(50, abs=1)
    motion.add(21, 10, 0.01)
    assert tracker.estimate()[0] == pytest.approx(60, abs=1)


def test_staying_on_main_line_does_not_switch_at_branch():
    railway = branching_map()
    motion = MotionBuffer()
    tracker = PKTrackerV3(railway, motion, SettingsV3(map_sigma=1))
    for time in range(31):
        motion.add(time, 10, 0.01)
        main_fix(tracker, time, 350 + 10 * time)
    assert tracker.track == 'A'
    assert tracker.estimate()[0] == pytest.approx(650, abs=1)


def test_switching_back_from_branch_to_main_line():
    railway = branching_map()
    motion = MotionBuffer()
    tracker = PKTrackerV3(railway, motion,
                          SettingsV3(map_sigma=1, initial_track='B'))
    for time in range(16):
        motion.add(time, 10, 0.01)
        branch_fix(tracker, time, 150 - 10 * time)
    assert tracker.state == 'AVANCE'
    assert tracker.track == 'B'
    for time in range(16, 21):
        motion.add(time, 10, 0.01)
        main_fix(tracker, time, 500 + 10 * (time - 15))
    assert tracker.track == 'A'
    assert tracker.estimate()[0] == pytest.approx(550, abs=1)


def test_no_link_when_endpoints_are_farther_than_tolerance():
    railway = RailwayMapV3([
        Hito('A', 0, 0, 0),
        Hito('A', 500, 0, 500 * DEGREES_PER_METER),
        Hito('B', 0, 31 * DEGREES_PER_METER, 500 * DEGREES_PER_METER),
        Hito('B', 500, 31 * DEGREES_PER_METER, 1000 * DEGREES_PER_METER),
    ])
    assert railway.junctions == []
    railway.build_junctions(32)
    assert any(link.source == 'A' and link.target == 'B'
               for link in railway.junctions)
