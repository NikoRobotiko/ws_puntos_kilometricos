"""Conexiones candidatas y seguimiento de cambios de vía para la versión 3."""

from collections import defaultdict
import csv
from dataclasses import dataclass
import math

from puntos_kilometricos.localizacion_pk import (
    Hito, KalmanPK, PKTracker as BaseTracker, RailwayMap as BaseMap,
    Settings as BaseSettings, calcular_distancia, formatear_pk, pk_a_metros,
)

DEFAULT_CSV_V3 = 'Coordenadas_PPKK_Simples.csv'
PRINCIPAL_TRACKS_V3 = frozenset({
    '020 - LA SAGRA-TOLEDO',
    '030 - BIF. MALAGA-A.V.-MARIA ZAMBRANO',
    '040 - MADRID-CHAMARTIN-CLARA CAMPOAMOR - VALENCIA-JOAQUIM SOROLLA',
    '042 - BIF. ALBACETE - ALACANT-TERMINAL',
    '044 - BIF. JOAQUIN SOROLLA-UIC - BIF. JESUS',
    '046 - BIF. MURCIA - EL REGUERON AG KM 522,1',
    '050 - MADRID-P. DE ATOCHA ALMUDENA GRANDES-LIMITE ADIF - LFPSA',
    '054 - BIF MONCASI-BIF CANAL IMPERIAL',
    '056 - BIF LES TORRES-BIF ARTESA DE LLEIDA',
    '080 - MADRID-CHAMARTIN-CLARA CAMPOAMOR-BURGOS-ROSA MANZANO',
    '082 - BIF. COTO DA TORRE - BIF. A GRANDEIRA AG. KM. 85,0',
    '084 - BIF. VENTA DE BAÑOS - LEON',
    '354 - EL REGUERON AG KM 522,1-MURCIA DEL CARMEN',
    '982 - BIFURCACIÓN MEDINA - TABOADELA AG. KM. 234,0',
    'MADRID-P. DE ATOCHA ALMUDENA GRANDES-SEVILLA-S. JUSTA',
})
# La unión final de 056 queda pendiente por contradicción entre PK y coordenadas.
PENDING_JUNCTION_ENDPOINTS_V3 = {
    ('056 - BIF LES TORRES-BIF ARTESA DE LLEIDA', 451100),
}
PREFERRED_POINTS_V3 = {
    ('050 - MADRID-P. DE ATOCHA ALMUDENA GRANDES-LIMITE ADIF - LFPSA', 315800),
    ('050 - MADRID-P. DE ATOCHA ALMUDENA GRANDES-LIMITE ADIF - LFPSA', 434700),
}


def filtrar_puntos_invalidos(rows, max_distance=100.0, min_pk_delta=1000.0):
    """Seleccionar las vías del proyecto y descartar hitos incompatibles.

    Para coordenadas exactamente iguales con PK distintos, se elimina el PK
    mayor. También se elimina el PK mayor cuando dos hitos están a menos de
    ``max_distance`` y difieren al menos ``min_pk_delta`` metros de PK.
    Los puntos confirmados por topología tienen preferencia ante duplicados.
    """
    source_rows = list(rows)
    grouped = defaultdict(list)
    discarded = 0
    #Comprobamos coordenadas inválidas y PK de vías no seleccionadas.
    #También busca los PK con coordenadas idénticas.
    #Si los datos de los PK que nos diesen no fuesen una puta mierda esto no haría falta.
    for row in source_rows:
        try:
            track = row['nombre_via'].strip()
            pk = pk_a_metros(row['PK'])
            #Se supone que en el CSV solo están las vías principales, 
            #Pero si en un futuro se emplea otro csv, únicamente se seguirán utilizando 
            #los PK de las vías en PRINCIPAL_TRACKS_V3, descartando el resto.
            if track not in PRINCIPAL_TRACKS_V3: 
                discarded += 1
                continue
            #Hacen falta los valores de las coordenadas para luego hacer comparaciones.
            lat = float(row['latitud'].replace(',', '.'))
            lon = float(row['longitud'].replace(',', '.'))
            if not track or not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError('Coordenadas o vía inválidas')
            grouped[track, pk].append((row, lat, lon))
        except (ValueError, TypeError, AttributeError, KeyError):
            discarded += 1

    #Se comprueba si hay dos puntos de una misma vía con el mismo valor PK, por alguna razón.
    by_track = defaultdict(list)
    for (track, pk), points in grouped.items():
        first = points[0]
        if any(calcular_distancia(first[1], first[2], point[1], point[2]) > 1
               for point in points[1:]):
            discarded += len(points)
            continue
        by_track[track].append((pk, first[0], first[1], first[2]))
        discarded += len(points) - 1

    removed = set()
    for points in by_track.values():
        for index, first in enumerate(points):
            for second in points[index + 1:]:
                delta_pk = abs(first[0] - second[0])    #Difererncia en los valores PK
                same_coordinates = first[2:] == second[2:]  #True si las coordenadas de ambos puntos son idénticas.
                #Si las coordenadas son distintas y la diferencia de PK es menor que min_pk_delta, se ignora la comparación.
                if not same_coordinates and delta_pk < min_pk_delta:
                    continue
                if calcular_distancia(first[2], first[3],
                                      second[2], second[3]) >= max_distance:
                    continue
                first_key = (first[1]['nombre_via'].strip(), first[0])
                second_key = (second[1]['nombre_via'].strip(), second[0])
                first_preferred = first_key in PREFERRED_POINTS_V3
                second_preferred = second_key in PREFERRED_POINTS_V3
                if first_preferred != second_preferred:
                    higher = second if first_preferred else first
                else:
                    higher = first if first[0] > second[0] else second
                removed.add(id(higher[1]))

    valid_ids = {id(point[1]) for points in by_track.values()
                 for point in points if id(point[1]) not in removed}
    valid_rows = [row for row in source_rows if id(row) in valid_ids]
    discarded += len(removed)
    return valid_rows, discarded


@dataclass(frozen=True)
class Junction:
    """Enlace candidato entre el extremo de una vía y un hito de otra."""

    source: str
    source_pk: float
    target: str
    target_pk: float
    distance: float
    explicit: bool = False
    switch_enabled: bool = True
    pk_direction: int | None = None

    def reverse(self):
        """Permitir recorrer el mismo enlace en el sentido contrario."""
        return Junction(self.target, self.target_pk,
                        self.source, self.source_pk, self.distance,
                        self.explicit, self.switch_enabled, self.pk_direction)


class RailwayMapV3(BaseMap):
    """Conservar los tramos de v2 y añadir enlaces candidatos entre vías."""

    @classmethod
    def from_csv(cls, path):
        """Cargar CSV y excluir puntos anómalos dentro de la misma vía."""
        with open(path, encoding='utf-8-sig', newline='') as stream:
            reader = csv.DictReader(stream, delimiter=';')
            required = {'nombre_via', 'PK', 'latitud', 'longitud'}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f'Columnas requeridas: {sorted(required)}')
            rows, discarded = filtrar_puntos_invalidos(reader)
        hitos = [Hito(row['nombre_via'].strip(), pk_a_metros(row['PK']),
                      float(row['latitud'].replace(',', '.')),
                      float(row['longitud'].replace(',', '.')))
                 for row in rows]
        return cls(hitos), discarded

    def __init__(self, hitos):
        super().__init__(hitos)
        self.hitos_by_track = defaultdict(list)
        for hito in hitos:
            self.hitos_by_track[hito.track].append(hito)
        for points in self.hitos_by_track.values():
            points.sort(key=lambda p: p.pk)
        self.build_junctions(30.0)

    def build_junctions(self, junction_distance):
        """Enlazar extremos con hitos de otra vía dentro del umbral."""
        if not math.isfinite(junction_distance) or junction_distance <= 0:
            raise ValueError('junction_distance debe ser positivo y finito')
        explicit = [link for link in getattr(self, 'junctions', ()) if link.explicit]
        self.junctions = []
        self.junctions_by_track = defaultdict(list)
        seen = set()
        usable = {segment[0] for segment in self.segments}
        for track, points in self.hitos_by_track.items():
            if track not in usable:
                continue
            for endpoint in (points[0], points[-1]):
                for other, other_points in self.hitos_by_track.items():
                    if other == track or other not in usable:
                        continue
                    for hito in other_points:
                        distance = calcular_distancia(
                            endpoint.lat, endpoint.lon, hito.lat, hito.lon)
                        if distance > junction_distance:
                            continue
                        key = frozenset(((track, endpoint.pk), (other, hito.pk)))
                        if key in seen:
                            continue
                        seen.add(key)
                        pending = bool(key & PENDING_JUNCTION_ENDPOINTS_V3)
                        link = Junction(track, endpoint.pk, other, hito.pk, distance,
                                        switch_enabled=not pending)
                        self._append_junction(link)
        for link in explicit:
            self._append_junction(link)

    def _append_junction(self, link):
        key = frozenset(((link.source, link.source_pk),
                         (link.target, link.target_pk)))
        previous = next((item for item in self.junctions if
                         frozenset(((item.source, item.source_pk),
                                    (item.target, item.target_pk))) == key), None)
        if previous is not None:
            if not link.explicit or previous.explicit:
                return
            self.junctions.remove(previous)
            self.junctions_by_track[previous.source].remove(previous)
            self.junctions_by_track[previous.target].remove(previous.reverse())
        self.junctions.append(link)
        self.junctions_by_track[link.source].append(link)
        self.junctions_by_track[link.target].append(link.reverse())

    def load_explicit_junctions(self, path):
        """Cargar enlaces verificados por nombre y PK, sin corregir los hitos."""
        required = {'nombre_via_origen', 'PK_origen',
                    'nombre_via_destino', 'PK_destino', 'habilitar_cambio'}
        with open(path, encoding='utf-8-sig', newline='') as stream:
            reader = csv.DictReader(stream, delimiter=';')
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f'Columnas requeridas: {sorted(required)}')
            for line, row in enumerate(reader, 2):
                try:
                    source = row['nombre_via_origen'].strip()
                    target = row['nombre_via_destino'].strip()
                    source_pk = pk_a_metros(row['PK_origen'])
                    target_pk = pk_a_metros(row['PK_destino'])
                    enabled_text = row['habilitar_cambio'].strip().lower()
                    if enabled_text not in ('true', 'false'):
                        raise ValueError('habilitar_cambio debe ser true o false')
                    direction_text = (row.get('relacion_sentidos_pk') or '').strip()
                    if direction_text not in ('', '1', '+1', '-1'):
                        raise ValueError('relacion_sentidos_pk debe ser 1, -1 o vacío')
                    direction = int(direction_text) if direction_text else None
                    if source == target:
                        raise ValueError('Las vías deben ser distintas')
                    source_coords = self._coordinates_at_pk(source, source_pk)
                    target_coords = self._coordinates_at_pk(target, target_pk)
                except (KeyError, StopIteration, ValueError, AttributeError) as exc:
                    raise ValueError(f'Enlace inválido en línea {line}: {exc}') from exc
                link = Junction(source, source_pk, target, target_pk,
                                calcular_distancia(*source_coords, *target_coords),
                                explicit=True,
                                switch_enabled=enabled_text == 'true',
                                pk_direction=direction)
                self._append_junction(link)

    def _coordinates_at_pk(self, track, pk):
        """Obtener coordenadas de un PK medido o interpolado en la vía."""
        points = self.hitos_by_track.get(track)
        if not points:
            raise ValueError(f'Vía ausente del trazado: {track}')
        for point in points:
            if point.pk == pk:
                return point.lat, point.lon
        for first, second in zip(points, points[1:]):
            if first.pk <= pk <= second.pk:
                ratio = (pk - first.pk) / (second.pk - first.pk)
                return (first.lat + ratio * (second.lat - first.lat),
                        first.lon + ratio * (second.lon - first.lon))
        raise ValueError(
            f'PK {formatear_pk(pk)} fuera del trazado de {track}')


@dataclass
class SettingsV3(BaseSettings):
    """Extender los umbrales de v2 con los del cambio de vía."""

    junction_distance: float = 30.0
    switch_window: float = 400.0
    switch_min_distance: float = 25.0
    switch_confirmations: int = 3


class PKTrackerV3(BaseTracker):
    """Detectar cambios de vía tras confirmar GPS y movimiento coherentes."""

    def __init__(self, railway_map, motion, settings=None):
        super().__init__(railway_map, motion, settings or SettingsV3())
        self.pending_transition = None

    def _transition_options(self, candidates, predicted):
        """Encontrar observaciones tras un enlace cercano al PK previsto."""
        options = []
        for link in self.map.junctions_by_track.get(self.track, ()):
            if not link.switch_enabled:
                continue
            if abs(predicted - link.source_pk) > self.settings.switch_window:
                continue
            for candidate in candidates:
                if candidate.track != link.target:
                    continue
                offset = abs(candidate.pk - link.target_pk)
                if (self.settings.switch_min_distance <= offset
                        <= self.settings.switch_window):
                    options.append((candidate, link))
        return sorted(options, key=lambda pair: pair[0].distance)

    def _confirm_transition(self, best, link, point, sigma, time):
        """Cambiar de escala PK tras observaciones y movimiento coherentes."""
        pending = self.pending_transition
        if (pending is None or pending[0] != link or pending[4] != point.epoch
                or time - pending[3] > max(5.0, 5 * self.settings.confirmation_interval)):
            self.pending_transition = (
                link, best.pk, point.distance, time, point.epoch, 1, 0)
            return False
        _, previous_pk, previous_distance, _, _, count, direction = pending
        encoder_delta = point.distance - previous_distance
        pk_delta = best.pk - previous_pk
        if abs(encoder_delta) < 5 or abs(pk_delta) < 5:
            return False
        ratio = abs(pk_delta / encoder_delta)
        if not 0.25 <= ratio <= 2:
            self.pending_transition = None
            return False
        observed_direction = 1 if pk_delta * encoder_delta > 0 else -1
        if (link.pk_direction is not None
                and observed_direction != self.filter.direction * link.pk_direction):
            self.pending_transition = None
            return False
        if direction and direction != observed_direction:
            self.pending_transition = None
            return False
        count += 1
        if count < self.settings.switch_confirmations:
            self.pending_transition = (
                link, best.pk, point.distance, time, point.epoch,
                count, observed_direction)
            return False
        self.track = best.track
        self.filter = KalmanPK(best.pk, self._variance(best, sigma),
                               point, observed_direction)
        self.pending_transition = None
        self.last_correction_time = time
        return True

    def observe(self, time, lat, lon, sigma):
        """Aceptar GPS y confirmar un cambio si nace en un enlace cercano."""
        if not all(math.isfinite(v) for v in (time, lat, lon, sigma)) or sigma <= 0:
            return False
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return False
        if self.last_gps_time is not None and time <= self.last_gps_time:
            return False
        self.last_gps_time = time
        candidates = [p for p in self.map.candidates(
            lat, lon, None if self.state == 'AVANCE'
            else self.track or self.settings.initial_track)
            if p.distance <= self.settings.max_track_distance]
        if not candidates:
            self.pending_transition = None
            return False
        point = self.motion.at(time)
        if self.state == 'AVANCE':
            if point is None or point.time <= self.filter.motion.time:
                self.pending_transition = None
                return False
            predicted, p = self.filter.predict(point)
            current = [c for c in candidates
                       if c.track == self.track and
                       (c.pk - predicted) ** 2 <= self.settings.innovation_gate ** 2
                       * (p + self._variance(c, sigma))]
            options = self._transition_options(candidates, predicted)
            if options:
                target, link = options[0]
                current_distance = current[0].distance if current else math.inf
                other_targets = [candidate for candidate, _ in options]
                if (target.distance + self.settings.ambiguity_margin < current_distance
                        and not self._ambiguous(target, other_targets)):
                    return self._confirm_transition(target, link, point, sigma, time)
            self.pending_transition = None
            if not current:
                return False
            best = current[0]
            if self._ambiguous(best, current):
                return False
            accepted = self.filter.update(best.pk, self._variance(best, sigma),
                                          point, self.settings.innovation_gate)
            if accepted:
                self.last_correction_time = time
            return accepted
        best = candidates[0]
        if self._ambiguous(best, candidates):
            return False
        if self.state == 'INIT':
            self.track = best.track
            self.state = 'ORIENTANDO'
        if point is None:
            return False
        variance = self._variance(best, sigma)
        if self.orientation is None or point.epoch != self.orientation[1].epoch:
            self._start_orientation(best, point, variance)
            return False
        first, initial_motion, initial_variance = self.orientation
        distance = point.distance - initial_motion.distance
        travel = point.travel - initial_motion.travel
        delta_pk = best.pk - first.pk
        if travel < self.settings.min_orientation_distance:
            return False
        if abs(distance) < 0.8 * travel:
            self._start_orientation(best, point, variance)
            return False
        threshold = max(self.settings.min_gps_displacement,
                        3 * math.sqrt(variance + initial_variance))
        if abs(delta_pk) < threshold or not 0.4 <= abs(delta_pk) / travel <= 1.6:
            return False
        direction = 1 if delta_pk * distance > 0 else -1
        if direction != self.direction_candidate:
            self.direction_candidate = direction
            self.confirmations = 0
            self.last_confirmation = None
        if (self.last_confirmation is None
                or time - self.last_confirmation >= self.settings.confirmation_interval):
            self.confirmations += 1
            self.last_confirmation = time
        if self.confirmations < self.settings.required_confirmations:
            return False
        self.filter = KalmanPK(best.pk, variance, point, direction)
        self.last_correction_time = time
        self.state = 'AVANCE'
        return True
