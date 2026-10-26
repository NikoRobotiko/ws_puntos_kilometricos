"""Mapa ferroviario y fusión escalar de PK, sin dependencias de ROS."""

from bisect import bisect_left
from collections import defaultdict, deque
import csv
from dataclasses import dataclass
import math
import re


def pk_a_metros(texto):
    """Interpretar KM+MMM sin perder los ceros de los metros."""
    match = re.fullmatch(r'(\d+)\+(\d{3})', str(texto).strip()) #Comprueba si el contenido de la celda coincide con la nomenclatura PK.
    if match is None:
        raise ValueError(f'PK inválido: {texto!r}')
    return int(match[1]) * 1000 + int(match[2])


def metros_a_pk(metros):
    """Redondear a metros completos y expresar el PK como KM+MMM."""
    if not math.isfinite(metros) or metros < 0:   #Por si acaso.
        raise ValueError('PK no admitido')
    signo = '-' if metros < 0 else ''
    km, resto = divmod(int(math.floor(abs(metros) + 0.5)), 1000)
    return f'{signo}{km}+{resto:03d}'


def calcular_distancia(lat1, lon1, lat2, lon2):
    """Distancia Haversine en metros."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((phi2 - phi1) / 2) ** 2
         + math.cos(phi1) * math.cos(phi2)
         * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 6371000.0 * 2 * math.asin(math.sqrt(min(1.0, max(0.0, a))))


@dataclass(frozen=True)
class Hito:
    """Referencia del CSV, con el PK expresado en metros."""

    track: str
    pk: float
    lat: float
    lon: float


@dataclass(frozen=True)
class Proyeccion:
    """Observación longitudinal obtenida al proyectar sobre un tramo."""
    track: str
    index: int
    pk: float
    distance: float
    scale: float


class RailwayMap:
    """Trazado por vía uniendo hitos consecutivos en orden creciente de PK."""

    def __init__(self, hitos):
        groups = defaultdict(list)
        for hito in hitos:
            groups[hito.track].append(hito)
        self.segments = []
        self.discarded_segments = 0
        for track, points in groups.items():
            points.sort(key=lambda p: p.pk)
            for index, (a, b) in enumerate(zip(points, points[1:])):
                length = calcular_distancia(a.lat, a.lon, b.lat, b.lon)
                delta = b.pk - a.pk
                if delta > 0 and length >= 1:
                    self.segments.append((track, index, a, b, length))
                else:
                    self.discarded_segments += 1
        if not self.segments:
            raise ValueError('El CSV no contiene tramos utilizables')

    @classmethod
    def from_csv(cls, path):
        """Validar filas y omitir PK duplicados con coordenadas conflictivas."""
        grouped = defaultdict(list)
        discarded = 0
        with open(path, encoding='utf-8-sig', newline='') as stream:
            reader = csv.DictReader(stream, delimiter=';')
            required = {'nombre_via', 'PK', 'latitud', 'longitud'}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f'Columnas requeridas: {sorted(required)}')
            for row in reader:
                try:
                    track = row['nombre_via'].strip()
                    pk = pk_a_metros(row['PK'])
                    lat = float(row['latitud'].replace(',', '.'))
                    lon = float(row['longitud'].replace(',', '.'))
                    if not track or not (-90 <= lat <= 90 and -180 <= lon <= 180):
                        raise ValueError('Coordenadas o vía inválidas')
                    grouped[track, pk].append(Hito(track, pk, lat, lon))
                except (ValueError, TypeError, AttributeError):
                    discarded += 1
        hitos = []
        for group in grouped.values():
            first = group[0]
            if any(calcular_distancia(first.lat, first.lon, p.lat, p.lon) > 1
                   for p in group[1:]):
                discarded += len(group)
            else:
                hitos.append(first)
                discarded += len(group) - 1
        return cls(hitos), discarded

    def candidates(self, lat, lon, track=None):
        """Proyectar GPS sobre cada cuerda sin extrapolar fuera del tramo."""
        results = []
        for name, index, a, b, length in self.segments:
            if track and name != track:
                continue
            factor = 6371000.0 * math.pi / 180
            east = factor * math.cos(math.radians((a.lat + b.lat) / 2))
            bx, by = (b.lon - a.lon) * east, (b.lat - a.lat) * factor
            gx, gy = (lon - a.lon) * east, (lat - a.lat) * factor
            raw_t = (gx * bx + gy * by) / (bx * bx + by * by)
            # Evitar que un GPS fuera del tramo fije artificialmente el PK extremo.
            if not 0 <= raw_t <= 1:
                continue
            distance = math.hypot(gx - raw_t * bx, gy - raw_t * by)
            results.append(Proyeccion(
                name, index, a.pk + raw_t * (b.pk - a.pk), distance,
                (b.pk - a.pk) / length))
        return sorted(results, key=lambda p: p.distance)


@dataclass(frozen=True)
class Motion:
    """Integral del encoder y varianza de proceso acumuladas en un instante."""

    time: float
    distance: float
    travel: float
    noise: float
    speed: float
    epoch: int = 0


def _travel(v0, v1, dt):
    if v0 * v1 < 0:
        return dt * (v0 * v0 + v1 * v1) / (2 * (abs(v0) + abs(v1)))
    return (abs(v0) + abs(v1)) * dt / 2


class MotionBuffer:
    """Integrar velocidad en tiempo de medida; no integrar huecos del encoder."""

    def __init__(self, history=30.0, max_gap=2.0, process_noise=0.1,
                 drift_per_meter=0.02, max_speed=100.0):
        self.samples = deque()
        self.history = history
        self.max_gap = max_gap
        self.process_noise = process_noise
        self.drift_per_meter = drift_per_meter
        self.max_speed = max_speed

    @property
    def latest(self):
        """Última integral disponible."""
        return self.samples[-1] if self.samples else None

    def add(self, time, speed, speed_variance):
        """Rechazar marcas repetidas, valores no finitos y velocidades imposibles."""
        if (not all(math.isfinite(v) for v in (time, speed, speed_variance))
                or speed_variance < 0 or abs(speed) > self.max_speed):
            return False
        last = self.latest
        if last is None:
            point = Motion(time, 0.0, 0.0, 0.0, speed)
        else:
            dt = time - last.time
            if dt <= 0:
                return False
            if dt > self.max_gap:
                point = Motion(time, last.distance, last.travel,
                               last.noise + (self.max_speed * dt) ** 2,
                               speed, last.epoch + 1)
            else:
                travel = _travel(last.speed, speed, dt)
                noise = (self.process_noise * dt + speed_variance * dt * dt
                         + self.drift_per_meter * travel)
                point = Motion(time, last.distance + (last.speed + speed) * dt / 2,
                               last.travel + travel, last.noise + noise,
                               speed, last.epoch)
        self.samples.append(point)
        while len(self.samples) > 2 and self.samples[1].time < time - self.history:
            self.samples.popleft()
        return True

    def at(self, time):
        """Interpolar solo con encoder disponible a ambos lados de la medida."""
        if not self.samples or time < self.samples[0].time or time > self.latest.time:
            return None
        index = bisect_left([p.time for p in self.samples], time)
        b = self.samples[index]
        if b.time == time:
            return b
        a = self.samples[index - 1]
        if a.epoch != b.epoch:
            return None
        dt = time - a.time
        t = dt / (b.time - a.time)
        speed = a.speed + t * (b.speed - a.speed)
        return Motion(time, a.distance + (a.speed + speed) * dt / 2,
                      a.travel + _travel(a.speed, speed, dt),
                      a.noise + t * (b.noise - a.noise), speed, a.epoch)


class KalmanPK:
    """Estado escalar x=PK [m], covarianza P [m²], control=desplazamiento."""

    def __init__(self, pk, variance, motion, direction):
        self.pk = pk
        self.variance = variance
        self.motion = motion
        self.direction = direction

    def predict(self, motion):
        """Transportar el ancla GPS a un instante posterior usando encoder."""
        return (self.pk + self.direction * (motion.distance - self.motion.distance),
                self.variance + max(0.0, motion.noise - self.motion.noise))

    def update(self, pk, variance, motion, gate=4.0):
        """Corregir en el instante GPS; recalcular luego las predicciones posteriores."""
        if motion.time <= self.motion.time:
            return False
        predicted, p = self.predict(motion)
        innovation = pk - predicted
        if innovation * innovation > gate * gate * (p + variance):
            return False
        gain = p / (p + variance)
        self.pk = predicted + gain * innovation
        self.variance = (1 - gain) ** 2 * p + gain * gain * variance
        self.motion = motion
        return True


@dataclass
class Settings:
    """Umbrales y ruidos iniciales que deben calibrarse con datos del vehículo."""

    max_track_distance: float = 30.0
    ambiguity_margin: float = 10.0
    map_sigma: float = 10.0
    min_orientation_distance: float = 20.0
    min_gps_displacement: float = 8.0
    required_confirmations: int = 3
    confirmation_interval: float = 1.0
    innovation_gate: float = 4.0
    initial_track: str = ''


class PKTracker:
    """Identificar la vía, orientar y fusionar GPS con encoder en esa vía."""

    def __init__(self, railway_map, motion, settings=None):
        self.map = railway_map
        self.motion = motion
        self.settings = settings or Settings()
        self.state = 'INIT'
        self.track = None
        self.orientation = None
        self.direction_candidate = 0
        self.confirmations = 0
        self.last_confirmation = None
        self.filter = None
        self.last_gps_time = None
        self.last_correction_time = None

    def _variance(self, candidate, sigma):
        return (candidate.scale * sigma) ** 2 + self.settings.map_sigma ** 2

    def _ambiguous(self, best, candidates):
        return any((p.track != best.track or abs(p.index - best.index) > 1)
                   and p.distance - best.distance < self.settings.ambiguity_margin
                   for p in candidates if p != best)

    def observe(self, time, lat, lon, sigma):
        """Aceptar GPS en orden de adquisición, aunque lleguen con retraso."""
        if not all(math.isfinite(v) for v in (time, lat, lon, sigma)) or sigma <= 0:
            return False
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return False
        if self.last_gps_time is not None and time <= self.last_gps_time:
            return False
        self.last_gps_time = time
        candidates = [p for p in self.map.candidates(
            lat, lon, self.track or self.settings.initial_track)
            if p.distance <= self.settings.max_track_distance]
        if not candidates:
            return False
        point = self.motion.at(time)
        if self.state == 'AVANCE':
            if point is None or point.time <= self.filter.motion.time:
                return False
            predicted, p = self.filter.predict(point)
            candidates = [c for c in candidates if
                          (c.pk - predicted) ** 2 <= self.settings.innovation_gate ** 2
                          * (p + self._variance(c, sigma))]
            if not candidates:
                return False
            best = candidates[0]
            if self._ambiguous(best, candidates):
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

    def _start_orientation(self, best, point, variance):
        self.orientation = best, point, variance
        self.direction_candidate = 0
        self.confirmations = 0
        self.last_confirmation = None

    def estimate(self):
        """PK y sigma actuales; conservar el ancla GPS durante pérdidas de señal."""
        if self.filter is None:
            return None
        pk, variance = self.filter.predict(self.motion.latest)
        return pk, math.sqrt(variance)
