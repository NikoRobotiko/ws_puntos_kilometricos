"""Nodo ROS 2 para localizar un vehículo ferroviario mediante GPS y encoder."""

from collections import deque
from dataclasses import fields
import json
import math
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String

from puntos_kilometricos.localizacion_pk import (
    MotionBuffer, PKTracker, RailwayMap, Settings, metros_a_pk,
)


def stamp_seconds(msg):
    """Usar tiempo de adquisición, incluido cero al comienzo de simulaciones."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Coordenadas_a_PPKK(Node):
    """Publicar PK con resolución de un metro y su incertidumbre estimada."""

    #default_csv_filename = 'Coordenadas_PPKK_AV.csv'
    default_csv_filename = 'Coordenadas_PPKK_Simples.csv'

    def __init__(self):
        super().__init__('coord_ppkk_node')
        try:
            csv_dir = get_package_share_directory('puntos_kilometricos')
        except PackageNotFoundError:
            csv_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
        default_csv = os.path.join(csv_dir, self.default_csv_filename)
        self.declare_parameter('ruta_csv', default_csv)
        defaults = Settings()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        self.settings = Settings(**{
            field.name: self.get_parameter(field.name).value for field in fields(defaults)})
        parameters = {
            'encoder_signed': True,
            'encoder_sigma': 0.1,
            'encoder_drift_variance_per_meter': 0.02,
            'process_variance_per_second': 0.1,
            'history_seconds': 30.0,
            'max_encoder_gap': 20.0,
            'max_speed': 100.0,
            'default_gps_sigma': 5.0,
            'max_gps_sigma': 10.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in parameters}
        for name, value in self.config.items():
            if name != 'encoder_signed' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} debe ser positivo y finito')
        for field in fields(defaults):
            value = getattr(self.settings, field.name)
            if field.name != 'initial_track' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{field.name} debe ser positivo y finito')
        mapa_red_av, discarded = RailwayMap.from_csv(
            self.get_parameter('ruta_csv').value)
        if self.settings.initial_track and not any(
                s[0] == self.settings.initial_track for s in mapa_red_av.segments):
            raise ValueError('initial_track no corresponde a una vía con tramos válidos')
        if discarded:
            self.get_logger().warning(f'CSV: {discarded} filas descartadas')
        self.get_logger().info(
            f'Mapa: {len(mapa_red_av.segments)} tramos utilizables; '
            f'{mapa_red_av.discarded_segments} tramos sin geometría utilizable omitidos')
        self.motion = MotionBuffer(
            history=self.config['history_seconds'], max_gap=self.config['max_encoder_gap'],
            process_noise=self.config['process_variance_per_second'],
            drift_per_meter=self.config['encoder_drift_variance_per_meter'],
            max_speed=self.config['max_speed'])
        self.tracker = PKTracker(mapa_red_av, self.motion, self.settings)
        self.pending_gps = deque(maxlen=100)
        self.last_encoder_received = None
        self.pub_pk = self.create_publisher(String, '/punto_kilometrico', 10)
        self.pub_metros = self.create_publisher(Float64, '/pk_metros', 10)
        self.pub_sigma = self.create_publisher(Float64, '/pk_sigma', 10)
        self.pub_estado = self.create_publisher(String, '/pk_estado', 10)
        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/fix', self.callback_gps, qos_profile_sensor_data)
        self.sub_odo = self.create_subscription(
            Odometry, '/encoder_odometry', self.callback_odometria, qos_profile_sensor_data)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Esperando GPS para identificar la vía y orientar el vehículo')

    def validar_gps(self, msg):
        """Obtener sigma horizontal conservadora desde la covarianza ENU."""
        if (msg.status.status < 0 
                or not math.isfinite(msg.latitude)
                or not math.isfinite(msg.longitude)
                or not -90 <= msg.latitude <= 90 or not -180 <= msg.longitude <= 180):
            return False, None
        
        sigma = self.config['default_gps_sigma']

        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            e, n = msg.position_covariance[0], msg.position_covariance[4]
            en = (msg.position_covariance[1] + msg.position_covariance[3]) / 2
            if (not all(math.isfinite(v) for v in (e, n, en))
                    or e <= 0 or n <= 0 or en * en > e * n):
                return False, None
            sigma = math.sqrt((e + n + math.hypot(e - n, 2 * en)) / 2)
        return sigma <= self.config['max_gps_sigma'], sigma

    def callback_odometria(self, msg):
        """Integrar twist.linear.x [m/s]; pose.position.x no es un cuentakilómetros."""
        speed = msg.twist.twist.linear.x
        if not self.config['encoder_signed']:
            speed = abs(speed)
        reported = msg.twist.covariance[0]
        if not math.isfinite(reported) or reported < 0:
            return
        variance = max(reported, self.config['encoder_sigma'] ** 2)
        if not self.motion.add(stamp_seconds(msg), speed, variance):
            return
        self.last_encoder_received = self.get_clock().now().nanoseconds * 1e-9
        self._process_gps()
        self.publish_estimate()

    def callback_gps(self, msg):
        """Encolar observaciones hasta disponer de encoder en el instante GPS."""
        valid, sigma = self.validar_gps(msg)
        if not valid:
            return
        observation = (stamp_seconds(msg), msg.latitude, msg.longitude, sigma)
        # En reposo se puede identificar la vía sin disponer aún de encoder.
        if self.motion.latest is None:
            self.tracker.observe(*observation)
            self.publish_status()
            return
        self.pending_gps.append(observation)
        self.pending_gps = deque(sorted(self.pending_gps), maxlen=100)
        self._process_gps()
        self.publish_estimate()

    def _process_gps(self):
        old_state = self.tracker.state
        while self.pending_gps and self.pending_gps[0][0] <= self.motion.latest.time:
            self.tracker.observe(*self.pending_gps.popleft())
        if old_state != self.tracker.state:
            self.get_logger().info(f'Estado: {self.tracker.state}; vía: {self.tracker.track}')

    def publish_estimate(self):
        """Publicar valor continuo y texto redondeado a un metro."""
        estimate = self.tracker.estimate()
        if estimate is None:
            return
        pk, sigma = estimate
        self.pub_pk.publish(String(data=f'{self.tracker.track} | PK: {metros_a_pk(pk)}'))
        self.pub_metros.publish(Float64(data=pk))
        self.pub_sigma.publish(Float64(data=sigma))

    def publish_status(self):
        """Indicar antigüedad, orientación e incertidumbre, incluso sin encoder."""
        now = self.get_clock().now().nanoseconds * 1e-9
        latest = self.motion.latest
        estimate = self.tracker.estimate()
        stale = (self.last_encoder_received is None
                 or now - self.last_encoder_received > self.config['max_encoder_gap']
                 or (latest is not None
                     and now - latest.time > self.config['max_encoder_gap']))
        self.pub_estado.publish(String(data=json.dumps({
            'estado': self.tracker.state,
            'via': self.tracker.track,
            'stamp_estimacion': latest.time if latest else None,
            'stamp_correccion_gps': self.tracker.last_correction_time,
            'sentido_encoder_a_pk': self.tracker.filter.direction
            if self.tracker.filter else None,
            'encoder_obsoleto': stale,
            'pk_metros': estimate[0] if estimate else None,
            'sigma_metros': estimate[1] if estimate else None,
        }, ensure_ascii=False)))


def main(args=None):
    """Ejecutar el nodo y liberar recursos al finalizar."""
    rclpy.init(args=args)
    node = None
    try:
        node = Coordenadas_a_PPKK()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
