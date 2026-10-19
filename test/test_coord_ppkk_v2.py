"""Comprobar los adaptadores ROS con mensajes reales y sin iniciar transporte DDS."""

from collections import deque
import json
import math
from types import SimpleNamespace

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from puntos_kilometricos.coord_ppkk_v2 import Coordenadas_a_PPKK
from puntos_kilometricos.localizacion_pk import (
    Hito, MotionBuffer, PKTracker, RailwayMap, Settings,
)


class Sink:
    """Recoger las publicaciones sin depender del descubrimiento de DDS."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Adapter:
    """Ejecutar los callbacks reales con reloj y publicadores controlados."""

    validar_gps = Coordenadas_a_PPKK.validar_gps
    callback_gps = Coordenadas_a_PPKK.callback_gps
    callback_odometria = Coordenadas_a_PPKK.callback_odometria
    _process_gps = Coordenadas_a_PPKK._process_gps
    publish_estimate = Coordenadas_a_PPKK.publish_estimate
    publish_status = Coordenadas_a_PPKK.publish_status

    def __init__(self):
        self.factor = 180 / (math.pi * 6371000)
        self.time = 0
        railway = RailwayMap([Hito('A', pk, 0, pk * self.factor)
                              for pk in (0, 500, 1000)])
        self.motion = MotionBuffer()
        self.tracker = PKTracker(railway, self.motion, Settings(map_sigma=1))
        self.config = {'encoder_signed': False, 'encoder_sigma': 0.1,
                       'default_gps_sigma': 5, 'max_gps_sigma': 10, 'max_encoder_gap': 2}
        self.pending_gps = deque(maxlen=100)
        self.last_encoder_received = None
        for name in ('pub_pk', 'pub_metros', 'pub_sigma', 'pub_estado'):
            setattr(self, name, Sink())

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.time * 10**9))

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None)

    def fix(self, time, pk):
        msg = NavSatFix()
        msg.header.stamp.sec = time
        msg.latitude = 0.0
        msg.longitude = pk * self.factor
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance[0] = 1.0
        msg.position_covariance[4] = 1.0
        return msg

    def odometry(self, time, speed):
        msg = Odometry()
        msg.header.stamp.sec = time
        msg.twist.twist.linear.x = speed
        # Debe ignorarse: pose.x no representa distancia sobre la vía.
        msg.pose.pose.position.x = 90000.0
        return msg


def test_real_messages_queue_future_fix_and_publish_meter_precision():
    node = Adapter()
    node.callback_gps(node.fix(0, 100))
    assert node.tracker.state == 'ORIENTANDO'
    assert not node.pub_pk.messages
    for time in range(1, 10):
        node.time = time
        node.callback_gps(node.fix(time, 100 + 10 * time))
        node.callback_odometria(node.odometry(time, 10.0))
    assert node.tracker.state == 'AVANCE'
    assert abs(node.pub_metros.messages[-1].data - 190) < 1e-6
    assert node.pub_pk.messages[-1].data == 'A | PK: 0+190'
    assert not node.pending_gps
    node.time = 15
    node.publish_status()
    status = json.loads(node.pub_estado.messages[-1].data)
    assert status['encoder_obsoleto']
    assert status['stamp_estimacion'] == 9


def test_invalid_fix_and_covariance_do_not_enter_queue():
    node = Adapter()
    for variance in (-1.0, float('nan'), 400.0):
        msg = node.fix(0, 100)
        msg.position_covariance[0] = variance
        node.callback_gps(msg)
    msg = node.fix(0, 100)
    msg.status.status = -1
    node.callback_gps(msg)
    assert node.tracker.state == 'INIT'
    assert not node.pending_gps
