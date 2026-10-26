# source /opt/ros/humble/setup.bash
# source ~/ferro/ws_puntos_kilometricos/install/setup.bash

import time
import math
import re

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

def coordenadas_a_decimal(texto):
    patron = (
        r'(\d{1,3})°\s*(\d{1,2})[\'’′]\s*'
        r'(\d+(?:[.,]\d+)?)["”″]\s*([NSWE])'
    )
    coincidencias = re.findall(patron, texto.upper())

    if len(coincidencias) != 2:
        raise ValueError("Introduce latitud y longitud en grados, minutos y segundos")

    resultado = {}
    for grados, minutos, segundos, hemisferio in coincidencias:
        grados = int(grados)
        minutos = int(minutos)
        segundos = float(segundos.replace(',', '.'))

        limite = 90 if hemisferio in 'NS' else 180
        if not (0 <= grados <= limite and 0 <= minutos < 60 and 0 <= segundos < 60):
            raise ValueError("Coordenada fuera de rango")

        valor = grados + minutos / 60 + segundos / 3600
        if hemisferio in 'SW':
            valor = -valor

        eje = 'latitud' if hemisferio in 'NS' else 'longitud'
        if eje in resultado:
            raise ValueError(f"Hay dos valores de {eje}")
        resultado[eje] = valor

    if set(resultado) != {'latitud', 'longitud'}:
        raise ValueError("Se necesita una latitud N/S y una longitud E/W")

    return resultado['latitud'], resultado['longitud']


def distancia_metros(a, b):
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    return 2 * 6371000 * math.asin(math.sqrt(h))


# Puntos de la trayectoria, en el orden de avance. A y B son los extremos.
PUNTOS = [
    coordenadas_a_decimal("""41°31'21.96"N 5°46'43.73"W"""),  # A
    coordenadas_a_decimal("""41°30'46.62"N 5°45'58.87"W"""),
    coordenadas_a_decimal("""41°30'34.63"N 5°45'20.69"W"""),
    coordenadas_a_decimal("""41°30'48.37"N 5°44'43.65"W"""),
    coordenadas_a_decimal("""41°31'30.44"N 5°43'08.83"W"""),
    coordenadas_a_decimal("""41°31'39.12"N 5°42'49.79"W"""),  # B
]

LONGITUDES = [
    distancia_metros(a, b) for a, b in zip(PUNTOS, PUNTOS[1:])
]
LONGITUD_TRAMO = sum(LONGITUDES)


def posicion_en_recorrido(distancia):
    restante = max(0.0, min(distancia, LONGITUD_TRAMO))
    for i, longitud in enumerate(LONGITUDES):
        if restante <= longitud or i == len(LONGITUDES) - 1:
            fraccion = restante / longitud
            a, b = PUNTOS[i], PUNTOS[i + 1]
            return (
                a[0] + fraccion * (b[0] - a[0]),
                a[1] + fraccion * (b[1] - a[1]),
            )
        restante -= longitud

VELOCIDAD = 17.71  # m/s
INTERVALO = 5.0   # segundos entre muestras (admite decimales)

if not math.isfinite(VELOCIDAD) or VELOCIDAD <= 0:
    raise ValueError('VELOCIDAD debe ser positiva y finita')
if not math.isfinite(INTERVALO) or INTERVALO <= 0:
    raise ValueError('INTERVALO debe ser positivo y finito')

PASOS = math.ceil(LONGITUD_TRAMO / (VELOCIDAD * INTERVALO))


rclpy.init()
node = rclpy.create_node('prueba_sintetica_madrid_frontera')
gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)
odo_pub = node.create_publisher(Odometry, '/encoder_odometry', 10)

try:
    for i in range(PASOS + 1):
        stamp = node.get_clock().now().to_msg()
        recorrido = min(i * VELOCIDAD * INTERVALO, LONGITUD_TRAMO)
        latitud, longitud = posicion_en_recorrido(recorrido)

        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.status.status = 0
        gps.latitude = latitud
        gps.longitude = longitud
        gps.position_covariance[0] = 1.0
        gps.position_covariance[4] = 1.0
        gps.position_covariance[8] = 1.0
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        odo = Odometry()
        odo.header.stamp = stamp
        odo.twist.twist.linear.x = VELOCIDAD
        odo.twist.covariance[0] = 0.01

        gps_pub.publish(gps)
        odo_pub.publish(odo)
        print(f'Muestra {i}/{PASOS}: {recorrido:.1f}/{LONGITUD_TRAMO:.1f} m', flush=True)
        if i < PASOS:
            tiempo_restante = (LONGITUD_TRAMO - recorrido) / VELOCIDAD
            time.sleep(min(INTERVALO, tiempo_restante))
finally:
    node.destroy_node()
    rclpy.shutdown()
