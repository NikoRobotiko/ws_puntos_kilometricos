import math
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Tramo la vía 050.
# LAT_A, LON_A = 40.835166640000, -2.873278200000
# LAT_B, LON_B = 40.837223468038, -2.869294536000
#Tramo de la vía Madrid - Ourense
LAT_A, LON_A = 41.4826361, -5.4259361111111115     #41°28'57.49"N 5°25'33.37"W
LAT_B, LON_B = 41.4787528, -5.418016666666667       #41°28'43.51"N 5°25'04.86"W
#LONGITUD_TRAMO = 405.74  # metros entre A y B
LONGITUD_TRAMO = 789.74

VELOCIDAD_IDA = 21.61    # m/s
VELOCIDAD_VUELTA = 18.2  # m/s

if not (0 < VELOCIDAD_IDA <= 100 and 0 < VELOCIDAD_VUELTA <= 100):
    raise ValueError('Las velocidades deben estar entre 0 y 100 m/s')

rclpy.init()
#node = rclpy.create_node('prueba_sintetica_madrid_frontera')
node = rclpy.create_node('prueba_sintetica_madrid_zamora')
gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)
odo_pub = node.create_publisher(Odometry, '/encoder_odometry', 10)


def publicar(fraccion, velocidad_encoder):
    stamp = node.get_clock().now().to_msg()

    gps = NavSatFix()
    gps.header.stamp = stamp
    gps.status.status = 0
    gps.latitude = LAT_A + fraccion * (LAT_B - LAT_A)
    gps.longitude = LON_A + fraccion * (LON_B - LON_A)
    gps.position_covariance[0] = 1.0
    gps.position_covariance[4] = 1.0
    gps.position_covariance[8] = 1.0
    gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    odo = Odometry()
    odo.header.stamp = stamp
    odo.twist.twist.linear.x = velocidad_encoder
    odo.twist.covariance[0] = 0.01

    gps_pub.publish(gps)
    odo_pub.publish(odo)


def recorrer(sentido, velocidad):
    pasos = math.ceil(LONGITUD_TRAMO / velocidad)

    for i in range(pasos + 1):
        recorrido = min(i * velocidad, LONGITUD_TRAMO)
        fraccion = recorrido / LONGITUD_TRAMO

        if sentido < 0:
            fraccion = 1.0 - fraccion

        publicar(fraccion, sentido * velocidad)
        print(
            f'{"Ida" if sentido > 0 else "Vuelta"}: '
            f'{recorrido:.1f}/{LONGITUD_TRAMO:.1f} m',
            flush=True,
        )

        if i < pasos:
            restante = (LONGITUD_TRAMO - recorrido) / velocidad
            time.sleep(min(1.0, restante))


try:
    recorrer(sentido=1, velocidad=VELOCIDAD_IDA)

    # Evita que el nodo integre una velocidad positiva y otra negativa
    # como si fueran una sola medición durante el cambio de sentido.
    print('Extremo alcanzado; espera de 3 segundos antes de volver.')
    time.sleep(3.0)

    recorrer(sentido=-1, velocidad=VELOCIDAD_VUELTA)
    time.sleep(1.0)
finally:
    node.destroy_node()
    rclpy.shutdown()