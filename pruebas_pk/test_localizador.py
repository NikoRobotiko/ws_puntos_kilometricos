# source /opt/ros/humble/setup.bash
# source ~/ferro/ws_puntos_kilometricos/install/setup.bash

import time
import math

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Tramo real del CSV v2: PK 100+000 a 100+400, entre Brihuega y Zaragoza.
LAT_A, LON_A = 40.835166640000, -2.873278200000
LAT_B, LON_B = 40.837223468038, -2.869294536000
LONGITUD_TRAMO=405.74

VELOCIDAD=17.71  #m/s

DURACION = math.ceil(LONGITUD_TRAMO/VELOCIDAD)


rclpy.init()
node = rclpy.create_node('prueba_sintetica_madrid_frontera')
gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)
odo_pub = node.create_publisher(Odometry, '/encoder_odometry', 10)

try:
    for i in range(DURACION + 1):
        stamp = node.get_clock().now().to_msg()
        fraccion = min(i * VELOCIDAD / LONGITUD_TRAMO, 1.0)  

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
        odo.twist.twist.linear.x = VELOCIDAD
        odo.twist.covariance[0] = 0.01

        gps_pub.publish(gps)
        odo_pub.publish(odo)
        print(f'Muestra {i}/{DURACION} publicada')
        #time.sleep(1.0)
        if i < DURACION:
            tiempo_restante = (LONGITUD_TRAMO - i * VELOCIDAD) / VELOCIDAD
            time.sleep(min(1.0, tiempo_restante))
finally:
    node.destroy_node()
    rclpy.shutdown()