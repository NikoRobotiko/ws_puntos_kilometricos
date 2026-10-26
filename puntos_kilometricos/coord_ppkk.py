import math
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os
import re

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

def calcular_distancia(lat1, lon1, lat2, lon2):
    "Calcula la distancia geodésica entre dos Coordenadas GPS"
    r=6371000.0
    phi1=math.radians(lat1)
    phi2=math.radians(lat2)
    delta_phi=math.radians(lat2-lat1)
    delta_lambda=math.radians(lon2-lon1)

    a = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c

class Coordenadas_a_PPKK(Node):
    def __init__(self):
        super().__init__('coord_ppkk_node')

        #Obtener la ruta del archivo csv de los PK.
        try:
            pkg_share=get_package_share_directory('puntos_kilometricos')
            ruta_csv_defecto=os.path.join(pkg_share, 'Coordenadas_PPKK_AV.csv')
        except Exception:
            ruta_csv_defecto='Coordenadas_PPKK_AV.csv'
        self.declare_parameter('ruta_csv', ruta_csv_defecto)
        ruta_csv=(
            self.get_parameter('ruta_csv').get_parameter_value().string_value
        )

        #Cargar la base de datos. Se extraen los puntos kilométricos
        self.get_logger().info(f'Cargando base de datos de Puntos Kilometricos: {ruta_csv}')
        self.bd_pks=pd.read_csv(ruta_csv, sep=";", decimal=",")

        #Convertir PK en formato texto a decimal
        self.bd_pks['PK_num']=self.bd_pks['PK'].apply(self.pk_a_decimal)

        num_invalidos=self.bd_pks['PK_num'].isna().sum()

        if num_invalidos>0:
            self.get_logger().warn(f'Se han descartado {num_invalidos} puntos kilometricos por formato incorrecto.')

        self.bd_pks=self.bd_pks.dropna(subset=['PK_num'])
        #SUSTITUIR#
        # self.bd_pks['PK_num']=self.bd_pks['PK'].apply(
        #     lambda x: (
        #         float(x.replace('+', '.')) if pd.notna(x) and '+' in x else None    #Sustituye el '+' por un '.' y lo convierte a float.
        #     )
        # )
        
        #Variables de control de la máquina de estados
        self.estado = 'INIT'  # Máquina de estados: INIT -> ORIENTACION -> AVANCE
        self.hitos_cercanos = []
        self.distancias_iniciales = {}

        self.via_actual = None  #Aquí es donde se guarda el nombre de la vía en la que se está, ej. Madrid-Burgos.
        self.pk_referencia_base = None
        self.odometria_base = None
        self.sentido_marcha = 1  # 1 (PKs ascendentes), -1 (PKs descendentes)

        self.odometria_actual = 0.0

        #Suscriptores
        #Hay qe cambiar el nombre de los topics por los reales
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.callback_gps, 10)
        self.sub_odo = self.create_subscription(Odometry, '/encoder_odometry', self.callback_odometria, 10)
        #Publicadores
        #Hay que cambiar el nombre del topic
        self.pub_pk = self.create_publisher(String, '/punto_kilometrico', 10)

        self.get_logger().info('Conversor PK iniciado. Esperando señal GPS (Estado: INIT)')

    def callback_odometria(self, msg):
        """Calback de odometría: Reciba la distancia recorrida acumulada del vehículo."""
        self.odometria_actual=msg.pose.pose.position.x
        #En modo AVANCE se calcula el avance a apartir de odometria pura
        if self.estado=='AVANCE':
            distancia_recorrida=self.odometria_actual+self.odometria_base

            #Operacion: nuevo PK = Pk_inicial + (distancia recorrida(km+m) * sentido)
            pk_actual_num=self.pk_referencia_base + (self.sentido_marcha*(distancia_recorrida/1000.0))

            #Salida: Conversión float a texto 'km+m'
            msg_pub=String()
            msg_pub.data=(
                f'{self.via_actual} | PK: {self.formatear_pk(pk_actual_num)}'
            )
            self.pub_pk.publish(msg_pub)


    def callback_gps(self, msg):
        if self.estado=='AVANCE':
            return
        
        lat_actual=msg.latitude
        lon_actual=msg.longitude

        if self.estado == 'INIT':
            self.bd_pks['dist_temp'] = self.bd_pks.apply(
                lambda row: calcular_distancia(
                    lat_actual, lon_actual, row['latitud'], row['longitud']
                ),
                axis=1,
            )

            cercanos = self.bd_pks.nsmallest(3, 'dist_temp')
            self.via_actual = cercanos.iloc[0]['nombre_via']
            self.hitos_cercanos = cercanos.to_dict('records')

            for hito in self.hitos_cercanos:
                self.distancias_iniciales[hito['PK_num']] = hito['dist_temp']

            self.get_logger().info(
                f'Vía detectada: {self.via_actual}. Esperando movimiento.'
            )
            self.estado = 'ORIENTANDO'

        elif self.estado == 'ORIENTANDO':
            if self.odometria_actual < 5.0:
                return
            self.get_logger().info('Movimiento detectado. Determinando sentido.')

            hito_siguiente=None
            hito_anterior=None

            deltas={}
            for hito in self.hitos_cercanos:
                dist_nueva=calcular_distancia(lat_actual, lon_actual, hito['latitud'], hito['longitud'])
                dist_vieja=self.distancias_iniciales[hito['PK_num']]

                deltas[hito['PK_num']]=(dist_nueva-dist_vieja, hito)
                #Delta: Variación de la distancia con el hito. Delta negativo, nos acercamos. Delta positivo, nos alejamos.
                hito_siguiente=min(deltas.values(), key=lambda x: x[0][1])
                hito_anterior=max(deltas.values(), key=lambda x: x[0][1])

                if dist_nueva < dist_vieja:
                    hito_siguiente=hito
                else:
                    hito_anterior=hito

            if hito_siguiente and hito_anterior:
                #Deduccion del sentido de marcha: PK aumentan o diminuyen.
                if hito_siguiente['PK_num']> hito_anterior['PK_num']:
                    self.sentido_marcha=1
                    self.get_logger.info('Sentido de marcha ascendente.')
                else:
                    self.sentido_marcha=-1
                    self.get_logger.info('Sentido de marcha descendente.')

            dist_hito_siguiente=calcular_distancia(lat_actual, lon_actual, hito_siguiente['latitud'], hito_siguiente['longitud'])

            self.pk_referencia_base=self.hito_siguiente['PK_num']-self.sentido_marcha*(dist_hito_siguiente/1000.0)
            self.odometria_base=self.odometria_actual

            self.get_logger().info(
                f'Calibracion completada. PK inicial fijado en {self.formatear_pk(self.pk_referencia_base)}'
            )
            self.get_logger().info(
                'Entrando en modo AVANCE (Navegacion ciega por odometría).'
            )
            self.estado='AVANCE'

    

    def formatear_pk(self, valor_decimal):
        """Convierte un valor decimal numérico al formato estandarizado KM+METROS."""
        km = int(valor_decimal)
        metros = int(round((valor_decimal - km) * 1000))
        #Esta comprobación hace falta por si en el redondeo se llega al siguiente km
        if metros >= 1000:
            km += 1
            metros -= 1000
        elif metros < 0:
            km -= 1
            metros += 1000

        return f'{km}+{metros:03d}'


    def pk_a_decimal(self, pk):
        """Convierte un valor estandarizado KM+METROS a un valor decimal numérico."""
        if pd.isna(pk):
            return None
        
        pk=str(pk).strip()

        #Validar formato
        if not re.fullmatch(r'\d+\+\d{3}', pk):
            return None

        #Convertir de decimal a PK.
        return float(pk.replace('+', '.'))
        return f'{km}+{metros:03d}'


def main(args=None):
    rclpy.init(args=args)
    nodo=Coordenadas_a_PPKK()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
