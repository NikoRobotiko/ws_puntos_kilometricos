"""Nodo ROS 2 de localización PK con conexiones y cambios de vía."""

from dataclasses import fields
import math
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import rclpy

from puntos_kilometricos.coord_ppkk_v2 import Coordenadas_a_PPKK as BaseNode
from puntos_kilometricos.localizacion_pk import Settings as BaseSettings
from puntos_kilometricos.localizacion_pk_v3 import (
    DEFAULT_CSV_V3, PKTrackerV3, RailwayMapV3, SettingsV3,
)


class Coordenadas_a_PPKK(BaseNode):
    """Mantener las entradas y salidas de v2 y habilitar desvíos en v3."""

    default_csv_filename = DEFAULT_CSV_V3

    def __init__(self):
        super().__init__()
        base_fields = {field.name for field in fields(BaseSettings)}

        for field in fields(SettingsV3):
            if field.name not in base_fields:
                self.declare_parameter(field.name, getattr(SettingsV3(), field.name))

        self.settings = SettingsV3(**{
            field.name: self.get_parameter(field.name).value
            for field in fields(SettingsV3)})
        
        for field in fields(SettingsV3):
            value = getattr(self.settings, field.name)
            if field.name != 'initial_track' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{field.name} debe ser positivo y finito')
            
        if self.settings.switch_confirmations < 2:
            raise ValueError('switch_confirmations debe ser al menos 2')
        
        if self.settings.switch_min_distance >= self.settings.switch_window:
            raise ValueError('switch_min_distance debe ser menor que switch_window')
        
        railway_map, discarded = RailwayMapV3.from_csv(
            self.get_parameter('ruta_csv').value)
        
        if (self.settings.initial_track
                and self.settings.initial_track not in railway_map.hitos_by_track):
            raise ValueError('initial_track no corresponde a una vía seleccionada en v3')
        
        railway_map.build_junctions(self.settings.junction_distance)
        
        try:
            config_dir = get_package_share_directory('puntos_kilometricos')
        except PackageNotFoundError:
            config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')

        self.declare_parameter(
            'ruta_enlaces', os.path.join(config_dir, 'Enlaces_vias_v3.csv'))
        route = self.get_parameter('ruta_enlaces').value
        if route:
            railway_map.load_explicit_junctions(route)
        self.tracker = PKTrackerV3(railway_map, self.motion, self.settings)
        if discarded:
            self.get_logger().warning(
                f'v3: {discarded} filas excluidas por selección de vías o validación')
        explicit = sum(link.explicit for link in railway_map.junctions)
        disabled = sum(not link.switch_enabled for link in railway_map.junctions)
        self.get_logger().info(
            f'v3: {len(railway_map.hitos_by_track)} vías seleccionadas; '
            f'{len(railway_map.junctions)} enlaces entre vías; '
            f'enlaces explícitos: {explicit}; sin cambio automático: {disabled}')

    def _process_gps(self):
        old_state = self.tracker.state
        old_track = self.tracker.track
        while self.pending_gps and self.pending_gps[0][0] <= self.motion.latest.time:
            self.tracker.observe(*self.pending_gps.popleft())
        if old_state != self.tracker.state or old_track != self.tracker.track:
            self.get_logger().info(f'Estado: {self.tracker.state}; vía: {self.tracker.track}')


def main(args=None):
    """Ejecutar el nodo v3 y liberar recursos al finalizar."""
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
