"""Generar una copia filtrada de los hitos para inspección cartográfica."""

import argparse
import csv
from pathlib import Path

from puntos_kilometricos.localizacion_pk_v3 import (
    DEFAULT_CSV_V3, filtrar_puntos_invalidos,
)


def main(args=None):
    """Leer el CSV original y guardar únicamente los puntos válidos."""
    config = Path(__file__).resolve().parent.parent / 'config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--entrada', default=config / DEFAULT_CSV_V3,
                        type=Path)
    parser.add_argument('--salida', default=config / 'Coordenadas_PPKK_AV_validos.csv',
                        type=Path)
    options = parser.parse_args(args)

    with options.entrada.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream, delimiter=';')
        fieldnames = reader.fieldnames
        if not fieldnames:
            parser.error('El CSV de entrada no contiene cabecera')
        rows, discarded = filtrar_puntos_invalidos(reader)

    options.salida.parent.mkdir(parents=True, exist_ok=True)
    with options.salida.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, delimiter=';',
                                lineterminator='\r\n')
        writer.writeheader()
        writer.writerows(rows)
    print(f'CSV generado: {options.salida} ({len(rows)} puntos válidos; '
          f'{discarded} descartados)')


if __name__ == '__main__':
    main()
