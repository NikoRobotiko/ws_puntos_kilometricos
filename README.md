# Localización ferroviaria por PK (v2)

`coord_ppkk_v2` estima el PK en metros mediante un Kalman escalar y publica la vía
identificada. La versión original `coord_ppkk.py` se conserva sin cambios.

La salida `1+050` significa 1050 m. El filtro trabaja con valores continuos en
metros; el texto se redondea a un metro y siempre incluye tres cifras tras `+`.
**Resolución de 1 m no implica exactitud de 1 m.** Los puntos separados cientos de
metros no describen la curva real de la vía. Interpolar sus coordenadas mediante
cuerdas puede introducir sesgos que el filtro no elimina. Para acreditar exactitud
métrica hacen falta geometría ferroviaria más densa, GPS adecuado y validación
contra referencias medidas.

## Entradas y arranque

- `/gps/fix`: `sensor_msgs/msg/NavSatFix`, coordenadas WGS84 y covarianza ENU en m².
- `/encoder_odometry`: `nav_msgs/msg/Odometry`. Se integra
  **`twist.twist.linear.x` en m/s en el eje longitudinal del vehículo**. No se usa
  `pose.pose.position.x` como distancia acumulada.
- Ambos mensajes deben llevar `header.stamp` de adquisición, en el mismo reloj
  ROS. Si se reproduce un bag con `/clock`, activar `use_sim_time`.
- Se usa QoS de sensores, compatible con publicadores best effort y reliable.

Las definiciones de mensajes están en el repositorio oficial de ROS 2:
[Odometry](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)
y [NavSatFix](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatFix.msg).

1. `INIT`: busca un tramo próximo no ambiguo. Si dos vías son indistinguibles,
   espera una posición que permita separarlas. `initial_track` permite indicar
   explícitamente el nombre de la vía tal como aparece en el CSV.
2. `ORIENTANDO`: espera movimiento suficiente en encoder y GPS. La variación
   longitudinal del PK proyectado respecto a los hitos iniciales debe superar la
   incertidumbre y coincidir en varias observaciones separadas en el tiempo.
   Puede arrancar sin mensajes de encoder; comienza la comparación cuando hay
   medidas temporales comunes. No publica un PK fusionado hasta confirmar sentido.
3. `AVANCE`: integra velocidad y corrige con GPS válido de la vía seleccionada.
   El sentido aprendido relaciona el eje del encoder con PK creciente o
   decreciente. Con velocidad firmada, una inversión de marcha se integra con su
   signo sin volver a orientar.

`encoder_signed:=true` es el valor inicial para el encoder disponible, que
entrega velocidad longitudinal con signo en m/s. La orientación inicial aprende
la relación entre ese signo y el aumento o disminución del PK. Después, una
inversión de marcha se integra automáticamente, incluso sin señal GPS.
Para un encoder que solo entregue magnitud puede configurarse
`encoder_signed:=false`; en ese caso se presupone que el vehículo mantiene el
sentido aprendido y es necesario volver a orientar si cambia de sentido.

La vía queda fijada tras identificarla. No se infieren cambios de línea ni rutas
por desvíos. Para otra vía hay que reiniciar y orientar de nuevo; resolver desvíos
requiere topología ferroviaria o información de ruta adicional.

## Filtro y sincronización

El estado es `x = PK [m]` y la varianza es `P [m²]`. La velocidad no es un segundo
estado: es una entrada de control medida por el encoder.

```text
Δd = (v_anterior + v_actual) / 2 · Δt
x_pred = x + sentido · Δd
P_pred = P + Q

z_GPS = PK interpolado entre los hitos del tramo
R = (sigma_GPS · ΔPK / longitud_cuerda)² + map_sigma²
K = P_pred / (P_pred + R)
x = x_pred + K · (z_GPS - x_pred)
P = (1 - K)² · P_pred + K² · R
```

`Q` combina `process_variance_per_second · Δt`, la varianza de velocidad por
`Δt²` y `encoder_drift_variance_per_meter · distancia_recorrida`. La covarianza
longitudinal del encoder tiene como mínimo `encoder_sigma²`. Es un modelo de
ruido efectivo que debe calibrarse con registros: el Kalman escalar no estima un
sesgo de escala del encoder como estado independiente.

La covarianza GPS se convierte en una sigma horizontal conservadora mediante su
mayor autovalor. Sin covarianza se usa `default_gps_sigma`. Los fixes sin señal,
coordenadas/covarianzas inválidas, sigma excesiva o innovaciones fuera del umbral
estadístico se descartan.

Se guarda un historial de integrales del encoder. Cada GPS se corrige en su
instante de adquisición y luego se transporta al último instante del encoder.
Esto evita tratar un GPS retrasado como si fuese una posición actual. Los GPS
que aún esperan encoder se encolan, hasta 100; los anteriores al historial o a
la última observación procesada se descartan. Las marcas repetidas de encoder no
se integran.

Sin GPS se sigue publicando con encoder y aumenta la varianza. Al recuperarlo,
se vuelve a corregir si la observación pasa los controles. Si el encoder deja de
publicar, se conserva la última estimación, se indica obsolescencia y no se
extrapola indefinidamente su última velocidad. Las correcciones GPS esperan
encoder en su instante de adquisición. Si hay un hueco de encoder superior a
`max_encoder_gap`, no se inventa la distancia recorrida y se incrementa mucho la
incertidumbre al recibir la siguiente muestra; el GPS puede reanclar la posición.

## Publicaciones

| Topic | Tipo | Contenido |
| --- | --- | --- |
| `/punto_kilometrico` | `std_msgs/String` | `nombre_via \| PK: 1+050` |
| `/pk_metros` | `std_msgs/Float64` | Estimación continua del PK en metros |
| `/pk_sigma` | `std_msgs/Float64` | Desviación estándar estimada, en metros |
| `/pk_estado` | `std_msgs/String` | JSON con fase, vía, marcas de estimación/corrección, PK, sigma y encoder obsoleto |

Las estimaciones se publican al recibir encoder o GPS; el estado se publica
además cada segundo. Cuando el encoder está obsoleto, las salidas conservadas no
representan una nueva medición. Consultar `stamp_estimacion` en `/pk_estado`.
La sigma depende de las hipótesis del modelo: no garantiza una cota para errores
sistemáticos de geometría, multipath, escala del encoder o elección de vía.

## CSV

Se esperan las columnas `nombre_via;PK;latitud;longitud`, admitiendo coma decimal
y BOM UTF-8. Se descartan formatos inválidos, coordenadas fuera de rango y PK
repetidos con coordenadas separadas más de 1 m; los duplicados coincidentes se
consolidan. Los hitos restantes se ordenan por PK para cada vía.

El ratio entre distancia geográfica y diferencia de PK no excluye tramos. Se
conserva como factor de conversión de la incertidumbre GPS a metros de PK.
Solo se omiten pares con ΔPK no positivo, distancia geográfica inferior a 1 m
o ΔPK superior a `max_segment_pk_gap`.
No se extrapolan observaciones GPS fuera de los extremos de
un tramo. Los descartes y el número de tramos utilizables se registran al
arrancar. En huecos del mapa se continúa con encoder una vez orientado, pero no
hay corrección GPS hasta volver a un tramo utilizable.

## Ejecución

Desde la raíz del workspace ROS 2:

```bash
colcon build --packages-select puntos_kilometricos
source install/setup.bash
ros2 run puntos_kilometricos coord_ppkk_v2
```

Para usar otra base de datos o especificar la vía inicial:

```bash
ros2 run puntos_kilometricos coord_ppkk_v2 --ros-args \
  -p ruta_csv:=/ruta/Coordenadas_PPKK_AV.csv \
  -p initial_track:='nombre exacto de la vía'
```

Parámetros principales, con valores iniciales orientativos:

| Parámetro | Valor | Unidad / propósito |
| --- | --- | --- |
| `encoder_signed` | `true` | El encoder disponible entrega velocidad con signo |
| `encoder_sigma` | `0.1` | m/s, ruido mínimo de velocidad |
| `encoder_drift_variance_per_meter` | `0.02` | m²/m, crecimiento espacial de varianza |
| `process_variance_per_second` | `0.1` | m²/s, crecimiento temporal de varianza |
| `map_sigma` | `10.0` | m, ruido efectivo de interpolación del mapa |
| `default_gps_sigma` | `5.0` | m, GPS sin covarianza |
| `max_gps_sigma` | `10.0` | m, límite de aceptación del GPS |
| `max_track_distance` | `30.0` | m, distancia máxima a la cuerda del mapa |
| `ambiguity_margin` | `10.0` | m, margen entre candidatos independientes |
| `min_orientation_distance` | `20.0` | m de encoder antes de orientar |
| `min_gps_displacement` | `8.0` | m; también se exige superar 3 sigma del desplazamiento |
| `required_confirmations` | `3` | Observaciones concordantes |
| `confirmation_interval` | `1.0` | s entre confirmaciones |
| `innovation_gate` | `4.0` | Sigma de innovación permitidas |
| `history_seconds` | `30.0` | s de historial para GPS retrasado |
| `max_encoder_gap` | `2.0` | s máximos integrables entre muestras |
| `max_speed` | `100.0` | m/s, límite físico del vehículo |
| `max_segment_pk_gap` | `1500.0` | m, separación máxima interpolable |

Ajustar primero el signo/unidades/timestamps del encoder, validar el mapa y
medir sus errores. Después calibrar `Q`, `R` y los umbrales contra datos con PK
conocido, incluyendo túneles, paradas, marcha atrás y recuperación del GPS.

## Pruebas funcionales

```bash
python3 -m pytest -q test/test_localizacion_pk.py test/test_coord_ppkk_v2.py
```

Cubren conversión de PK, validación del CSV, orientación, ambigüedad entre vías,
interpolación temporal, integración de velocidad, huecos de encoder, marcha atrás,
GPS retrasado, rechazo de outliers y una simulación con GPS ruidoso y deriva del
encoder. Los adaptadores se prueban con mensajes ROS reales y publicadores
controlados, sin depender del descubrimiento DDS. No sustituyen un ensayo sobre
el vehículo ni acreditan exactitud de un metro.
