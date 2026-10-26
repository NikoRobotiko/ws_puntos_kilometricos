# Contexto de trabajo: localización ferroviaria por PK

Este archivo resume las decisiones, pruebas y dudas tratadas con el usuario sobre `coord_ppkk_v2` y `localizacion_pk`. No sustituye a `README.md`: allí está la documentación general del proyecto; aquí se conserva el contexto de colaboración y de las pruebas sintéticas. Verificar siempre el código y los datos actuales antes de dar por vigente un detalle.

## Alcance y preferencias del usuario

- El interés principal es comprobar `coord_ppkk_v2` y `localizacion_pk` por sí mismos. Evitar ampliar el análisis a otros nodos o a «deuda de estilo» salvo que afecten directamente a la prueba.
- El vehículo de prueba no está sobre una vía ni tiene señal GPS. Se inyectan por consola mensajes sintéticos de GPS y encoder en ROS 2.
- El usuario prefiere scripts Python sencillos que pueda guardar y ejecutar él mismo en el NUC. No crear archivos de prueba en este repositorio sin que lo pida; previamente solicitó deshacer una adición no deseada.
- Separar claramente los comandos de shell (`source ...`, `python3 archivo.py`) del contenido del `.py`. El usuario obtuvo `SyntaxError` por introducir `source` y `python3 - <<'PY'` dentro de un archivo Python.
- Quiere poder variar la velocidad, probar ida y vuelta, curvas, aceleraciones y deceleraciones, y contrastar los PK observados con los esperados. Para un trayecto entre dos puntos fijos, no introducir otra distancia independiente como si cambiara el destino.

## Semántica del PK y del movimiento

- El PK es una referencia no negativa de una vía. `metros_a_pk` expresa metros como `KM+MMM`, redondeando al metro; la cifra decimal interna viene de la estimación continua de `PKTracker`.
- Circular en el sentido contrario no implica PK negativo: el PK disminuye dentro de la vía; el sentido se maneja por separado. Al llegar al PK 0+000, la vía termina o se continúa por otra vía cuyo PK vuelve a ser no negativo. No extrapolar automáticamente a PK negativos ni inferir cambios de vía sin topología/ruta.
- El signo de `Odometry.twist.twist.linear.x` representa avance/retroceso respecto al eje del vehículo cuando `encoder_signed=True` (valor inicial). La orientación aprendida por el localizador relaciona ese signo con PK creciente o decreciente. La distancia recorrida, como magnitud, no es negativa.
- `coord_ppkk_v2` usa `/gps/fix` (`sensor_msgs/msg/NavSatFix`) y `/encoder_odometry` (`nav_msgs/msg/Odometry`). Integra `twist.twist.linear.x` en m/s, con marcas `header.stamp` coherentes y regla trapezoidal entre muestras; no usa `pose.pose.position.x` como distancia acumulada. Publica `/punto_kilometrico`, `/pk_metros`, `/pk_sigma` y `/pk_estado`.
- Estados: `INIT` identifica una vía próxima no ambigua; `ORIENTANDO` determina la relación encoder–PK tras movimiento suficiente y varias confirmaciones; `AVANCE` integra encoder y corrige con GPS. Antes de `AVANCE` puede no haber mensajes en `/punto_kilometrico`. Consultar `/pk_estado` para diagnosticarlo.
- Las publicaciones de PK pueden aparecer duplicadas en un `echo`: una misma estimación puede emitirse al procesar GPS y odometría. El texto se redondea a 1 m; `/pk_metros` permite ver variaciones submétricas. Esa resolución no garantiza exactitud de 1 m.
- No dejar huecos entre muestras de encoder mayores que `max_encoder_gap` (por defecto 2 s) si se quiere integrar movimiento continuo. En una parada, publicar muestras con velocidad cero. Tras reiniciar un escenario independiente, reiniciar también `coord_ppkk_v2` para obtener una identificación y orientación nuevas.

## Conexión ROS 2 ya comprobada en el NUC

- Con el launcher del sistema en marcha, `ros2 topic info -v /gps/fix` mostró a `coord_ppkk_v2` como suscriptor `BEST_EFFORT`. El publicador sintético era `RELIABLE`: QoS compatibles. También se observó `navsat_transform_node` suscrito a GPS.
- Inicialmente coexistían el GPS real adaptado y el sintético en `/gps/fix`; el usuario deshabilitó temporalmente el publicador real. Conviene comprobar que solo haya **un** publicador GPS durante estas pruebas.
- `ros2 topic info -v /encoder_odometry` mostró un publicador sintético y a `coord_ppkk_v2` como suscriptor; también eran compatibles.
- `navsat_transform_node` emitió avisos por `frame_id` vacío o transformaciones no disponibles. Eso puede afectar a ese nodo, pero no explica por sí solo la ausencia de PK de `coord_ppkk_v2`.
- La suscripción de otro nodo a `/punto_kilometrico` mediante `std_msgs.msg.String` y `self.current_state['punto_kilometrico'] = msg.data` es adecuada para guardar el texto en un CSV, suponiendo que el QoS sea compatible y exista el import.

## Pruebas de la vía 050, Madrid–frontera francesa

- Tramo sintético entre A `(40.835166640000, -2.873278200000)` y B `(40.837223468038, -2.869294536000)`, etiquetados como PK `100+000` y `100+400` en el CSV usado durante esa prueba. La distancia geográfica aproximada de la cuerda A–B es **405,74 m**, mientras que la diferencia de PK es **400 m**. No son magnitudes intercambiables.
- En una prueba anterior se publicaron 11 muestras, una por segundo; el GPS avanzaba 10 m de PK por muestra. Los ecos `100+080`, `100+090`, `100+100` y `/pk_metros` alrededor de 100080, 100090, 100100 mostraron el avance. No era un fallo del redondeo: el generador saltaba 10 m cada segundo.
- Con velocidad de ida `17,71 m/s` se vieron PK sucesivos aproximadamente `100+105`, `100+122`, ..., `100+402`. Las pequeñas diferencias se explican por tiempos reales de publicación, interpolación GPS, escala distinta entre cuerda y PK, y filtrado. En una prueba de ida/vuelta se informó un regreso próximo a `99+998`; eso **no** implica que un PK negativo sea válido.
- `DURACION = int(LONGITUD_TRAMO / VELOCIDAD)` trunca la duración. Además, fijar el GPS al extremo pero seguir enviando velocidad constante no constituye una detención física consistente. Para llegar exactamente al destino, generar una última muestra parcial y ajustar la velocidad/intervalo integrado, o definir una secuencia de velocidades cuyos incrementos trapezoidales sumen la longitud prevista.
- Si el destino es B, la expectativa cartográfica es aproximadamente `100+400`, no `100+405`, aunque la cuerda mida ~405,74 m. Para esperar PK `100+405`, el destino debería estar más allá de B sobre el tramo siguiente de la misma vía.

## Pruebas de trayectoria y velocidad propuestas

- Ya se propuso, sin añadir archivo al repositorio, un script `test_trayectorias.py` con tres escenarios: recta con velocidad variable; polilínea de vía con velocidad variable; polilínea con ida, parada y vuelta a distinta velocidad. Publica GPS y odometría una vez por segundo con marcas temporales coherentes.
- La «curva» sintética se aproxima por segmentos rectos entre hitos del mapa, no por una geometría ferroviaria continua. Hitos propuestos `(PK_m, lat, lon)`: `(217000, 41.327394668248, -1.682246984)`, `(217700, 41.329228822840, -1.674160642)`, `(218000, 41.330179239389, -1.670894259)`, `(218700, 41.332960025066, -1.663422232)`, `(219000, 41.334394640000, -1.660319491)`. La longitud aproximada de esa polilínea es 1998,38 m y el incremento de PK es 2000 m. Confirmar que los hitos estén en el CSV activo del NUC antes de usarlos.
- Para validar aceleración/frenado e inversión, comparar distancia acumulada por integración trapezoidal del encoder con la posición GPS interpolada en la misma polilínea, el PK esperado en el CSV y `/pk_metros`; observar también `/pk_sigma` y `/pk_estado`.

## Prueba curva implementada y estado real de su ejecución

- El archivo local `pruebas_pk/test_curva.py` genera una polilínea a partir de **seis** puntos elegidos con Google Earth, de A `41°31'21.96"N 5°46'43.73"W` a B `41°31'39.12"N 5°42'49.79"W`. Interpola la posición GPS dentro de cada tramo según la distancia recorrida. La longitud se calcula como suma de distancias geográficas de los cinco segmentos: en la ejecución del NUC se mostró aproximadamente **6484,0 m**. Esto no prueba que la geometría coincida exactamente con el eje ferroviario ni con la escala PK del CSV; las coordenadas fueron seleccionadas por el usuario, y queda por contrastarlas con el mapa activo.
- `coordenadas_a_decimal(texto)` convierte un par de coordenadas DMS de Google Earth a `(latitud, longitud)` decimal. Acepta `N/S` y `E/W`; sur y oeste se vuelven negativos. El ejemplo `41°30'46.62"N 5°45'58.87"W` produce `(41.51295, -5.766352777777778)`. En Python se pueden usar comillas triples para copiar el texto sin escapes: `"""41°30'46.62"N 5°45'58.87"W"""`.
- El generador tiene `VELOCIDAD = 17.71` m/s e `INTERVALO = 5.0` s, ambos modificables. Publica GPS y encoder y muestra el avance una vez por intervalo; permite valores decimales de segundos. Con esos valores se anunció `Muestra 0/61: 0.0/6484.0 m`, de modo que la prueba completa requeriría alrededor de cinco minutos. El último intervalo se acorta para alcanzar B. La **velocidad del encoder sigue siendo constante**, incluida la última muestra; la fracción final y los tiempos reales de `sleep` deben verificarse antes de interpretar errores métricos finos.
- Una ejecución en el NUC se interrumpió con `Ctrl+C` después de la muestra 0. El `KeyboardInterrupt` durante `sleep` es esperable; el `RCLError: failed to shutdown: rcl_shutdown already called` posterior provino de intentar `rclpy.shutdown()` cuando ROS ya había iniciado el cierre. Se aconsejó `except KeyboardInterrupt` y, en `finally`, `if rclpy.ok(): rclpy.shutdown()`. No consta que esa corrección se haya aplicado en el archivo local ni en el NUC.
- En las ejecuciones apareció `RTPS_TRANSPORT_SHM Error: Failed init_port ... open_and_lock_file failed`. Es un aviso del transporte de memoria compartida Fast DDS, **no una demostración por sí sola** de que GPS/encoder no llegaran. El usuario indicó que no observaba mensajes en `ros2 topic echo`, pero no aportó todavía los resultados de `topic info -v`, `echo /gps/fix`, `echo /encoder_odometry` o `/pk_estado` de esa ejecución. La prueba curva **no tiene resultado de PK validado** en esta conversación.
- Si se publican muestras cada 5 s y `max_encoder_gap` sigue en 2 s, `MotionBuffer` abre una nueva época para cada par de muestras y no suma el desplazamiento; por tanto, la orientación puede no completarse aunque los mensajes lleguen. El usuario mostró la intención de cambiar el parámetro del nodo a `20.0` s en su NUC; se sugirió `7.0` s como margen más ajustado. No consta una ejecución completa posterior que confirme qué valor quedó realmente activo. El `max_encoder_gap` del **código local inspeccionado** seguía siendo `2.0` al redactar este resumen.
- Para verificar comunicación y lógica por separado: comprobar publicadores/suscriptores con `ros2 topic info -v /gps/fix` y `/encoder_odometry`; esperar más de dos intervalos al hacer `ros2 topic echo` de ambas entradas; luego consultar `/pk_estado` para saber si el nodo está en `INIT`, `ORIENTANDO` o `AVANCE`. No borrar recursos de SHM ni cambiar middleware a ciegas.
- En el NUC el usuario ejecuta desde `~/ws_puntos_kilometricos`. Para reconstruir tras cambios de código o parámetros predeterminados: `cd ~/ws_puntos_kilometricos` y `colcon build --symlink-install --packages-select puntos_kilometricos && source install/setup.bash`; después reiniciar el nodo desde un entorno cargado con la instalación actualizada. Con symlink, algunas ediciones Python se reflejan sin reconstrucción, pero requieren reiniciar el proceso.

## Información útil para diseñar la siguiente versión de `coord_ppkk`

- Conservar como invariantes separados la identidad de la vía, el PK **no negativo**, el sentido de circulación y la distancia recorrida no negativa. Una inversión de marcha no es un cambio de vía; cruzar PK 0+000 no autoriza PK negativo. El nodo v2 fija la vía tras identificarla; manejar conexiones y cambios de vía requiere información de topología/ruta que las pruebas actuales no validaron.
- Diferenciar la distancia física a lo largo de una polilínea, la distancia de la cuerda entre hitos y la diferencia entre PK. La discrepancia de ~5,74 m en el tramo 100+000–100+400 demuestra que imponer igualdad puede sesgar las expectativas; no atribuir automáticamente al Kalman cualquier diferencia de llegada.
- Mantener trazabilidad de datos y tiempo: vía y CSV efectivos, coordenadas DMS convertidas, velocidad firmada, `header.stamp`, frecuencia de publicación, pérdidas/huecos de encoder, estados y correcciones GPS. Un experimento reproducible debe registrar **entrada, PK esperado y salida**, no solo el texto de `/punto_kilometrico`.
- Diseñar las pruebas curvas y de aceleración/deceleración con muestras temporalmente coherentes y un perfil de velocidad cuya integral coincida con el desplazamiento GPS; publicar velocidad cero durante las paradas y una inversión de signo sin huecos largos. La propuesta existe, pero **no hay aún resultados medidos** en esta conversación para curva completa, aceleración o frenado.
- Antes de sacar conclusiones sobre una versión nueva, aislar fallos de DDS/QoS o de generación de datos de los errores de localización. `/pk_estado` y `/pk_metros` son más diagnósticos que el PK textual redondeado.

## Script previo del usuario: zona Madrid–Zamora/Ourense

- El usuario compartió un script de ida y vuelta entre A `(41.4826361, -5.4259361111111115)` y B `(41.4787528, -5.418016666666667)`, con `LONGITUD_TRAMO = 789.74`, velocidad de ida `21.61 m/s` y de vuelta `18.2 m/s`. La longitud declarada y la pertenencia exacta de ambos puntos a una vía/PK del CSV activo **no se han verificado**.
- Había una errata crucial en la longitud de A: escribió `-2.-5.4259361111111115`. Es sintaxis Python válida y da `-7.4259361111111115`, situando A lejos de la ubicación pretendida. Debe ser `-5.4259361111111115`. Esa errata explica de forma plausible que `/punto_kilometrico` no mostrara nada: no se encontraría un tramo próximo para iniciar la orientación.
- Tras corregirla, si sigue sin publicarse PK, inspeccionar `ros2 topic echo /pk_estado` para distinguir `INIT` de `ORIENTANDO`; comprobar el CSV realmente cargado (`ruta_csv`), proximidad y ambigüedad de ambos puntos, GPS aceptado, tiempos/sentido del encoder y existencia de un solo publicador GPS.
- Su función `recorrer` usa `ceil(longitud / velocidad)`, limita la última posición GPS a B y acorta el último `sleep`, pero sigue enviando la velocidad nominal incluso en la última muestra. En el extremo espera 3 s **sin publicar muestras de encoder** y luego invierte el signo. Esto puede crear un hueco `> max_encoder_gap` y hace inconsistente la integración de la última fracción y del cambio de sentido; corregirlo si se analiza cuantitativamente la ida/vuelta. No atribuir toda discrepancia al filtro sin revisar el generador.

## Diagnóstico práctico

Comandos de shell (no incluirlos dentro de un `.py`):

```bash
ros2 topic info -v /gps/fix
ros2 topic info -v /encoder_odometry
ros2 topic echo /pk_estado
ros2 topic echo /pk_metros
ros2 topic echo /punto_kilometrico
```

En el NUC, cargar primero el entorno de la distribución ROS y el `install/setup.bash` del workspace correcto, y ejecutar el archivo Python después. La ruta del workspace en esa máquina puede diferir de la ruta de este repositorio; no asumir que `~/ferro/ws_puntos_kilometricos` existe. Para diferencias entre sesiones, pedir las salidas de `/pk_estado`, el CSV activo, los valores exactos GPS/encoder y la consola del launcher.
