# TP 03 - Planificador de rutas con Greedy (Programación III, UADE)

## Qué hay en la carpeta

| Archivo | Para qué |
|---|---|
| `tp_greedy_vilte.py` | La implementación completa (lectura del mapa, greedy, finalización, traducción a instrucciones). **Renombrarlo con tu apellido.** |
| `ejecutar_robot.py` | Manda las instrucciones al simulador (o las prueba en modo consola sin robot). |
| `mapas/mapa1.json`, `mapa2.json`, `mapa3.json` | Los tres mapas de la cátedra, tal cual el PDF. |
| `resultados/log_mapaX.txt` | Log de cada ejecución: candidatos por paso, resultado, ruta, dibujo, instrucciones. |
| `resultados/instrucciones_mapaX.json` | Ruta e instrucciones de cada mapa en JSON. |
| `resultados/ejecucion_simulador_mapaX.txt` | Salida de la prueba contra el simulador (pose final del robot). |

## Cómo correrlo

Python 3.10 o más nuevo, sin librerías extra.

```
python tp_greedy_vilte.py mapas/mapa1.json mapas/mapa2.json mapas/mapa3.json --salida resultados
```

Prueba de las instrucciones sin robot (integra las órdenes y chequea que termine en la celda esperada):

```
python ejecutar_robot.py resultados/instrucciones_mapa1.json
```

## Cómo probarlo en el simulador

Abrí primero el simulador y dejá la ventana abierta. Después, en otra terminal, según el entorno que tengas:

```
# Laboratorio TP03 (05LaboratoriosTPs/TP03_Programacion_III_Greedy)
python ejecutar_robot.py resultados/instrucciones_mapa1.json --backend lab --carpeta <ruta a mi_desarrollo>

# Simulador G1 de 04Simuladores/UnitreeMujocoOficial (run_g1_sim.ps1)
python ejecutar_robot.py resultados/instrucciones_mapa1.json --backend g1api --carpeta <ruta a UnitreeMujocoOficial>
```

El mapa 3 termina en BLOQUEADO, así que por defecto no mueve el robot. Para ver la ruta parcial en el simulador:
agregá `--aunque-no-llegue`.

## Resultados

| Mapa | Resultado | Movimientos greedy | Ruta mínima (BFS) |
|---|---|---|---|
| 1 | DESTINO_ALCANZADO | 8 | 8 |
| 2 | DESTINO_ALCANZADO | 20 | 12 |
| 3 | BLOQUEADO en (6,4) | 12 | 16 |

## Seguridad

Probar siempre primero en el simulador. Cualquier demo con el robot físico es opcional, a velocidad reducida (0,25 m/s) y
supervisada por el docente.
