"""
TP 03 - Planificador de rutas con estrategia Greedy
Programación III - UADE

Qué hace este programa:
  1. Lee un mapa desde un JSON (grilla, inicio, destino, tamaño de celda, etc.).
  2. Arma la ruta con una estrategia greedy: en cada paso mira los vecinos
     válidos, calcula la distancia Manhattan al destino y se va al más cercano.
     Si hay empate, se usa un orden fijo: ARRIBA -> DERECHA -> ABAJO -> IZQUIERDA.
  3. Termina por DESTINO_ALCANZADO, BLOQUEADO o LIMITE_DE_PASOS.
  4. Traduce la ruta a instrucciones para el robot (AVANZAR / GIRAR).

Uso:
    python tp_greedy_vilte.py mapas/mapa1.json mapas/mapa2.json mapas/mapa3.json
    python tp_greedy_vilte.py mapas/mapa1.json --salida resultados

Nota: la función ruta_minima_referencia() (BFS) NO es parte del greedy. La dejé
solamente para comparar contra la ruta óptima en el análisis de los mapas.
"""

import argparse
import json
import os
import sys
from collections import deque

# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------

LIBRE = 0
OBSTACULO = 1
ZONA_PROHIBIDA = 2

# Orden fijo de desempate (también es el orden en que se generan los candidatos).
# Los movimientos son (delta_fila, delta_columna).
MOVIMIENTOS = [
    ("ARRIBA", (-1, 0)),
    ("DERECHA", (0, 1)),
    ("ABAJO", (1, 0)),
    ("IZQUIERDA", (0, -1)),
]

# Orientaciones del robot sobre el mapa visto desde arriba:
# ARRIBA = NORTE, DERECHA = ESTE, ABAJO = SUR, IZQUIERDA = OESTE.
# Van en sentido horario, así que girar a la derecha es sumar 1 en esta lista.
ORIENTACIONES = ["NORTE", "ESTE", "SUR", "OESTE"]
MOVIMIENTO_A_ORIENTACION = {
    "ARRIBA": "NORTE",
    "DERECHA": "ESTE",
    "ABAJO": "SUR",
    "IZQUIERDA": "OESTE",
}

DESTINO_ALCANZADO = "DESTINO_ALCANZADO"
BLOQUEADO = "BLOQUEADO"
LIMITE_DE_PASOS = "LIMITE_DE_PASOS"

CAUSAS = {
    DESTINO_ALCANZADO: "El robot llegó a la celda destino.",
    BLOQUEADO: "No existen candidatos válidos para continuar.",
    LIMITE_DE_PASOS: "Se alcanzó el máximo de pasos permitido.",
}

CLAVES_OBLIGATORIAS = [
    "grilla",
    "inicio",
    "destino",
    "tamano_celda_metros",
    "orientacion_inicial",
    "maximo_pasos",
]


# ---------------------------------------------------------------------------
# 1. Lectura y validación del mapa
# ---------------------------------------------------------------------------

class MapaInvalido(Exception):
    """Se lanza cuando el JSON no cumple con lo que espera el programa."""


def _validar_posicion(nombre, pos, filas, columnas):
    if (not isinstance(pos, list) or len(pos) != 2
            or not all(isinstance(v, int) and not isinstance(v, bool) for v in pos)):
        raise MapaInvalido(f"'{nombre}' tiene que ser una lista [fila, columna] de enteros.")
    f, c = pos
    if not (0 <= f < filas and 0 <= c < columnas):
        raise MapaInvalido(f"'{nombre}' {pos} está fuera de la grilla ({filas}x{columnas}).")


def cargar_mapa(ruta_archivo):
    """Lee el JSON, valida todo y devuelve un diccionario con los datos del mapa."""
    try:
        with open(ruta_archivo, encoding="utf-8") as archivo:
            datos = json.load(archivo)
    except FileNotFoundError:
        raise MapaInvalido(f"No encontré el archivo '{ruta_archivo}'.")
    except json.JSONDecodeError as error:
        raise MapaInvalido(f"El archivo '{ruta_archivo}' no es un JSON válido: {error}")

    faltantes = [k for k in CLAVES_OBLIGATORIAS if k not in datos]
    if faltantes:
        raise MapaInvalido(f"Faltan campos obligatorios en el JSON: {', '.join(faltantes)}")

    grilla = datos["grilla"]
    if not isinstance(grilla, list) or not grilla or not all(isinstance(fila, list) for fila in grilla):
        raise MapaInvalido("'grilla' tiene que ser una lista de filas (listas) no vacía.")

    filas = len(grilla)
    columnas = len(grilla[0])
    if columnas == 0 or any(len(fila) != columnas for fila in grilla):
        raise MapaInvalido("Todas las filas de la grilla deben tener la misma cantidad de columnas.")

    for f, fila in enumerate(grilla):
        for c, valor in enumerate(fila):
            if valor not in (LIBRE, OBSTACULO, ZONA_PROHIBIDA) or isinstance(valor, bool):
                raise MapaInvalido(f"Valor inválido {valor!r} en la celda ({f},{c}). Solo se permite 0, 1 o 2.")

    _validar_posicion("inicio", datos["inicio"], filas, columnas)
    _validar_posicion("destino", datos["destino"], filas, columnas)

    inicio = tuple(datos["inicio"])
    destino = tuple(datos["destino"])
    if grilla[inicio[0]][inicio[1]] != LIBRE:
        raise MapaInvalido("La celda de inicio tiene que ser libre (0).")
    if grilla[destino[0]][destino[1]] != LIBRE:
        raise MapaInvalido("La celda de destino tiene que ser libre (0).")

    tamano = datos["tamano_celda_metros"]
    if not isinstance(tamano, (int, float)) or isinstance(tamano, bool) or tamano <= 0:
        raise MapaInvalido("'tamano_celda_metros' tiene que ser un número positivo.")

    orientacion = str(datos["orientacion_inicial"]).upper()
    if orientacion not in ORIENTACIONES:
        raise MapaInvalido(f"'orientacion_inicial' debe ser una de {ORIENTACIONES}.")

    maximo = datos["maximo_pasos"]
    if not isinstance(maximo, int) or isinstance(maximo, bool) or maximo <= 0:
        raise MapaInvalido("'maximo_pasos' tiene que ser un entero positivo.")

    return {
        "nombre": datos.get("nombre", os.path.basename(ruta_archivo)),
        "grilla": grilla,
        "filas": filas,
        "columnas": columnas,
        "inicio": inicio,
        "destino": destino,
        "tamano_celda": float(tamano),
        "orientacion_inicial": orientacion,
        "maximo_pasos": maximo,
    }


# ---------------------------------------------------------------------------
# 2. Algoritmo greedy
# ---------------------------------------------------------------------------

def distancia_manhattan(posicion, destino):
    """Función de selección: distancia Manhattan de una celda al destino."""
    return abs(posicion[0] - destino[0]) + abs(posicion[1] - destino[1])


def es_factible(posicion, mapa, visitadas):
    """Criterio de factibilidad: dentro de la grilla, libre (0) y no visitada."""
    f, c = posicion
    if not (0 <= f < mapa["filas"] and 0 <= c < mapa["columnas"]):
        return False  # fuera de la grilla
    if mapa["grilla"][f][c] != LIBRE:
        return False  # obstáculo (1) o zona prohibida (2)
    if posicion in visitadas:
        return False  # el robot no puede pisar dos veces la misma celda
    return True


def generar_candidatos(posicion, mapa, visitadas):
    """
    Devuelve la lista de movimientos posibles desde 'posicion' como tuplas
    (nombre_movimiento, (fila, columna), distancia). Salen en el orden fijo
    ARRIBA -> DERECHA -> ABAJO -> IZQUIERDA.
    """
    candidatos = []
    for nombre, (df, dc) in MOVIMIENTOS:
        vecino = (posicion[0] + df, posicion[1] + dc)
        if es_factible(vecino, mapa, visitadas):
            candidatos.append((nombre, vecino, distancia_manhattan(vecino, mapa["destino"])))
    return candidatos


def seleccionar_candidato(candidatos):
    """
    Elige el candidato de menor distancia. Como la lista ya viene ordenada según
    el criterio de desempate y min() devuelve el primero que encuentra, en caso
    de empate gana el que aparece antes en ARRIBA -> DERECHA -> ABAJO -> IZQUIERDA.
    """
    return min(candidatos, key=lambda cand: cand[2])


def ejecutar_greedy(mapa, log=print):
    """
    Corre el greedy sobre el mapa. Nunca retrocede ni revisa decisiones.
    Devuelve un diccionario con la ruta, los movimientos, la cantidad de pasos
    y la causa de finalización.
    """
    posicion = mapa["inicio"]
    destino = mapa["destino"]
    visitadas = {posicion}
    ruta = [posicion]
    movimientos = []
    maximo = mapa["maximo_pasos"]

    log(f"Inicio: {list(posicion)} | Destino: {list(destino)} | "
        f"Distancia inicial: {distancia_manhattan(posicion, destino)} | Máximo de pasos: {maximo}")
    log("")

    estado = None
    while True:
        if posicion == destino:
            estado = DESTINO_ALCANZADO
            break
        if len(movimientos) >= maximo:
            estado = LIMITE_DE_PASOS
            break

        candidatos = generar_candidatos(posicion, mapa, visitadas)
        paso = len(movimientos) + 1
        if not candidatos:
            log(f"Paso {paso:>2} | en {list(posicion)} | sin candidatos válidos")
            estado = BLOQUEADO
            break

        elegido = seleccionar_candidato(candidatos)
        detalle = ", ".join(f"{n}->{list(p)} d={d}" for n, p, d in candidatos)
        log(f"Paso {paso:>2} | en {list(posicion)} | candidatos: [{detalle}] | "
            f"elijo {elegido[0]} -> {list(elegido[1])} (d={elegido[2]})")

        movimientos.append(elegido[0])
        posicion = elegido[1]
        visitadas.add(posicion)
        ruta.append(posicion)

    return {
        "ruta": ruta,
        "movimientos": movimientos,
        "pasos": len(movimientos),
        "estado": estado,
        "causa": CAUSAS[estado],
        "posicion_final": posicion,
        "distancia_final": distancia_manhattan(posicion, destino),
    }


# ---------------------------------------------------------------------------
# 3. Comparación con ruta mínima (solo para el análisis, no es greedy)
# ---------------------------------------------------------------------------

def ruta_minima_referencia(mapa):
    """BFS sobre celdas libres. Devuelve la ruta más corta o None si no existe."""
    inicio, destino = mapa["inicio"], mapa["destino"]
    anterior = {inicio: None}
    cola = deque([inicio])
    while cola:
        actual = cola.popleft()
        if actual == destino:
            ruta = []
            while actual is not None:
                ruta.append(actual)
                actual = anterior[actual]
            return ruta[::-1]
        for _, (df, dc) in MOVIMIENTOS:
            vecino = (actual[0] + df, actual[1] + dc)
            if vecino in anterior:
                continue
            f, c = vecino
            if 0 <= f < mapa["filas"] and 0 <= c < mapa["columnas"] and mapa["grilla"][f][c] == LIBRE:
                anterior[vecino] = actual
                cola.append(vecino)
    return None


# ---------------------------------------------------------------------------
# 4. Traducción de la ruta a instrucciones del robot
# ---------------------------------------------------------------------------

def formato_metros(valor):
    """0.5 -> '0,50' (coma decimal, como en la consigna)."""
    return f"{valor:.2f}".replace(".", ",")


def ruta_a_instrucciones(ruta, orientacion_inicial, tamano_celda, agrupar=False):
    """
    Pasa la ruta de celdas a instrucciones físicas. Devuelve una lista de dicts:
        {"tipo": "GIRAR", "sentido": "DERECHA"|"IZQUIERDA"|"MEDIA_VUELTA", "grados": 90|180}
        {"tipo": "AVANZAR", "metros": float}
    Si agrupar=True, los AVANZAR consecutivos se suman en uno solo.
    """
    instrucciones = []
    orientacion = orientacion_inicial
    traduccion_mov = {(df, dc): MOVIMIENTO_A_ORIENTACION[nombre] for nombre, (df, dc) in MOVIMIENTOS}

    for anterior, actual in zip(ruta, ruta[1:]):
        delta = (actual[0] - anterior[0], actual[1] - anterior[1])
        nueva = traduccion_mov[delta]
        giro = (ORIENTACIONES.index(nueva) - ORIENTACIONES.index(orientacion)) % 4
        if giro == 1:
            instrucciones.append({"tipo": "GIRAR", "sentido": "DERECHA", "grados": 90})
        elif giro == 2:
            instrucciones.append({"tipo": "GIRAR", "sentido": "MEDIA_VUELTA", "grados": 180})
        elif giro == 3:
            instrucciones.append({"tipo": "GIRAR", "sentido": "IZQUIERDA", "grados": 90})
        orientacion = nueva

        if agrupar and instrucciones and instrucciones[-1]["tipo"] == "AVANZAR":
            instrucciones[-1]["metros"] += tamano_celda
        else:
            instrucciones.append({"tipo": "AVANZAR", "metros": tamano_celda})
    return instrucciones


def instruccion_a_texto(instr):
    if instr["tipo"] == "AVANZAR":
        return f"AVANZAR {formato_metros(instr['metros'])} m"
    if instr["sentido"] == "MEDIA_VUELTA":
        return "GIRAR 180°"
    return f"GIRAR {instr['sentido']} {instr['grados']}°"


# ---------------------------------------------------------------------------
# 5. Representaciones textuales de la ruta
# ---------------------------------------------------------------------------

def dibujar_ruta(mapa, ruta):
    """
    Dibuja la grilla con la ruta: I = inicio, D = destino, # = obstáculo,
    X = zona prohibida, . = libre sin usar, flechas = dirección de salida de
    cada celda recorrida, * = última celda alcanzada si no es el destino.
    """
    flechas = {(-1, 0): "↑", (0, 1): "→", (1, 0): "↓", (0, -1): "←"}
    simbolos = {}
    for actual, siguiente in zip(ruta, ruta[1:]):
        simbolos[actual] = flechas[(siguiente[0] - actual[0], siguiente[1] - actual[1])]

    lineas = ["    " + " ".join(str(c) for c in range(mapa["columnas"]))]
    for f in range(mapa["filas"]):
        fila = []
        for c in range(mapa["columnas"]):
            celda = (f, c)
            valor = mapa["grilla"][f][c]
            if celda == mapa["inicio"]:
                fila.append("I")
            elif celda == mapa["destino"]:
                fila.append("D")
            elif valor == OBSTACULO:
                fila.append("#")
            elif valor == ZONA_PROHIBIDA:
                fila.append("X")
            elif celda in simbolos:
                fila.append(simbolos[celda])
            elif celda == ruta[-1]:
                fila.append("*")
            else:
                fila.append(".")
        lineas.append(f"{f:>2}  " + " ".join(fila))
    return "\n".join(lineas)


def ruta_como_texto(ruta):
    return " -> ".join(f"({f},{c})" for f, c in ruta)


# ---------------------------------------------------------------------------
# 6. Armado del reporte de una ejecución
# ---------------------------------------------------------------------------

def procesar_mapa(ruta_json, carpeta_salida=None):
    """Carga un mapa, corre greedy, arma el reporte y (si corresponde) guarda los archivos."""
    mapa = cargar_mapa(ruta_json)
    base = os.path.splitext(os.path.basename(ruta_json))[0]

    lineas = []

    def log(texto=""):
        lineas.append(texto)
        print(texto)

    log("=" * 78)
    log(f"{mapa['nombre']}  ({os.path.basename(ruta_json)})")
    log("=" * 78)
    log(f"Grilla {mapa['filas']}x{mapa['columnas']} | celda {formato_metros(mapa['tamano_celda'])} m | "
        f"orientación inicial {mapa['orientacion_inicial']}")
    log("Desempate: ARRIBA -> DERECHA -> ABAJO -> IZQUIERDA")
    log("")

    resultado = ejecutar_greedy(mapa, log)

    log("")
    log(f"RESULTADO: {resultado['estado']}")
    log(f"Causa: {resultado['causa']}")
    log(f"Movimientos realizados: {resultado['pasos']}")
    log(f"Posición final: {list(resultado['posicion_final'])} | "
        f"distancia Manhattan al destino: {resultado['distancia_final']}")
    log("")
    log("Ruta recorrida:")
    log("  " + ruta_como_texto(resultado["ruta"]))
    log("")
    log("Representación de la ruta (I inicio, D destino, # obstáculo, X prohibida, * donde quedó):")
    log(dibujar_ruta(mapa, resultado["ruta"]))
    log("")

    # Comparación contra el camino mínimo (solo informativo)
    minima = ruta_minima_referencia(mapa)
    if minima is None:
        log("Referencia (BFS): no existe ninguna ruta válida en este mapa.")
    else:
        log(f"Referencia (BFS): existe una ruta de {len(minima) - 1} movimientos: {ruta_como_texto(minima)}")
        if resultado["estado"] == DESTINO_ALCANZADO:
            extra = resultado["pasos"] - (len(minima) - 1)
            log(f"Greedy usó {resultado['pasos']} movimientos ({extra} más que el mínimo)."
                if extra else "Greedy coincidió con la longitud mínima.")
        else:
            log("Greedy no llegó al destino aunque el mapa tiene solución.")
    log("")

    instrucciones = ruta_a_instrucciones(resultado["ruta"], mapa["orientacion_inicial"], mapa["tamano_celda"])
    instrucciones_agrupadas = ruta_a_instrucciones(
        resultado["ruta"], mapa["orientacion_inicial"], mapa["tamano_celda"], agrupar=True)

    log(f"Instrucciones para el robot (paso a paso, {len(instrucciones)} instrucciones):")
    log("  " + " -> ".join(instruccion_a_texto(i) for i in instrucciones) if instrucciones else "  (sin movimientos)")
    log("")
    log(f"Instrucciones agrupadas (avances consecutivos sumados, {len(instrucciones_agrupadas)} instrucciones):")
    log("  " + " -> ".join(instruccion_a_texto(i) for i in instrucciones_agrupadas)
        if instrucciones_agrupadas else "  (sin movimientos)")
    log("")

    if carpeta_salida:
        os.makedirs(carpeta_salida, exist_ok=True)
        with open(os.path.join(carpeta_salida, f"log_{base}.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lineas) + "\n")
        with open(os.path.join(carpeta_salida, f"instrucciones_{base}.json"), "w", encoding="utf-8") as f:
            json.dump({
                "mapa": mapa["nombre"],
                "estado": resultado["estado"],
                "orientacion_inicial": mapa["orientacion_inicial"],
                "tamano_celda_metros": mapa["tamano_celda"],
                "ruta": [list(p) for p in resultado["ruta"]],
                "instrucciones": instrucciones_agrupadas,
            }, f, ensure_ascii=False, indent=2)

    return resultado


def main():
    parser = argparse.ArgumentParser(description="Planificador de rutas greedy para el robot Unitree (TP 03).")
    parser.add_argument("mapas", nargs="+", help="uno o más archivos JSON de mapa")
    parser.add_argument("--salida", default=None, help="carpeta donde guardar logs e instrucciones")
    args = parser.parse_args()

    codigo = 0
    for ruta_json in args.mapas:
        try:
            procesar_mapa(ruta_json, args.salida)
        except MapaInvalido as error:
            print(f"ERROR en {ruta_json}: {error}", file=sys.stderr)
            codigo = 1
    return codigo


if __name__ == "__main__":
    sys.exit(main())
