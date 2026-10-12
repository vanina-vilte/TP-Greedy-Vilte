"""
Ejecuta en el robot Unitree (simulador) las instrucciones que genera
tp_greedy_vilte.py.

Primero corré el planificador para generar los archivos:
    python tp_greedy_vilte.py mapas/mapa1.json mapas/mapa2.json mapas/mapa3.json --salida resultados

Después probás las instrucciones, siempre primero SIN robot (modo consola):
    python ejecutar_robot.py resultados/instrucciones_mapa1.json

Y recién cuando verifiques que todo cierra, contra el simulador:
    # Laboratorio TP03 (05LaboratoriosTPs/TP03_Programacion_III_Greedy), con el simulador abierto
    python ejecutar_robot.py resultados/instrucciones_mapa1.json --backend lab --carpeta <ruta a mi_desarrollo>

    # Simulador G1 de 04Simuladores/UnitreeMujocoOficial, con run_g1_sim.ps1 abierto
    python ejecutar_robot.py resultados/instrucciones_mapa1.json --backend g1api --carpeta <ruta a UnitreeMujocoOficial>

Seguridad: velocidades bajas (0.25 m/s y 1 rad/s) y ninguna orden dura más de 10 s.
Cualquier prueba con robot físico la hace el docente, no este script.
"""

import argparse
import json
import math
import os
import sys
import time

VELOCIDAD = 0.25          # m/s, la recomendada por la consigna
VELOCIDAD_GIRO = 1.0      # rad/s
TIEMPO_MAXIMO_ORDEN = 10  # s por orden


# ---------------------------------------------------------------------------
# Instrucciones -> órdenes de velocidad y tiempo
# ---------------------------------------------------------------------------

def instrucciones_a_ordenes(instrucciones, tamano_celda):
    """
    Cada AVANZAR se parte en tramos de una celda (así ninguna orden se pasa del
    tiempo máximo) y cada GIRAR se pasa a rad/s. Convención: giro positivo =
    izquierda, negativo = derecha (la misma del laboratorio).
    """
    ordenes = []
    for instr in instrucciones:
        if instr["tipo"] == "AVANZAR":
            celdas = max(1, round(instr["metros"] / tamano_celda))
            for _ in range(celdas):
                ordenes.append({"tipo": "avanzar", "velocidad": VELOCIDAD,
                                "tiempo": tamano_celda / VELOCIDAD,
                                "detalle": f"avanzar {tamano_celda:.2f} m"})
        else:
            angulo = math.radians(instr["grados"])
            if instr["sentido"] == "DERECHA":
                signo, texto = -1, "girar a la derecha 90°"
            else:  # IZQUIERDA o MEDIA_VUELTA (la media vuelta la hacemos hacia la izquierda)
                signo = 1
                texto = "girar a la izquierda 90°" if instr["sentido"] == "IZQUIERDA" else "media vuelta"
            ordenes.append({"tipo": "girar", "velocidad": signo * VELOCIDAD_GIRO,
                            "tiempo": angulo / VELOCIDAD_GIRO, "detalle": texto})
    for o in ordenes:
        assert o["tiempo"] <= TIEMPO_MAXIMO_ORDEN, "una orden supera el tiempo máximo permitido"
    return ordenes


# ---------------------------------------------------------------------------
# Backends
# ---------------------------------------------------------------------------

class RobotConsola:

    def __init__(self, orientacion_inicial, tamano_celda, inicio):
        # Sistema de la grilla: x = columna (ESTE), y = -fila (NORTE hacia arriba)
        self.x = inicio[1] * tamano_celda
        self.y = -inicio[0] * tamano_celda
        self.rumbo = {"ESTE": 0.0, "NORTE": math.pi / 2,
                      "OESTE": math.pi, "SUR": -math.pi / 2}[orientacion_inicial]
        self.tiempo_total = 0.0

    def conectar(self):
        print("[consola] modo prueba, no se mueve ningún robot")

    def avanzar(self, velocidad, tiempo):
        self.x += velocidad * tiempo * math.cos(self.rumbo)
        self.y += velocidad * tiempo * math.sin(self.rumbo)
        self.tiempo_total += tiempo

    def girar(self, velocidad, tiempo):
        self.rumbo += velocidad * tiempo
        self.tiempo_total += tiempo

    def detenerse(self):
        pass

    def desconectar(self):
        pass

    def celda_actual(self, tamano_celda):
        return (round(-self.y / tamano_celda), round(self.x / tamano_celda))


class RobotLab:
    """Adaptador al Robot del laboratorio TP03 (avanzar/girar con velocidad y tiempo)."""

    def __init__(self, carpeta):
        sys.path.insert(0, os.path.abspath(carpeta))
        from robot import Robot  
        self._robot = Robot()

    def conectar(self):
        self._robot.conectar()

    def avanzar(self, velocidad, tiempo):
        self._robot.avanzar(velocidad=velocidad, tiempo=tiempo)

    def girar(self, velocidad, tiempo):
        self._robot.girar(velocidad=velocidad, tiempo=tiempo)

    def detenerse(self):
        self._robot.detenerse()

    def desconectar(self):
        self._robot.desconectar()

    def estado(self):
        e = self._robot.verificar_estado()
        return e.x, e.y, math.degrees(e.yaw)


class RobotG1Api:
    """Adaptador a g1_student_api.RobotG1 (UnitreeMujocoOficial), que usa movimiento(...)."""

    def __init__(self, carpeta):
        sys.path.insert(0, os.path.abspath(carpeta))
        from g1_student_api import RobotG1  # noqa: E402
        self._robot = RobotG1()

    def conectar(self):
        self._robot.conectar()

    def avanzar(self, velocidad, tiempo):
        self._robot.movimiento(adelante=velocidad, costado=0.0, giro=0.0, tiempo=tiempo)

    def girar(self, velocidad, tiempo):
        self._robot.movimiento(adelante=0.0, costado=0.0, giro=velocidad, tiempo=tiempo)

    def detenerse(self):
        self._robot.detenerse()

    def desconectar(self):
        self._robot.desconectar()

    def estado(self):
        e = self._robot.verificar_estado()
        return e.x, e.y, math.degrees(e.yaw)


# ---------------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Ejecuta las instrucciones del TP greedy en el robot (simulador).")
    parser.add_argument("instrucciones", help="archivo instrucciones_mapaX.json generado por el planificador")
    parser.add_argument("--backend", choices=["consola", "lab", "g1api"], default="consola")
    parser.add_argument("--carpeta", help="carpeta con robot.py (lab) o g1_student_api.py (g1api)")
    parser.add_argument("--aunque-no-llegue", action="store_true",
                        help="ejecutar la ruta parcial aunque el estado no sea DESTINO_ALCANZADO (solo simulador)")
    args = parser.parse_args()

    with open(args.instrucciones, encoding="utf-8") as f:
        datos = json.load(f)

    print(f"Mapa: {datos['mapa']} | estado del planificador: {datos['estado']}")
    if datos["estado"] != "DESTINO_ALCANZADO" and not args.aunque_no_llegue:
        print("El planificador no llegó al destino, así que no se mueve el robot "
              "(informar el bloqueo es el resultado correcto). Usá --aunque-no-llegue si querés "
              "ver el recorrido parcial en el simulador.")
        return 0

    tamano = datos["tamano_celda_metros"]
    ordenes = instrucciones_a_ordenes(datos["instrucciones"], tamano)
    duracion = sum(o["tiempo"] for o in ordenes)
    print(f"{len(ordenes)} órdenes, duración estimada {duracion:.1f} s")

    if args.backend == "consola":
        robot = RobotConsola(datos["orientacion_inicial"], tamano, datos["ruta"][0])
    else:
        if not args.carpeta:
            print("Para este backend hace falta --carpeta.", file=sys.stderr)
            return 2
        robot = RobotLab(args.carpeta) if args.backend == "lab" else RobotG1Api(args.carpeta)

    robot.conectar()
    try:
        for i, o in enumerate(ordenes, 1):
            print(f"  [{i:>2}/{len(ordenes)}] {o['tipo']:<7} v={o['velocidad']:+.2f} t={o['tiempo']:.2f} s  ({o['detalle']})")
            if o["tipo"] == "avanzar":
                robot.avanzar(o["velocidad"], o["tiempo"])
            else:
                robot.girar(o["velocidad"], o["tiempo"])
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    finally:
        robot.detenerse()
        if hasattr(robot, "estado"):
            x, y, yaw = robot.estado()
            f0, c0 = datos["ruta"][0]
            f1, c1 = datos["ruta"][-1]
            print(f"Pose final del robot: x={x:+.2f} m, y={y:+.2f} m, rumbo={yaw:+.0f}°")
            print(f"Pose esperada si llegó a la celda {[f1, c1]} (desde {[f0, c0]}): "
                  f"x={(c1 - c0) * tamano:+.2f} m, y={-(f1 - f0) * tamano:+.2f} m")
        robot.desconectar()

    if isinstance(robot, RobotConsola):
        esperada = tuple(datos["ruta"][-1])
        llegada = robot.celda_actual(tamano)
        estado = "OK" if llegada == esperada else "NO COINCIDE"
        print(f"Celda final por integración de las órdenes: {list(llegada)} | esperada: {list(esperada)} -> {estado}")
        return 0 if llegada == esperada else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
