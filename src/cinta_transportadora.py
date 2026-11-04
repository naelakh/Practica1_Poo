import mujoco
import numpy as np

from src import config


class CintaTransportadora:
    """Una cinta: si está en marcha, empuja las piezas; si no, las frena."""

    def __init__(self, numero, nombre, velocidad, direccion, direccion_giro,
                 eje_centrado, centro, actuadores, signo_rodillo):
        self.numero = numero
        self.nombre = nombre
        self.velocidad = velocidad
        self.direccion = np.array(direccion, dtype=float)           # hacia dónde avanza
        self.direccion_giro = None if direccion_giro is None else np.array(direccion_giro, dtype=float)
        self.eje_centrado = eje_centrado      # 0 = corrige X, 1 = corrige Y
        self.centro = centro                  # línea central de la banda
        self.actuadores = actuadores          # rodillos en MuJoCo
        self.signo_rodillo = signo_rodillo
        self.activa = True                    # sustituye a BANDERA_CINTA_X
        self.bloqueos = set()                 # cámaras que la tienen parada

    def arrancar(self):
        self.activa = True

    def detener(self):
        self.activa = False

    def conmutar(self):
        self.activa = not self.activa
        return self.activa

    def esta_activa(self):
        return self.activa and not self.bloqueos

    def bloquear(self, camara):
        self.bloqueos.add(camara)

    def desbloquear(self, camara):
        self.bloqueos.discard(camara)

    def fuerza_frenado(self, vel):
        return self.limitar(-config.VEL_KP * vel[0], -config.VEL_KP * vel[1])

    def calcular_fuerza(self, pos, vel):
        if not self.esta_activa():
            return self.fuerza_frenado(vel)

        alpha = 0.0                                   # 0 = tramo recto, 1 = girando
        v_deseada = self.velocidad * self.direccion
        if self.direccion_giro is not None:           # la cinta 5 no gira
            avance = pos[0] * self.direccion[0] + pos[1] * self.direccion[1]
            alpha = np.clip((avance - config.TURN_START)
                            / (config.CORNER_SWITCH - config.TURN_START), 0.0, 1.0)
            v_deseada = self.velocidad * ((1 - 0.45 * alpha) * self.direccion
                                          + 0.85 * alpha * self.direccion_giro)

        fuerza = config.VEL_KP * (v_deseada - vel[:2])               # alcanzar la velocidad
        e = self.eje_centrado
        fuerza[e] += (1 - alpha) * config.CENTER_KP * (self.centro - pos[e])   # centrar
        return self.limitar(fuerza[0], fuerza[1])

    def limitar(self, fx, fy):
        m = config.MAX_FORCE
        return float(np.clip(fx, -m, m)), float(np.clip(fy, -m, m))

    def actualizar_rodillos(self, model, data):
        giro = self.signo_rodillo * config.VELOCIDAD_RODILLO if self.esta_activa() else 0.0
        for nombre in self.actuadores:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, nombre)
            data.ctrl[aid] = giro