import mujoco

from src import config


class ObjetoIndustrial:
    """Una pieza que circula por la planta."""

    def __init__(self, indice, forma, color, posicion_inicial, segmento_inicial):
        self.indice = indice
        self.nombre = f"obj{indice}"
        self.forma = forma
        self.color = color
        self.posicion_inicial = posicion_inicial
        self.segmento_inicial = segmento_inicial
        self.segmento = segmento_inicial      # cinta en la que está (0-4)
        self.altura_reposo = config.OBJECT_REST_Z[indice]
        self.body_id = None
        self.qadr = None
        self.dadr = None

    def conectar(self, model):
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.nombre)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.nombre}_free")
        if self.body_id < 0 or jid < 0:
            raise RuntimeError(f"No se encontró la pieza '{self.nombre}' en el modelo.")
        self.qadr = model.jnt_qposadr[jid]
        self.dadr = model.jnt_dofadr[jid]

    def obtener_posicion(self, data):
        return data.xpos[self.body_id]

    def obtener_velocidad(self, data):
        return data.qvel[self.dadr:self.dadr + 3]

    def colocar(self, data, x, y, segmento):
        data.qpos[self.qadr:self.qadr + 3] = [x, y, self.altura_reposo]
        data.qpos[self.qadr + 3:self.qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[self.dadr:self.dadr + 6] = 0.0
        self.segmento = segmento

    def reiniciar(self, data):
        x, y = self.posicion_inicial
        self.colocar(data, x, y, self.segmento_inicial)

    def devolver_a_linea(self, data):
        """Tras entrar al horno vuelve a la Cinta 1."""
        self.colocar(data, -1.05 + 0.18 * (self.indice % 3), -config.LANE, 0)

    def se_ha_caido(self, data):
        x, y, z = self.obtener_posicion(data)
        return z < -0.1 or abs(x) > 5.0 or abs(y) > 2.4

    def recolocar_tras_caida(self, data):
        self.colocar(data, -0.8 + 0.25 * self.indice, -config.LANE, 0)

    def actualizar_segmento(self, pos):
        """Pasa a la siguiente cinta al cruzar la esquina (0→1→2→3→0)."""
        x, y = pos[0], pos[1]
        s = config.CORNER_SWITCH
        if self.segmento == 0 and x >= s:
            self.segmento = 1
        elif self.segmento == 1 and y >= s:
            self.segmento = 2
        elif self.segmento == 2 and x <= -s:
            self.segmento = 3
        elif self.segmento == 3 and y <= -s:
            self.segmento = 0
        return self.segmento

    def bloquear_rotacion(self, data):
        if config.LOCK_OBJECT_ROTATION:
            data.qpos[self.qadr + 3:self.qadr + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[self.dadr + 3:self.dadr + 6] = 0.0

    def centrar_si_cilindro(self, data):
        """Mantiene los cilindros en el centro de la cinta (no ruedan)."""
        if self.forma != "cilindro" or not config.LOCK_CYLINDERS_TO_CENTERLINE:
            return
        q, d = self.qadr, self.dadr
        x, y = data.qpos[q], data.qpos[q + 1]
        t, L = config.TURN_START, config.LANE

        if self.segmento == 0 and x < t:
            data.qpos[q + 1], data.qvel[d + 1] = -L, 0.0
        elif self.segmento == 1 and y < t:
            data.qpos[q], data.qvel[d] = L, 0.0
        elif self.segmento == 2 and x > -t:
            data.qpos[q + 1], data.qvel[d + 1] = L, 0.0
        elif self.segmento == 3 and y > -t:
            data.qpos[q], data.qvel[d] = -L, 0.0
        elif self.segmento == 4:
            data.qpos[q + 1], data.qvel[d + 1] = config.BRANCH_Y, 0.0