import mujoco
from src import config

class Horno:
    """Cuenta las piezas que entran y controla la temperatura (y su color)."""

    COLOR_FRIO = [0.1, 0.3, 1.0, 1.0]        # azul
    COLOR_EN_RANGO = [1.0, 0.6, 0.1, 1.0]    # naranja
    COLOR_CALIENTE = [1.0, 0.1, 0.1, 1.0]    # rojo

    def __init__(self, nombre_geoms=config.HORNO_GEOMS, nombre_luz=config.HORNO_LUZ,
                 temperatura_inicial=config.HORNO_TEMP_INICIAL):
        self.nombre_geoms = nombre_geoms
        self.nombre_luz = nombre_luz
        self.geom_ids = []
        self.luz_id = -1
        self.numero_piezas = 0          # sustituye a oven_counts
        self.ultima_pieza = None
        self.temperatura = temperatura_inicial
        self.temperatura_min = config.HORNO_TEMP_MIN
        self.temperatura_max = config.HORNO_TEMP_MAX
        self.entrada_x = config.OVEN_ENTRY_X

    def conectar(self, model):
        self.geom_ids = []
        for nombre in self.nombre_geoms:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, nombre)
            if gid < 0:
                raise RuntimeError(f"No se encontró el geom '{nombre}' del horno.")
            self.geom_ids.append(gid)
        self.luz_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_LIGHT, self.nombre_luz)

    # ---------- Piezas ----------
    def pieza_entra(self, pieza, pos):
        return pieza.segmento == 4 and pos[0] <= self.entrada_x

    def registrar_pieza(self, pieza=None):
        self.numero_piezas += 1
        self.ultima_pieza = pieza

    def reiniciar(self):
        self.numero_piezas = 0
        self.ultima_pieza = None

    # ---------- Temperatura ----------
    def aumentar_temperatura(self, incremento):
        self.temperatura = min(self.temperatura + incremento, config.HORNO_TEMP_LIMITE)

    def disminuir_temperatura(self, decremento):
        self.temperatura = max(self.temperatura - decremento, 0.0)

    def obtener_temperatura(self):
        return self.temperatura

    def obtener_estado(self):
        if self.temperatura < self.temperatura_min:
            return "FRIO"
        if self.temperatura > self.temperatura_max:
            return "SOBRECALENTADO"
        return "EN RANGO"

    def obtener_color(self):
        estado = self.obtener_estado()
        if estado == "FRIO":
            return self.COLOR_FRIO
        if estado == "SOBRECALENTADO":
            return self.COLOR_CALIENTE
        return self.COLOR_EN_RANGO

    def actualizar_color(self, model):
        color = self.obtener_color()
        emision = 0.2 + 0.8 * (self.temperatura / config.HORNO_TEMP_LIMITE)   # más brillo si más caliente
        for gid in self.geom_ids:
            model.geom_rgba[gid] = color
            mat_id = model.geom_matid[gid]
            if mat_id >= 0:
                model.mat_rgba[mat_id] = color
                model.mat_emission[mat_id] = emision
        if self.luz_id >= 0:
            model.light_diffuse[self.luz_id] = color[:3]