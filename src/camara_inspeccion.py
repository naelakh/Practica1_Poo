import cv2
import mujoco
import numpy as np

from src import config

KERNEL = np.ones((5, 5), np.uint8)


class CamaraInspeccion:
    """Cámara de una cinta: detecta la forma de la pieza y para su cinta un momento."""

    def __init__(self, nombre, titulo, cinta):
        self.nombre = nombre
        self.titulo = titulo
        self.cinta = cinta                 # la cinta que vigila
        self.cam_id = None
        self.imagen = None
        self.contorno = None               # para dibujarlo
        self.objeto_detectado = False      # variable booleana pedida
        self.forma_detectada = None
        self.color_detectado = None
        self.revisados = set()             # colores ya inspeccionados
        self.parada = False                # True mientras esta cámara para la cinta
        self.hora_parada = 0.0
        self.factor_perspectiva = 1.0

    def conectar(self, model):
        self.cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, self.nombre)
        if self.cam_id < 0:
            raise RuntimeError(f"No se encontró la cámara '{self.nombre}'.")

    def capturar(self, renderer, data):
        renderer.update_scene(data, camera=self.nombre)
        return cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

    # ---------- Detección ----------
    def detectar_forma(self, imagen):
        """Busca piezas de cada color en el centro de la imagen y clasifica la mayor."""
        self.contorno = self.color_detectado = None
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        alto, ancho = imagen.shape[:2]
        m = config.VISION_MARGEN_BORDE
        zona_min = ancho * (0.5 - config.VISION_ZONA_CENTRAL)      # franja central de la imagen
        zona_max = ancho * (0.5 + config.VISION_ZONA_CENTRAL)
        piezas = []                        # (área, contorno, color)

        for color, rangos in config.VISION_RANGOS_COLOR.items():
            mascara = np.zeros((alto, ancho), np.uint8)
            for h_min, h_max in rangos:
                mascara |= cv2.inRange(hsv, (h_min, config.VISION_SAT_MIN, config.VISION_VAL_MIN),
                                       (h_max, 255, 255))
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, KERNEL)
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, KERNEL)

            contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contornos:
                area = cv2.contourArea(c)
                x, y, w, h = cv2.boundingRect(c)
                toca_borde = x <= m or y <= m or x + w >= ancho - m or y + h >= alto - m
                en_centro = zona_min <= x + w / 2 <= zona_max
                if area >= config.VISION_AREA_MIN and not toca_borde and en_centro:
                    piezas.append((area, c, color))

        if not piezas:
            return None
        nuevas = [p for p in piezas if p[2] not in self.revisados]    # primero las no revisadas
        _, self.contorno, self.color_detectado = max(nuevas or piezas, key=lambda p: p[0])
        return self.clasificar_forma(self.cara_superior(hsv, self.contorno))

    def cara_superior(self, hsv, contorno):
        """La cara de arriba es la parte más brillante de la pieza (sin los laterales)."""
        brillo = hsv[:, :, 2]
        pieza = np.zeros(brillo.shape, np.uint8)
        cv2.drawContours(pieza, [contorno], -1, 255, -1)
        limite = config.VISION_FACTOR_CARA * int(brillo[pieza > 0].max())
        cara = np.where((pieza > 0) & (brillo >= limite), 255, 0).astype(np.uint8)
        cara = cv2.morphologyEx(cara, cv2.MORPH_OPEN, KERNEL)

        contornos, _ = cv2.findContours(cara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos:
            c = max(contornos, key=cv2.contourArea)
            if cv2.contourArea(c) >= 0.3 * cv2.contourArea(contorno):
                return c
        return contorno

    def clasificar_forma(self, contorno):
        c = contorno.astype(np.float32)
        c[:, :, 1] *= self.factor_perspectiva          # corrige la inclinación de la cámara
        area = cv2.contourArea(c)
        perimetro = cv2.arcLength(c, True)
        circularidad = 4 * np.pi * area / perimetro ** 2 if perimetro > 0 else 0.0
        vertices = len(cv2.approxPolyDP(c, 0.04 * perimetro, True))

        if circularidad > 0.82:
            return "circulo"
        if vertices == 3:
            return "triangulo"
        if vertices == 4:
            (_, _), (w, h), _ = cv2.minAreaRect(c)
            return "cuadrado" if max(w, h) / min(w, h) < config.VISION_PROPORCION_CUADRADO else "rectangulo"
        return "desconocida"

    # ---------- Control de la cinta ----------
    def actualizar(self, renderer, data, tiempo):
        if self.cam_id is not None:                    # factor = 1 / cos(inclinación)
            eje_z = data.cam_xmat[self.cam_id].reshape(3, 3)[:, 2]
            self.factor_perspectiva = 1.0 / max(float(eje_z[2]), 0.3)
        self.imagen = self.capturar(renderer, data)
        self.forma_detectada = self.detectar_forma(self.imagen)
        self.objeto_detectado = self.forma_detectada is not None

        if not self.objeto_detectado:                  # no hay pieza: lista para la siguiente
            self.revisados.clear()
            self.parada = False
            self.cinta.desbloquear(self.nombre)
        elif not self.parada:
            if self.color_detectado not in self.revisados:     # pieza nueva: parar la cinta
                self.parada = True
                self.hora_parada = tiempo
                self.cinta.bloquear(self.nombre)
        elif config.REANUDAR_TRAS_INSPECCION and tiempo - self.hora_parada >= config.TIEMPO_INSPECCION:
            self.parada = False                        # inspección terminada: reanudar
            self.revisados.add(self.color_detectado)
            self.cinta.desbloquear(self.nombre)
        return self.imagen

    def reiniciar(self):
        self.cinta.desbloquear(self.nombre)
        self.revisados.clear()
        self.parada = False
        self.objeto_detectado = False
        self.forma_detectada = self.color_detectado = self.contorno = None

    def dibujar(self):
        img = self.imagen.copy()
        if self.contorno is not None:
            color = (0, 0, 255) if self.parada else (0, 255, 0)      # rojo = inspeccionando
            cv2.drawContours(img, [self.contorno], -1, color, 2)
        if self.objeto_detectado:
            texto = f"DETECTADO: {self.forma_detectada}" + (" (INSPECCION)" if self.parada else "")
        else:
            texto = "SIN PIEZA"
        cv2.putText(img, texto, (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        return img