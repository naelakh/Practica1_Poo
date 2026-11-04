import time

import cv2
import mujoco
import mujoco.viewer
import numpy as np

from src import config
from src import robot as panda_ik      # IK del profesor (sin modificar)
from src.camara_inspeccion import CamaraInspeccion
from src.cinta_transportadora import CintaTransportadora
from src.horno import Horno
from src.modelo_planta import build_mjcf
from src.objeto_industrial import ObjetoIndustrial

VENTANA = "Camaras de la planta"
AYUDA = "Q/ESC salir | P pausa | 1-5 cintas | R reset | S captura | +/- horno"
L = config.LANE
v = config.BELT_SPEED

# numero, velocidad, direccion, giro, eje centrado, centro, actuadores, signo rodillo
DATOS_CINTAS = [
    (1, v, [1, 0], [0, 1], 1, -L, ("belt_act_b1", "belt_act_b2"), +1),
    (2, v, [0, 1], [-1, 0], 0, L, ("belt_act_r1", "belt_act_r2"), -1),
    (3, v, [-1, 0], [0, -1], 1, L, ("belt_act_t1", "belt_act_t2"), -1),
    (4, v, [0, -1], [1, 0], 0, -L, ("belt_act_l1", "belt_act_l2"), +1),
    (5, config.BRANCH_SPEED, [-1, 0], None, 1, config.BRANCH_Y, ("belt_act_o1", "belt_act_o2"), -1),
]
# indice, forma, color, posicion inicial, segmento
DATOS_OBJETOS = [
    (0, "caja", "rojo", (-L, 0.95), 3),
    (1, "cilindro", "azul", (L, -0.72), 1),
    (2, "caja", "amarillo", (0.58, L), 2),
    (3, "cilindro", "verde", (-0.62, -L), 0),
    (4, "caja", "morado", (-0.45, L), 2),
]
DATOS_CAMARAS = [
    ("cam_bottom", "CINTA 1 - INFERIOR - +X"),
    ("cam_right", "CINTA 2 - DERECHA - +Y"),
    ("cam_top", "CINTA 3 - SUPERIOR - -X"),
    ("cam_left", "CINTA 4 - IZQUIERDA - -Y"),
    ("cam_oven", "CINTA 5 - HORNO - -X"),
]
class PlantaIndustrial:
    """Une cintas, cámaras, piezas, horno y robot, y ejecuta la simulación."""

    def __init__(self):
        self.cintas = [CintaTransportadora(d[0], f"CINTA {d[0]}", *d[1:]) for d in DATOS_CINTAS]
        self.objetos = [ObjetoIndustrial(*d) for d in DATOS_OBJETOS]
        self.camaras = [CamaraInspeccion(nombre, titulo, cinta)        # cada cámara con su cinta
                        for (nombre, titulo), cinta in zip(DATOS_CAMARAS, self.cintas)]
        self.horno = Horno()

        self.robot_config = panda_ik.make_config(lane=L, branch_y=config.BRANCH_Y,
                                                 branch_speed=config.BRANCH_SPEED)
        self.robot_estado = panda_ik.new_robot_state()
        self.robot_ids = None
        self.modo_prueba_ik = config.ROBOT_IK_TEST_MODE

        self.model = self.data = self.renderer = self.viewer = None
        self.pausada = False
        self.en_marcha = True
        self.ultimo_tiempo_camaras = -1.0
        self.mosaico = None

    # ================= Inicio =================
    def inicializar(self):
        config.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        cfg = self.robot_config
        xml = build_mjcf(cfg["base_x"], cfg["base_y"], cfg["base_z"])
        self.model = panda_ik.build_model_with_franka(xml, cfg)
        self.data = mujoco.MjData(self.model)

        for elemento in self.objetos + self.camaras + [self.horno]:
            elemento.conectar(self.model)
        self.robot_ids = panda_ik.get_robot_ids(self.model)

        self.reiniciar_objetos()
        self.inicializar_robot()
        self.actualizar_rodillos()
        self.horno.actualizar_color(self.model)

        self.renderer = mujoco.Renderer(self.model, height=config.CAM_H, width=config.CAM_W)
        if config.SHOW_CAMERA_MOSAIC:
            cv2.namedWindow(VENTANA, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(VENTANA, config.CAM_W * 3, config.CAM_H * 2)

        print("\n=== PLANTA INDUSTRIAL (POO): 5 CINTAS + 5 CAMARAS + PANDA + HORNO ===")
        print("Teclas:", AYUDA)
        if self.modo_prueba_ik:
            print("Modo IK TEST: T = pose objetivo | H = pose HOME")
            print("Objetivo IK:", panda_ik.ik_target_text(self.robot_estado))
        else:
            print("Modo robot: pick-and-place automatico")
        self.imprimir_horno()
        print()

        if config.SHOW_MUJOCO_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.viewer.cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")

    def imprimir_horno(self):
        print(f"Horno: {self.horno.obtener_temperatura():.0f} C ({self.horno.obtener_estado()})")

    def reiniciar_objetos(self):
        for pieza in self.objetos:
            pieza.reiniciar(self.data)
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def actualizar_rodillos(self):
        for cinta in self.cintas:
            cinta.actualizar_rodillos(self.model, self.data)

    def reiniciar(self):
        self.reiniciar_objetos()
        self.horno.reiniciar()
        for camara in self.camaras:
            camara.reiniciar()
        self.robot_estado.clear()
        self.robot_estado.update(panda_ik.new_robot_state())
        self.inicializar_robot()
        self.actualizar_rodillos()
        print("Planta y Panda reiniciados")

    # ================= Transporte =================
    def aplicar_transporte(self):
        self.data.xfrc_applied[:] = 0.0
        robot = self.robot_estado
        for pieza in self.objetos:
            if robot["grasped_obj"] == pieza.indice:          # la lleva el robot
                continue
            pos = pieza.obtener_posicion(self.data)
            vel = pieza.obtener_velocidad(self.data)

            if robot["target_obj"] == pieza.indice and robot["state"] != "idle":
                fx, fy = panda_ik.robot_hold_force(pos, vel, self.robot_config, config.MAX_FORCE)
            else:
                pieza.actualizar_segmento(pos)
                cinta = self.cintas[pieza.segmento]
                if self.hay_pieza_delante(pieza, pos, cinta):
                    fx, fy = cinta.fuerza_frenado(vel)          # guarda la distancia
                else:
                    fx, fy = cinta.calcular_fuerza(pos, vel)
            self.data.xfrc_applied[pieza.body_id, 0] = fx
            self.data.xfrc_applied[pieza.body_id, 1] = fy

            if self.horno.pieza_entra(pieza, pos):
                self.horno.registrar_pieza(pieza)
                pieza.devolver_a_linea(self.data)
            elif pieza.se_ha_caido(self.data):
                pieza.recolocar_tras_caida(self.data)

    def hay_pieza_delante(self, pieza, pos, cinta):
        """True si otra pieza está muy cerca por delante (en el sentido de la cinta)."""
        for otra in self.objetos:
            if otra is pieza:
                continue
            pos_otra = otra.obtener_posicion(self.data)
            d = np.array([pos_otra[0] - pos[0], pos_otra[1] - pos[1]])
            if np.linalg.norm(d) < config.DISTANCIA_SEGURIDAD and np.dot(d, cinta.direccion) > 0:
                return True
        return False

    def paso_simulacion(self):
        self.actualizar_maquina_estados_robot()
        self.aplicar_transporte()
        self.controlar_robot()
        mujoco.mj_step(self.model, self.data)
        for pieza in self.objetos:
            pieza.bloquear_rotacion(self.data)
            pieza.centrar_si_cilindro(self.data)
        if not self.modo_prueba_ik:
            panda_ik.follow_tcp_with_grasped_object(self.model, self.data, self.robot_ids, self.robot_estado)
        mujoco.mj_forward(self.model, self.data)

    # ================= Robot (usa src/robot.py) =================
    def inicializar_robot(self):
        panda_ik.initialize_robot(self.model, self.data, self.robot_ids, self.robot_estado)
        if self.modo_prueba_ik:
            self.robot_a_home()             # empieza lejos de las cintas

    def robot_a_home(self):
        panda_ik.set_ik_target_home(self.model, self.robot_ids, self.robot_estado)
        mujoco.mj_forward(self.model, self.data)

    def robot_a_pose_prueba(self):
        panda_ik.set_ik_target_rpy(self.model, self.robot_ids, self.robot_estado,
                                   np.array(config.ROBOT_IK_TARGET_POSITION, dtype=float),
                                   np.array(config.ROBOT_IK_TARGET_RPY_DEG, dtype=float),
                                   orientation_frame=config.ROBOT_IK_ORIENTATION_FRAME)
        mujoco.mj_forward(self.model, self.data)

    def actualizar_maquina_estados_robot(self):
        if self.modo_prueba_ik:
            return
        segmentos = [p.segmento for p in self.objetos]
        panda_ik.update_robot_state_machine(self.model, self.data, [p.body_id for p in self.objetos],
                                            self.robot_ids, self.robot_estado, segmentos,
                                            config.OBJECT_REST_Z, self.robot_config)
        for pieza, seg in zip(self.objetos, segmentos):     # el robot puede llevarla a la cinta 5
            pieza.segmento = seg

    def controlar_robot(self):
        if self.modo_prueba_ik:
            panda_ik.control_manual_ik(self.model, self.data, self.robot_ids, self.robot_estado,
                                       gripper_open=config.ROBOT_IK_GRIPPER_OPEN)
        else:
            panda_ik.control_robot_ik(self.model, self.data, self.robot_ids, self.robot_estado, self.robot_config)
            panda_ik.follow_tcp_with_grasped_object(self.model, self.data, self.robot_ids, self.robot_estado)

    # ================= Cámaras y mosaico =================
    def texto_estado(self):
        cintas = " ".join(f"C{c.numero}:{'ON' if c.esta_activa() else 'OFF'}" for c in self.cintas)
        return (f"t={self.data.time:5.1f}s {'PAUSA' if self.pausada else 'MARCHA'} "
                f"{cintas} Panda={self.robot_estado['state']} ciclos={self.robot_estado['cycles']} "
                f"horno={self.horno.numero_piezas} {self.horno.obtener_temperatura():.0f}C")

    def etiquetar(self, imagen, titulo, estado):
        img = imagen.copy()
        cv2.rectangle(img, (0, 0), (img.shape[1], 36), (20, 20, 20), -1)
        cv2.putText(img, titulo, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (245, 245, 245), 2, cv2.LINE_AA)
        cv2.putText(img, estado, (12, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (245, 245, 245), 1, cv2.LINE_AA)
        return img

    def actualizar_camaras(self):
        t = self.data.time
        if not (self.ultimo_tiempo_camaras < 0.0 or self.pausada
                or t - self.ultimo_tiempo_camaras >= 1.0 / config.CAM_FPS):
            return                                          # solo 10 veces por segundo
        self.ultimo_tiempo_camaras = t

        estado = self.texto_estado()
        imagenes = []
        for camara in self.camaras:
            camara.actualizar(self.renderer, self.data, t)  # detecta y para su cinta
            imagenes.append(self.etiquetar(camara.dibujar(), camara.titulo, estado))
        self.renderer.update_scene(self.data, camera="overview")
        general = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        imagenes.append(self.etiquetar(general, "VISTA GENERAL", estado))

        self.actualizar_rodillos()          # las cámaras pueden haber parado cintas
        self.mostrar_mosaico(imagenes)

    def mostrar_mosaico(self, imagenes):
        """Junta las 6 imágenes en 2 filas de 3 y las muestra."""
        self.mosaico = np.vstack((np.hstack(imagenes[0:3]), np.hstack(imagenes[3:6])))
        cv2.putText(self.mosaico, AYUDA, (12, self.mosaico.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        if config.SHOW_CAMERA_MOSAIC:
            cv2.imshow(VENTANA, self.mosaico)

    # ================= Teclado =================
    def procesar_teclado(self):
        tecla = cv2.waitKey(1) & 0xFF
        if tecla in (ord('q'), 27):
            self.en_marcha = False
        elif tecla == ord('p'):
            self.pausada = not self.pausada
            print("PAUSA GLOBAL" if self.pausada else "SIMULACION REANUDADA")
        elif ord('1') <= tecla <= ord('5'):
            cinta = self.cintas[tecla - ord('1')]
            activa = cinta.conmutar()
            cinta.actualizar_rodillos(self.model, self.data)
            print(f"Cinta {cinta.numero}: {'ON' if activa else 'OFF'}")
        elif tecla == ord('r'):
            self.reiniciar()
        elif tecla == ord('t') and self.modo_prueba_ik:
            self.robot_a_pose_prueba()
            print("[IK] Objetivo:", panda_ik.ik_target_text(self.robot_estado))
        elif tecla == ord('h') and self.modo_prueba_ik:
            self.robot_a_home()
            print("[IK] HOME cartesiano")
        elif tecla == ord('s') and self.mosaico is not None:
            ruta = config.OUTPUT_DIR / f"mosaico_{int(time.time())}.png"
            cv2.imwrite(str(ruta), self.mosaico)
            print(f"Captura guardada: {ruta}")
        elif tecla == ord('+'):
            self.horno.aumentar_temperatura(config.HORNO_PASO_TEMP)
            self.imprimir_horno()
        elif tecla == ord('-'):
            self.horno.disminuir_temperatura(config.HORNO_PASO_TEMP)
            self.imprimir_horno()

    # ================= Bucle principal =================
    def actualizar(self):
        if not self.pausada:
            self.paso_simulacion()
        self.horno.actualizar_color(self.model)
        if self.viewer is not None:
            self.viewer.sync()
        self.actualizar_camaras()
        if config.SHOW_CAMERA_MOSAIC:
            self.procesar_teclado()

    def ejecutar(self):
        self.inicializar()

        # Bucle: se repite hasta pulsar Q/ESC o cerrar la ventana de MuJoCo
        while self.en_marcha and (self.viewer is None or self.viewer.is_running()):
            inicio = time.perf_counter()
            self.actualizar()
            espera = self.model.opt.timestep - (time.perf_counter() - inicio)
            if espera > 0:
                time.sleep(espera)

        self.cerrar()

    def cerrar(self):
        """Cierra las ventanas al terminar."""
        if self.viewer is not None:
            self.viewer.close()
        if self.renderer is not None:
            self.renderer.close()
        cv2.destroyAllWindows()