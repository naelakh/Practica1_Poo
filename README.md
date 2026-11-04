# Práctica I: Planta industrial (POO)

**Asignatura:** Programación Orientada a Objetos (POO), Universidad de Burgos
**Autora:** Naela Khaldi

Simulación de una planta industrial en **MuJoCo** con cinco cintas transportadoras,
cinco cámaras de inspección, cinco piezas, un horno y un robot Franka Emika Panda.
El código procedural del profesor (`planta_4_cintas.py`) se ha reorganizado
siguiendo la **programación orientada a objetos**.

## Funcionalidades

- Cada cinta guarda su propio estado (en marcha / parada).
- Cada cámara detecta la forma de la pieza con **OpenCV** (círculo, cuadrado, rectángulo o triángulo).
- Cuando detecta una pieza, `objeto_detectado = True` y la cinta se para durante la inspección; después se reanuda.
- Distancia de seguridad entre piezas para que no avancen pegadas.
- Horno con temperatura: cambia de color (azul = frío, naranja = en rango, rojo = sobrecalentado).
- Robot Panda controlado con la cinemática inversa proporcionada (`robot.py`).

## Requisitos

- Python 3.10 o superior
- Las librerías de `requirements.txt` (MuJoCo, OpenCV y NumPy)
- Git y conexión a Internet la primera vez (para descargar el modelo del robot)

## Instalación

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
# source .venv/bin/activate     # Linux / macOS
pip install -r requirements.txt
```

El modelo del Panda se toma de **MuJoCo Menagerie**. Si no está en la carpeta del
proyecto ni en la carpeta superior, `robot.py` lo descarga automáticamente con Git.
También se puede clonar a mano:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

## Ejecución

Desde la carpeta raíz del proyecto:

```bash
python main.py
```

Se abren dos ventanas:

- **MuJoCo**: la planta en 3D.
- **Camaras de la planta**: mosaico con las 5 cámaras de inspección y la vista general.

## Teclas

Con la ventana *Camaras de la planta* seleccionada (letras en minúscula):

| Tecla | Acción |
|-------|--------|
| `Q` / `ESC` | Salir |
| `P` | Pausa / continuar |
| `1`–`5` | Arrancar / parar cada cinta |
| `R` | Reiniciar la planta |
| `S` | Guardar captura del mosaico en `capturas/` |
| `+` / `-` | Subir / bajar la temperatura del horno (10 °C) |
| `T` / `H` | Robot a la pose de prueba / a HOME |

## Cómo funciona

### Idea general

Cada elemento de la planta es un **objeto** con su propio estado (atributos) y su
comportamiento (métodos). `PlantaIndustrial` crea todos los objetos a partir de las
listas `DATOS_CINTAS`, `DATOS_OBJETOS` y `DATOS_CAMARAS` y los coordina.

```
main.py
 └── PlantaIndustrial
      ├── 5 × CintaTransportadora
      ├── 5 × ObjetoIndustrial
      ├── 5 × CamaraInspeccion   (cada una controla su cinta)
      ├── Horno
      └── robot.py               (IK del profesor, usada como caja negra)
```

### Un paso de simulación

`ejecutar()` repite `actualizar()` en bucle. En cada paso:

1. **Robot**: actualiza su máquina de estados (solo en modo automático).
2. **Transporte**: cada pieza pide a su cinta la fuerza (`calcular_fuerza()`), que se
   aplica con `data.xfrc_applied`. Si hay otra pieza muy cerca delante
   (`hay_pieza_delante()`), se frena. Al pasar una esquina cambia de cinta
   (`actualizar_segmento()`).
3. **Física**: `mujoco.mj_step()`.
4. **Estabilizar**: las piezas no giran y los cilindros se mantienen en el centro.
5. **Horno**: actualiza su color según la temperatura.
6. **Cámaras**: 10 veces por segundo, cada cámara detecta y, si hace falta, para su cinta.

### Detección de formas (`CamaraInspeccion`)

1. Pasa la imagen a HSV y crea **una máscara por color** de pieza (así dos piezas juntas no se mezclan).
2. Busca los contornos con `cv2.findContours`.
3. Ignora los contornos pequeños, los que tocan el borde y los que no están en el centro.
4. Usa solo la **cara superior** de la pieza y corrige la inclinación de la cámara.
5. Clasifica la forma:
   - circularidad > 0,82 → `circulo`
   - 3 vértices → `triangulo`
   - 4 vértices → `cuadrado` (lados casi iguales) o `rectangulo`

### Parada de la cinta

- Si aparece una pieza nueva, la cámara pone `objeto_detectado = True` y bloquea
  su cinta con `cinta.bloquear()`.
- Pasado `TIEMPO_INSPECCION` (0,3 s, en `config.py`), la desbloquea y la pieza sigue.
- El conjunto `revisados` evita inspeccionar dos veces la misma pieza.
- Se usa `bloquear()` en vez de `detener()` para no anular la parada manual con las teclas 1–5.

### Horno (`Horno`)

- Cuenta las piezas que entran (`pieza_entra()`, `registrar_pieza()`).
- La temperatura se cambia con `+` y `-` y siempre queda entre 0 °C y 200 °C.
- El color en MuJoCo depende del estado:

| Temperatura | Estado | Color |
|-------------|--------|-------|
| menos de 50 °C | FRIO | azul |
| de 50 a 100 °C | EN RANGO | naranja |
| más de 100 °C | SOBRECALENTADO | rojo |

## Estructura del proyecto

```
Practica_1/
├── main.py                      # Único punto de entrada
├── requirements.txt             # Librerías necesarias
├── planta_4_cintas.py           # Código original del profesor (referencia)
├── capturas/                    # Capturas guardadas con la tecla S
└── src/
    ├── __init__.py
    ├── config.py                # Constantes
    ├── modelo_planta.py         # Genera el XML de MuJoCo
    ├── cinta_transportadora.py  # Clase CintaTransportadora
    ├── camara_inspeccion.py     # Clase CamaraInspeccion
    ├── objeto_industrial.py     # Clase ObjetoIndustrial
    ├── horno.py                 # Clase Horno
    ├── planta_industrial.py     # Clase PlantaIndustrial
    └── robot.py                 # IK del Panda (del profesor, sin modificar)
```
