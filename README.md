# Práctica I: Planta industrial (POO)

**Asignatura:** Programación Orientada a Objetos (POO), Universidad de Burgos
**Autor:** Naela Khaldi

imulación de una planta industrial en **MuJoCo** (cinco cintas, cinco cámaras,
cinco piezas, un horno y un robot Panda). El código procedural del profesor
(`planta_4_cintas.py`) se ha reorganizado en **clases**.

## Funcionalidades

- Cada cinta guarda su propio estado (en marcha / parada).
- Cada cámara detecta la forma de la pieza con **OpenCV** (círculo, cuadrado, rectángulo o triángulo).
- Cuando detecta una pieza, `objeto_detectado = True` y la cinta se para durante la inspección; después se reanuda.
- Distancia de seguridad entre piezas para que no avancen pegadas.
- Horno con temperatura: cambia de color (azul = frío, naranja = en rango, rojo = sobrecalentado).
- Robot Panda controlado con la cinemática inversa proporcionada (`robot.py`).


## Instalación

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
# source .venv/bin/activate     # Linux / macOS
pip install -r requirements.txt
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

- **CintaTransportadora:** guarda si está en marcha y calcula la fuerza que mueve las piezas.
- **CamaraInspeccion:** detecta la forma con OpenCV (máscara por color, contornos, circularidad
  y número de vértices). Si ve una pieza nueva, `objeto_detectado = True` y bloquea su cinta
  0,3 s; después la reanuda.
- **ObjetoIndustrial:** posición, forma y cinta en la que está cada pieza.
- **Horno:** cuenta las piezas y cambia de color según la temperatura:
  azul (< 50 °C), naranja (50–100 °C) y rojo (> 100 °C).
- **PlantaIndustrial:** crea todos los objetos, ejecuta el bucle, gestiona el teclado y usa
  `robot.py` (IK del profesor, sin modificar). Además mantiene una distancia mínima entre piezas.

## Estructura del proyecto

```
Practica_1/
├── main.py                     
├── requirements.txt           
├── planta_4_cintas.py           
├── capturas/                    
└── src/
    ├── __init__.py
    ├── config.py              
    ├── modelo_planta.py         
    ├── cinta_transportadora.py  
    ├── camara_inspeccion.py     
    ├── objeto_industrial.py     
    ├── horno.py                 
    ├── planta_industrial.py     
    └── robot.py                 
```
