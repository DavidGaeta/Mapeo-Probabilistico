from vpython import *
import numpy as np
from scipy.spatial import KDTree
import heapq

L = 5
gray = color.gray(0.7)
red = vector(1, 0, 0)


animation = canvas(width=1000, height=800, align='center',background=color.white)
animation.range = L
animation.title = 'Robótica móvil'
animation.caption = "Mapeado de trayectoria"


animation.camera.pos = vector(L/2, L/2, L/2 + 10)  
#animation.camera.axis = vector(0, 0, 1)  #

# Dibujar el cubo
d = L
h = 2*d
r = 0.005
boxbottom = curve(color=gray, radius=r)
boxbottom.append([vector(0, 0, 0), vector(0, 0, d), vector(d, 0, d), vector(d, 0, 0), vector(0, 0, 0)])
boxtop = curve(color=gray, radius=r)
boxtop.append([vector(0, d, 0), vector(0, d, d), vector(d, d, d), vector(d, d, 0), vector(0, d, 0)])
vert1, vert2, vert3, vert4 = [curve(color=gray, radius=r) for _ in range(4)]
vert1.append([vector(0, 0, 0), vector(0, d, 0)])
vert2.append([vector(0, 0, d), vector(0, d, d)])
vert3.append([vector(d, 0, d), vector(d, d, d)])
vert4.append([vector(d, 0, 0), vector(d, d, 0)])
# Parámetros
N = 700  # Número de puntos
N_obs = 3  # Número de obstáculos
max_vecinos = 5  # Máximo de conexiones por nodo
obs_radius = 2  # Radio de los obstáculos

# Definir puntos de inicio y final
inicio = np.array([5, 5, 5])
final = np.array([0, 0, 0])

# Dibujar inicio y final
sphere(pos=vector(*final), radius=0.15, color=color.red)
sphere(pos=vector(*inicio), radius=0.15, color=color.green)

# Generar puntos y los obstaculos aleatorios en el espacio
puntos = np.random.uniform(0, L, (N, 3))
puntos = np.vstack([inicio, puntos, final]) # Concatena el inicio y final para simplificar 
obstaculos = np.random.uniform(0, L, (N_obs, 3))

# Genera los obstaculos en el mapa
for obst in obstaculos:
    sphere(pos=vector(*obst), radius=obs_radius, color=color.gray(0.5))

# Genera los puntos en el mapa
esferas = []
for punto in puntos:
    color_punto = color.green if np.all(punto == inicio) else color.red if np.all(punto == final) else color.blue
    esferas.append(sphere(pos=vector(*punto), radius=0.05, color=color_punto))

# verificacion de conexiones
def colision_libre(p1, p2):
    for obs in obstaculos:
        d = np.linalg.norm(np.cross(p2 - p1, obs - p1)) / np.linalg.norm(p2 - p1)
        if d < obs_radius:
            return False  # Colisión detectada
    return True  # Conexión válida

# 
grafo = {tuple(p): [] for p in puntos}
aristas = []

#Deepsek ayudo en esto 10/10
tree = KDTree(puntos)
for i, punto in enumerate(puntos):
    _, vecinos_idx = tree.query(punto, k=max_vecinos+1)  # Obtener vecinos más cercanos
    for j in vecinos_idx[1:]:  # Saltar el primer elemento (es el propio nodo)
        vecino = puntos[j]
        if colision_libre(punto, vecino):  # Conectar solo si no hay colisión
            aristas.append((punto, vecino))


for arista in aristas: # For para conectar aristas.
    p1, p2 = arista
    curve(pos=[vector(*p1), vector(*p2)], color=color.orange, radius=0.01)

def dijkstra(grafo, inicio, final):
    inicio, final = tuple(inicio), tuple(final)
    distancias = {nodo: float('inf') for nodo in grafo}
    distancias[inicio] = 0
    camino = {}
    heap = [(0, inicio)]

    while heap:
        dist_actual, nodo_actual = heapq.heappop(heap)
        if nodo_actual == final:
            ruta = []
            while nodo_actual in camino:
                ruta.append(nodo_actual)
                nodo_actual = camino[nodo_actual]
            ruta.append(inicio)
            return ruta[::-1]

        for vecino, peso in grafo[nodo_actual]:
            distancia_nueva = dist_actual + peso
            if distancia_nueva < distancias[vecino]:
                distancias[vecino] = distancia_nueva
                camino[vecino] = nodo_actual
                heapq.heappush(heap, (distancia_nueva, vecino))

    return None

ruta = dijkstra(grafo, inicio, final)

ruta = dijkstra(grafo, inicio, final)

# Dibujar la mejor ruta en verde

if ruta:
    for i in range(len(ruta) - 1):
        curve(pos=[vector(*ruta[i]), vector(*ruta[i+1])], color=color.green, radius=0.15)

#  Cilindro, me falta implementación de movimiento.
bar = cylinder(pos=vector(*inicio), axis=vector(0.1, 0.1, 0.1), radius=0.1, color=color.blue)

#Animacion, se va muy rapida.
if ruta:
    for punto in ruta:
        bar.pos = vector(*punto)
        rate(5)  # Controla la velocidad del movimiento

print("Simulación completada.")