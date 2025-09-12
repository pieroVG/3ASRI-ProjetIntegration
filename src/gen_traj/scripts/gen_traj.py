#!/usr/bin/env python3

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

import rospy
from geometry_msgs.msg import Point

# Fonctions

def publish_target(point: Point):
    """
    Publie un Point sur le topic /target_coordinates
    """
    pub = rospy.Publisher("/target_coordinates", Point, queue_size=10)
    rospy.init_node("target_publisher", anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # On publie une fois (ou en boucle si tu veux répéter)
    rospy.loginfo(f"Publication du point cible: x={point.x}, y={point.y}, z={point.z}")
    pub.publish(point)

    rate.sleep()  # petite pause pour s'assurer que le message est bien envoyé


def get_arret(nb_scan):
    return nb_scan < 4

# constantes
h_table = 0
h_objet = 30e-3  # Hauteur de l'objet
largeur_max_obj = 20e-3  # largeur de l'objet

A = largeur_max_obj  # Rayon du cercle autour de l'objet

dtheta = 0.174533  # pas angulaire en radian (environ 10 deg)

portee_max = 5  # Portée max du capteur
dist_secu = 15e-3  # distance de sécurité

dt = 0.1  # pas de temps

# variables
x_traj, y_traj, z_traj = [], [], []
scan_points = []  # pour stocker les positions de scan

t = 0
teta = 0
c_x, c_y = 0.5, 0.5

nb_scan = 0

critere_arret = True
# position initiale
x = c_x
y = c_y
z = 1

target = Point(x, y, z)
publish_target(target)


x_traj.append(x)
y_traj.append(y)
z_traj.append(z)

# point de scan -> stocké en vert
scan_points.append((x, y, z))


teta_cible = 0
# génération trajectoire autour de l'objet
while critere_arret:
    
    while teta <= teta_cible:
        t = t + dt
        x = A * math.cos(teta) + c_x
        y = A * math.sin(teta) + c_y
        z = h_objet/2   # pour le moment à mi-hauteur
        teta += dtheta
        target = Point(x, y, z)
        publish_target(target)
	
        x_traj.append(x)
        y_traj.append(y)
        z_traj.append(z)

    # point de scan -> stocké en vert
    scan_points.append((x, y, z))

    t = 0
    critere_arret = get_arret(len(scan_points))
    teta_cible = teta + 2*math.pi/3



'''
# --- Création de la figure ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Trajectoire du bras
ax.plot(x_traj, y_traj, z_traj, label='Trajectoire')
#ax.scatter(x_traj, y_traj, z_traj, c='r', marker='o')

# Points de scan en vert
if scan_points:
    xs, ys, zs = zip(*scan_points)
    ax.scatter(xs, ys, zs, c='g', marker='^', s=60, label='Scans')

# --- Tracer le cube représentant l’objet ---
r = largeur_max_obj / 2
# sommets du cube
x_cube = [c_x - r, c_x + r]
y_cube = [c_y - r, c_y + r]
z_cube = [h_table, h_table + h_objet]

# créer les faces du cube
xx, yy = np.meshgrid(x_cube, y_cube)
ax.plot_surface(xx, yy, z_cube[0]*np.ones_like(xx), alpha=0.3, color='blue')  # base
ax.plot_surface(xx, yy, z_cube[1]*np.ones_like(xx), alpha=0.3, color='blue')  # top

yy, zz = np.meshgrid(y_cube, z_cube)
ax.plot_surface(x_cube[0]*np.ones_like(yy), yy, zz, alpha=0.3, color='blue')  # face x-
ax.plot_surface(x_cube[1]*np.ones_like(yy), yy, zz, alpha=0.3, color='blue')  # face x+

xx, zz = np.meshgrid(x_cube, z_cube)
ax.plot_surface(xx, y_cube[0]*np.ones_like(xx), zz, alpha=0.3, color='blue')  # face y-
ax.plot_surface(xx, y_cube[1]*np.ones_like(xx), zz, alpha=0.3, color='blue')  # face y+

# légendes et labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()

plt.show()
'''
