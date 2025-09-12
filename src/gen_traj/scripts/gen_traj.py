#!/usr/bin/env python3

#Imports généraux
import math

#Imports de ROS
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

#Initialisation du node
rospy.init_node("Generation_de_trajectoire", anonymous=True)

# constantes
h_table = 0
h_objet = 30e-2  # Hauteur de l'objet
largeur_max_obj = 20e-2  # largeur de l'objet

A = largeur_max_obj  # Rayon du cercle autour de l'objet

dtheta = 0.174533  # pas angulaire en radian (environ 10 deg)

portee_max = 5  # Portée max du capteur
dist_secu = 15e-2  # distance de sécurité

dt = 0.1  # pas de temps

# variables
x_traj, y_traj, z_traj = [], [], []
scan_points = []  # pour stocker les positions de scan

t = 0
theta = 0
c_x, c_y = 0.5, 0.5

critere_arret = True
global movement_status
movement_status = False

# Fonctions

def init_publisher():
    '''
    Initialise le publisher sur le topic /target_coordinates
    '''
    pub = rospy.Publisher("/target_coordinates", Point, queue_size=10)
    rate = rospy.Rate(10) #10 HZ
    return pub, rate

def publish_target(pub, rate, point: Point):
    """
    Publie un Point sur le topic /target_coordinates
    """

    #Publication
    rospy.loginfo(f"Publication du point cible: x={point.x}, y={point.y}, z={point.z}")
    pub.publish(point)
    rate.sleep()  # petite pause pour s'assurer que le message est bien envoyé

def callback(status):
    global movement_status
    movement_status = status.data
    rospy.loginfo(f'Movement status recu: {movement_status}')
    

def subscribe_movement_status():
    rospy.Subscriber("/movement_status", Bool, callback)


def attendre_fin_mouvement(rate):
    '''
    Attend que movement_status soit true
    '''
    global movement_status
    rospy.loginfo("Attente du signal de movement_status...")
    while not rospy.is_shutdown() and not movement_status:
        rate.sleep()
    movement_status = False #reset pour le prochain usage

def get_arret(nb_scan):
    return nb_scan < 4



# Initialiser le publisher
pub, rate = init_publisher()
#Subscribe au topic
subscribe_movement_status()

# Attendre que le subscriber soit pret
rospy.sleep(2.0)
rospy.loginfo("Demarrage de la generation de trajectoire")



# position initiale
x = c_x
y = c_y
z = 1

target = Point(x, y, z)
publish_target(pub, rate, target)
attendre_fin_mouvement(rate)

x_traj.append(x)
y_traj.append(y)
z_traj.append(z)

# point de scan -> stocké en vert
scan_points.append((x, y, z))


theta_cible = 0

# génération trajectoire autour de l'objet

while critere_arret:
    
    while theta <= theta_cible and not rospy.is_shutdown(): # verifier l'etat de ROS
        t = t + dt
        x = A * math.cos(theta) + c_x
        y = A * math.sin(theta) + c_y
        z = h_objet/2   # pour le moment à mi-hauteur
        theta += dtheta
        target = Point(x, y, z)
        publish_target(pub,rate, target)
        attendre_fin_mouvement(rate)
        x_traj.append(x)
        y_traj.append(y)
        z_traj.append(z)

    # point de scan -> stocké en vert
    scan_points.append((x, y, z))

    t = 0
    critere_arret = get_arret(len(scan_points))
    theta_cible = theta + 2*math.pi/3



