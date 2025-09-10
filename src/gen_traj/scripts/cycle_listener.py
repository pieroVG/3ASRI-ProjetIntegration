#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import tf.transformations
import math
import numpy as np
from geometry_msgs.msg import Pose, Point

class RobotController:
    def __init__(self):
        """Initialise le contrôleur du robot, MoveIt et le subscriber."""
        # --- Initialisations ROS et MoveIt ---
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_look_at_controller', anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("hc10_arm")
        
        # Point fixe que le robot doit toujours regarder
        self.look_at_point = Point(x=0.3, y=0.0, z=0.0) # z sera mis à jour dynamiquement

        # --- Subscriber ---
        # Le noeud écoute les messages de type Point sur le topic /target_coordinates
        self.target_sub = rospy.Subscriber("/target_coordinates", Point, self.coordinate_callback)
        
        rospy.loginfo("Contrôleur initialisé. En attente de coordonnées sur /target_coordinates...")

    def go_to_initial_pose(self):
        """Amène le robot à sa position de départ."""
        rospy.loginfo("Déplacement vers la position initiale...")
        
        # Interprétation de [0.3, 0, 1, 0, 0, 0] comme [x,y,z, roll,pitch,yaw]
        initial_pose = Pose()
        initial_pose.position.x = 0.3
        initial_pose.position.y = 0.0
        initial_pose.position.z = 1.0
        
        # roll=0, pitch=0, yaw=0 -> quaternion identité (pas de rotation)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        initial_pose.orientation.x = q[0]
        initial_pose.orientation.y = q[1]
        initial_pose.orientation.z = q[2]
        initial_pose.orientation.w = q[3]

        self.move_group.set_pose_target(initial_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("Position initiale atteinte.")
        else:
            rospy.logerr("Échec du déplacement vers la position initiale.")
            # On quitte si on ne peut pas atteindre la position de départ
            rospy.signal_shutdown("Impossible d'atteindre la pose initiale")

    def calculate_look_at_quaternion(self, tool_pose):
        """
        Calcule l'orientation pour que l'outil à tool_pose regarde vers le point d'intérêt.
        Nous supposons que l'axe "avant" de l'outil est son axe Z.
        """
        # Le point à regarder a la même hauteur Z que la cible de l'outil
        self.look_at_point.z = tool_pose.position.z
        
        # Calculer le vecteur directionnel : de l'outil VERS le point à regarder
        direction_vector = np.array([
            self.look_at_point.x - tool_pose.position.x,
            self.look_at_point.y - tool_pose.position.y,
            self.look_at_point.z - tool_pose.position.z
        ])
        
        # Normaliser ce vecteur pour obtenir l'axe Z de l'outil
        z_axis = direction_vector / np.linalg.norm(direction_vector)
        
        # L'axe X de l'outil peut être calculé comme perpendiculaire à l'axe Z et à l'axe Z du monde
        world_z_axis = np.array([0, 0, 1])
        x_axis = np.cross(world_z_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # L'axe Y de l'outil est perpendiculaire aux axes X et Z pour former un repère droit
        y_axis = np.cross(z_axis, x_axis)
        
        # Créer la matrice de rotation à partir des 3 axes
        rotation_matrix = np.identity(4)
        rotation_matrix[0, 0:3] = x_axis
        rotation_matrix[1, 0:3] = y_axis
        rotation_matrix[2, 0:3] = z_axis
        
        # Convertir la matrice de rotation en quaternion
        quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
        return quaternion

    def coordinate_callback(self, msg):
        """
        Fonction appelée à chaque réception de message sur /target_coordinates.
        """
        rospy.loginfo(f"Coordonnées reçues : x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
        
        target_pose = Pose()
        target_pose.position.x = msg.x
        target_pose.position.y = msg.y
        target_pose.position.z = msg.z
        
        # Calculer l'orientation requise
        q = self.calculate_look_at_quaternion(target_pose)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        # Commander le robot
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("Point cible atteint avec la contrainte d'orientation.")
        else:
            rospy.logerr("Échec du déplacement vers le point cible.")

def main():
    controller = RobotController()
    controller.go_to_initial_pose()
    
    # rospy.spin() maintient le script en vie pour qu'il puisse écouter les topics
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass