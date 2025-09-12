#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# . devel/setup.bash
# chmod +x src/gen_traj/scripts/cycle_listener.py
# rosrun gen_traj cycle_listener.py
# 
# Topics listeners : 
#   /look_at_point_update : geometry_msgs/Point
#   /target_coordinates : geometry_msgs/Point 
#
# Topic publisher :
#   /movement_status : std_msgs/String ("SUCCESS" or "FAILURE")

import sys
import rospy
import moveit_commander
import tf.transformations
import math
import numpy as np
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool # Importation pour le message de statut

class RobotController:
    def __init__(self):
        """Initialise le contrôleur du robot, MoveIt, les subscribers et le publisher."""
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_look_at_controller', anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("hc10_arm") 
        
        self.look_at_point = Point(x=0.5, y=0.0, z=0.0)
        rospy.loginfo(f"Point d'intérêt 'look_at' initialisé par défaut à x={self.look_at_point.x}, y={self.look_at_point.y}")

        # --- Subscribers ---
        self.target_sub = rospy.Subscriber("/target_coordinates", Point, self.coordinate_callback)
        self.look_at_sub = rospy.Subscriber("/look_at_point_update", Point, self.look_at_point_callback)
        
        # Publisher pour le statut du mouvement
        self.status_publisher = rospy.Publisher("/movement_status", Bool, queue_size=10)
        
        rospy.loginfo("Contrôleur initialisé. En attente de coordonnées sur les topics...")

    def look_at_point_callback(self, msg):
        rospy.loginfo(f"Mise à jour du point d'intérêt 'look_at' -> x={msg.x:.2f}, y={msg.y:.2f}")
        self.look_at_point.x = msg.x
        self.look_at_point.y = msg.y

    def go_to_initial_pose(self):
        rospy.loginfo("Déplacement vers la position initiale...")
        initial_pose = Pose()
        initial_pose.position.x = 0.5; initial_pose.position.y = 0.0; initial_pose.position.z = 1.0
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w = q
        self.move_group.set_pose_target(initial_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop(); self.move_group.clear_pose_targets()
        if success: rospy.loginfo("Position initiale atteinte.")
        else: rospy.signal_shutdown("Impossible d'atteindre la pose initiale")

    def calculate_look_at_quaternion(self, tool_pose):
        direction_vector = np.array([ self.look_at_point.x - tool_pose.position.x, self.look_at_point.y - tool_pose.position.y, 0.0])
        if np.linalg.norm(direction_vector) < 1e-5: return tf.transformations.quaternion_from_euler(0, math.pi / 2, 0)
        angle_yaw = math.atan2(direction_vector[1], direction_vector[0])
        roll, pitch, yaw = 0.0, math.pi / 2, angle_yaw + math.pi
        return tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')

    def coordinate_callback(self, msg):
        """
        Calcule et exécute un chemin cartésien, puis publie le statut.
        """
        rospy.loginfo(f"Coordonnées reçues : x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
        
        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = msg.x, msg.y, msg.z
        q = self.calculate_look_at_quaternion(target_pose)
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q

        waypoints = [target_pose]
        
        rospy.loginfo("Calcul de la trajectoire cartésienne (ligne droite)...")
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.1, 0.0)

        if fraction > 0.99: # Si le plan est quasi-complet
            rospy.loginfo("Trajectoire calculée. Exécution...")
            self.move_group.execute(plan, wait=True)
            rospy.loginfo("Mouvement terminé avec succès.")
            self.status_publisher.publish(True) # Publication du succès
        else:
            rospy.logerr(f"Échec du calcul de la trajectoire. Réussite à {fraction * 100:.2f}%. Mouvement annulé.")
            self.status_publisher.publish(False) # Publication de l'échec

        self.move_group.stop()

def main():
    controller = RobotController()
    controller.go_to_initial_pose()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
