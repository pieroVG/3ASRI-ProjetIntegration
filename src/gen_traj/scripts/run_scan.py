#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations

def main():
    # --- Initialisations ---
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('goto_specific_pose_node', anonymous=True)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("hc10_arm")
    
    # Paramètres pour la planification
    move_group.set_planning_time(5) # Donner 5 secondes pour trouver un plan
    move_group.set_num_planning_attempts(5) # Essayer plusieurs fois si nécessaire

    # --- 1. Définir la Pose Cible ---
    rospy.loginfo("Définition de la pose cible...")
    target_pose = Pose()

    # --- a) Définir l'Orientation "regardant vers le bas" ---
    # Rotation de 180 degrés (pi radians) autour de l'axe Y.
    orientation_q = tf.transformations.quaternion_from_euler(0, 3.14159, 0)
    target_pose.orientation.x = orientation_q[0]
    target_pose.orientation.y = orientation_q[1]
    target_pose.orientation.z = orientation_q[2]
    target_pose.orientation.w = orientation_q[3]

    # --- b) Définir la Position ---
    # Hauteur de 1 mètre par rapport à la base du robot
    target_pose.position.z = 0.5
    # Distance horizontale de 30 cm par rapport à la base.
    # On se place sur l'axe X pour plus de simplicité.
    target_pose.position.x = 0.3
    target_pose.position.y = 0.0

    rospy.loginfo(f"La cible est définie à :\n"
                  f"  Position (x,y,z) = ({target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z})\n"
                  f"  Orientation (x,y,z,w) = ({target_pose.orientation.x:.2f}, {target_pose.orientation.y:.2f}, {target_pose.orientation.z:.2f}, {target_pose.orientation.w:.2f})")

    # --- 2. Planifier et Exécuter le Mouvement ---
    input("\nAppuyez sur 'Entrée' pour déplacer le robot vers cette pose.")

    move_group.set_pose_target(target_pose)
    
    rospy.loginfo("Planification et exécution du mouvement...")
    success = move_group.go(wait=True)

    if success:
        rospy.loginfo("Le robot a atteint la pose cible avec succès.")
    else:
        rospy.logerr("Échec du déplacement vers la pose cible. La pose est peut-être hors de portée ou en collision.")

    # Nettoyage
    move_group.stop()
    move_group.clear_pose_targets()
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass