#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations
import math

def main():
    # --- Définissez votre cible complète ici en utilisant des unités intuitives ---
    
    # 1. Position (en mètres)
    pos_x, pos_y, pos_z = 0.3, 0.0, 1.0
    
    # 2. Orientation avec Roll, Pitch, Yaw (en degrés)
    roll_deg, pitch_deg, yaw_deg = 0, 0, 0

    # --- Initialisation et Conversion (généralement pas besoin de modifier) ---
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('goto_pose_with_rpy', anonymous=True)
    move_group = moveit_commander.MoveGroupCommander("hc10_arm")
    
    # a. Convertir les degrés en radians
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    
    # b. Convertir les angles d'Euler (RPY) en quaternion
    quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad, axes='sxyz')
    
    # c. Créer la cible de pose
    target_pose = Pose()
    target_pose.position.x = pos_x
    target_pose.position.y = pos_y
    target_pose.position.z = pos_z
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]

    # --- Exécution ---
    move_group.set_pose_target(target_pose)
    
    rospy.loginfo("Déplacement vers la pose définie par RPY...")
    success = move_group.go(wait=True)
    
    if success:
        rospy.loginfo("Pose atteinte.")
    else:
        rospy.logerr("Échec du mouvement.")
        
    move_group.stop()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass