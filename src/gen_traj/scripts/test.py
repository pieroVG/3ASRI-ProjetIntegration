#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# --- Fonction utilitaire pour la visualisation ---
# Nous la créons pour éviter de répéter le code
def create_marker_array(sequence_de_poses, current_target_index=-1):
    """
    Crée un MarkerArray pour afficher une liste de poses.
    Met en évidence le point cible actuel.
    """
    marker_array = MarkerArray()
    
    for i, pose in enumerate(sequence_de_poses):
        marker = Marker()
        # Le header est crucial : il définit le repère et l'horodatage
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        
        # Le namespace et l'ID permettent à RViz de gérer chaque marqueur
        marker.ns = "target_points"
        marker.id = i
        
        # Définir le type de marqueur (une sphère)
        marker.type = Marker.SPHERE
        
        # Action : ADD signifie ajouter ou modifier le marqueur
        marker.action = Marker.ADD
        
        # Définir la pose, l'échelle et la couleur du marqueur
        marker.pose = pose
        marker.scale = Vector3(0.05, 0.05, 0.05) # Petite sphère
        
        if i == current_target_index:
            # Le point cible actuel sera VERT et plus gros
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) # Vert
            marker.scale = Vector3(0.08, 0.08, 0.08) # Grosse sphère
        else:
            # Les autres points seront BLEUS
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0) # Bleu
        
        # Durée de vie : 0 signifie "infini" (jusqu'à être modifié ou supprimé)
        marker.lifetime = rospy.Duration()
        
        marker_array.markers.append(marker)
        
    return marker_array

def main():
    # --- Initialisations (comme avant) ---
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('hc10_sequence_visualizer_native', anonymous=True)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("hc10_arm")

    # --- Initialisation du Publisher de Marqueurs ---
    # On crée un publisher qui enverra nos marqueurs sur un topic
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    
    # Petite pause pour s'assurer que le publisher est bien connecté à RViz
    rospy.sleep(0.5)

    # --- Définition de votre séquence de points (comme avant) ---
    pose1 = Pose()
    pose1.orientation.w = 1.0
    pose1.position.x = 0.6
    pose1.position.y = 0.3
    pose1.position.z = 0.7

    pose2 = Pose()
    pose2.orientation.w = 1.0
    pose2.position.x = 0.6
    pose2.position.y = -0.3
    pose2.position.z = 0.7

    pose3 = Pose()
    pose3.orientation.w = 1.0
    pose3.position.x = 0.8
    pose3.position.y = 0.0
    pose3.position.z = 0.5
    
    sequence_de_poses = [pose1, pose2, pose3]

    # --- Étape de Visualisation ---
    rospy.loginfo("Affichage des points cibles dans RViz...")
    marker_array = create_marker_array(sequence_de_poses)
    marker_pub.publish(marker_array)

    # --- Étape de Confirmation (comme avant) ---
    rospy.loginfo("Points affichés. Vous pouvez inspecter la trajectoire dans RViz.")
    input("Appuyez sur 'Entrée' dans ce terminal pour commencer le mouvement du robot...")

    # --- Étape d'Exécution ---
    rospy.loginfo("Lancement de la séquence de mouvement.")
    
    for i, pose_cible in enumerate(sequence_de_poses):
        rospy.loginfo(f"Mouvement vers le point {i+1}/{len(sequence_de_poses)}")
        
        # Met à jour les marqueurs pour surligner le point actuel
        updated_marker_array = create_marker_array(sequence_de_poses, current_target_index=i)
        marker_pub.publish(updated_marker_array)

        move_group.set_pose_target(pose_cible)
        plan_success = move_group.go(wait=True)
        
        if not plan_success:
            rospy.logerr(f"Échec du mouvement vers le point {i+1}")
            break

        rospy.sleep(1)

    # Nettoyage final : on republie les marqueurs sans point en surbrillance
    final_marker_array = create_marker_array(sequence_de_poses)
    marker_pub.publish(final_marker_array)
    
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo("Séquence terminée.")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass