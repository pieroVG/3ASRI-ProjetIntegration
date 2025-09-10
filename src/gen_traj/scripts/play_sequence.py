#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import json
from geometry_msgs.msg import Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# --- Fonctions Utilitaires ---

def dict_to_pose(pose_dict):
    """Convertit un dictionnaire (depuis JSON) en objet geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x = pose_dict['position']['x']
    pose.position.y = pose_dict['position']['y']
    pose.position.z = pose_dict['position']['z']
    pose.orientation.x = pose_dict['orientation']['x']
    pose.orientation.y = pose_dict['orientation']['y']
    pose.orientation.z = pose_dict['orientation']['z']
    pose.orientation.w = pose_dict['orientation']['w']
    return pose

# (On réutilise la même fonction de visualisation que dans la réponse précédente)
def create_marker_array(sequence_de_poses, current_target_index=-1):
    """Crée un MarkerArray pour afficher une liste de poses."""
    marker_array = MarkerArray()
    for i, pose in enumerate(sequence_de_poses):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory_points"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = Vector3(0.05, 0.05, 0.05)
        if i == current_target_index:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) # Vert pour la cible
            marker.scale = Vector3(0.08, 0.08, 0.08)
        else:
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0) # Bleu pour les autres
        marker.lifetime = rospy.Duration()
        marker_array.markers.append(marker)
    return marker_array

def main():
    # --- Initialisations ---
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sequence_player', anonymous=True)
    move_group = moveit_commander.MoveGroupCommander("hc10_arm")
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    rospy.sleep(0.5) # Laisser le temps au publisher de s'établir

    # --- Chargement et préparation de la séquence ---
    file_path = "hc10_sequence.json"
    sequence_of_poses = []
    
    try:
        with open(file_path, 'r') as f:
            waypoints_dict = json.load(f)
            # Convertir chaque dictionnaire en objet Pose
            for wp_dict in waypoints_dict:
                sequence_of_poses.append(dict_to_pose(wp_dict))
        rospy.loginfo(f"{len(sequence_of_poses)} points chargés avec succès depuis '{file_path}'.")
    except IOError:
        rospy.logerr(f"Erreur : Le fichier '{file_path}' est introuvable. Avez-vous enregistré une séquence d'abord ?")
        return
    except (json.JSONDecodeError, KeyError) as e:
        rospy.logerr(f"Erreur de lecture du fichier JSON : {e}")
        return

    if not sequence_of_poses:
        rospy.logwarn("Le fichier de séquence est vide. Rien à faire.")
        return

    # --- Visualisation et Confirmation ---
    rospy.loginfo("Affichage de la trajectoire complète dans RViz...")
    marker_array = create_marker_array(sequence_of_poses)
    marker_pub.publish(marker_array)

    input("La trajectoire est affichée. Appuyez sur 'Entrée' pour commencer l'exécution...")

    # --- Exécution de la séquence ---
    rospy.loginfo("Lancement de la séquence de mouvement.")
    
    for i, pose_cible in enumerate(sequence_of_poses):
        rospy.loginfo(f"Déplacement vers le point {i+1}/{len(sequence_of_poses)}")
        
        # Mettre à jour les marqueurs pour surligner le point actuel
        updated_marker_array = create_marker_array(sequence_of_poses, current_target_index=i)
        marker_pub.publish(updated_marker_array)
        
        move_group.set_pose_target(pose_cible)
        plan_success = move_group.go(wait=True)
        
        if not plan_success:
            rospy.logerr(f"Échec de la planification ou de l'exécution pour le point {i+1}. Arrêt de la séquence.")
            break

        rospy.sleep(0.5) # Petite pause

    # Nettoyage final des marqueurs (tout en bleu)
    final_marker_array = create_marker_array(sequence_of_poses)
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