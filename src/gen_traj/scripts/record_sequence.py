#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import json
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerFeedback

def pose_to_dict(pose):
    """Convertit un message geometry_msgs/Pose en dictionnaire."""
    return {
        'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
        'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y, 'z': pose.orientation.z, 'w': pose.orientation.w}
    }

class SequenceRecorder:
    def __init__(self, file_path="hc10_sequence.json"):
        # --- Initialisations ---
        self.move_group = moveit_commander.MoveGroupCommander("hc10_arm")
        self.file_path = file_path
        self.waypoints = []
        self.latest_marker_pose = None

        # --- Charger les points existants ---
        try:
            with open(self.file_path, 'r') as f:
                self.waypoints = json.load(f)
            rospy.loginfo(f"{len(self.waypoints)} points chargés depuis '{self.file_path}'.")
        except (IOError, json.JSONDecodeError):
            rospy.loginfo("Démarrage d'une nouvelle séquence.")
        
        # --- Subscriber pour le marqueur interactif ---
        # REMPLACEZ CECI par le nom de topic que vous avez trouvé !
        marker_topic = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback"
        rospy.Subscriber(marker_topic, InteractiveMarkerFeedback, self.marker_feedback_callback)
        rospy.loginfo(f"Écoute du marqueur interactif sur le topic : {marker_topic}")

    def marker_feedback_callback(self, feedback_msg):
        """Cette fonction est appelée à chaque interaction avec le marqueur dans RViz."""
        # On ne met à jour la pose que lorsque l'utilisateur a fini de bouger la souris
        if feedback_msg.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.latest_marker_pose = feedback_msg.pose
            # rospy.loginfo_throttle(1.0, "Nouvelle pose cible reçue du marqueur.") # Décommenter pour déboguer

    def run(self):
        """Boucle principale pour l'enregistrement."""
        print("\n" + "="*50)
        print("      OUTIL D'ENREGISTREMENT DE SÉQUENCE DE POINTS (v2)")
        print("="*50)
        print("Instructions :")
        print("  1. Dans RViz, déplacez l'effecteur interactif (orange).")
        print("  2. Revenez à CE terminal et appuyez sur 'Entrée' pour sauvegarder.")
        print("  3. Le robot se déplacera vers le point pour confirmation.")
        print("  4. Tapez 'q' puis 'Entrée' pour quitter.")
        print("="*50 + "\n")

        while not rospy.is_shutdown():
            user_input = input("Appuyez sur Entrée pour sauvegarder le point, ou 'q' pour quitter > ")
            
            if user_input.lower() == 'q':
                break

            if self.latest_marker_pose is None:
                rospy.logwarn("Aucune pose n'a été reçue du marqueur. Avez-vous bien bougé le marqueur dans RViz ?")
                continue
            
            self.waypoints.append(pose_to_dict(self.latest_marker_pose))
            rospy.loginfo(f"Point {len(self.waypoints)} sauvegardé !")
            
            with open(self.file_path, 'w') as f:
                json.dump(self.waypoints, f, indent=4)
            
            rospy.loginfo("Déplacement vers le point enregistré pour confirmation...")
            self.move_group.set_pose_target(self.latest_marker_pose)
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()

        rospy.loginfo(f"Séquence finale de {len(self.waypoints)} points sauvegardée dans '{self.file_path}'.")

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sequence_recorder_v2', anonymous=True)
    
    recorder = SequenceRecorder()
    recorder.run()
    
    moveit_commander.roscpp_shutdown()