#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class Trajectoire:
	def __init__(self):
		self.h_table = 0
		self.h_objet = 30e-2  # Hauteur de l'objet
		self.largeur_max_obj = 20e-2  # largeur de l'objet

		self.A = self.largeur_max_obj  # Rayon du cercle autour de l'objet
		self.dtheta = 0.785398  # pas angulaire en radian (environ 45 deg)
		self.portee_max = 5  # Portée max du capteur
		self.dist_secu = 15e-2  # distance de sécurité
		self.dt = 0.1  # pas de temps

		self.teta = 0
		self.teta_cible = 0
		self.c_x = 0.5
		self.c_y = 0.5

		self.nb_scan = 0

		# position initiale
		self.x = self.c_x
		self.y = self.c_y
		self.z = 1

		self.critere_arret = True
		self.step = 0

		# Initialisation ROS
		rospy.init_node("traj_generator", anonymous=True)
		self.pub = rospy.Publisher("/target_coordinates", Point, queue_size=10)
		rospy.Subscriber("/movement_status", Bool, self.robot_state)

	def publish_target(self, point: Point):
		rospy.loginfo(f"Publication du point cible: x={point.x}, y={point.y}, z={point.z}")
		self.pub.publish(point)

	def get_arret(self, nb_scan):
		return nb_scan < 4

	def robot_state(self, msg: Bool):
		if msg.data and self.step < 8:
			x = self.A * math.cos(self.teta + self.step * self.dtheta) + self.c_x
			y = self.A * math.sin(self.teta + self.step * self.dtheta) + self.c_y
			z = self.h_objet / 2 + self.dist_secu

			target = Point(x, y, z)
			self.publish_target(target)

			self.step += 1
		else:
			return

	def run(self):
		rospy.spin()


def main():
	objet = Trajectoire()
	objet.run()


if __name__ == "__main__":
	main()
