# 3ASRI-ProjetIntegration
http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

### Instruction
```
git clone https://github.com/pieroVG/3ASRI-ProjetIntegration.git
cd 3ASRI-ProjetIntegration
. /opt/ros/noetic/setup.bash
catkin build
. devel/setup.bash
```

Test pour vérifier si c'est bien installé :
```
roslaunch hc10_moveit_config demo.launch 
```

Refaire les setup.bash après avoir installer hc10_moveit_config si erreur
```
source devel/setup.bash
```

## Tutoriel pour la génération de trajectoire
https://moveit.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

Lancer le modèle du robot sous gazebo :
```
roslaunch hc10_moveit_config demo_gazebo.launch 
```

## Tutoriel d'intégration MoveIt dans Gazebo
https://moveit.github.io/moveit_tutorials/doc/gazebo_simulation/gazebo_simulation.html

## Tutoriel d'intégration d'un kinect dans l'environnement
https://moveit.github.io/moveit_tutorials/doc/mesh_filter/mesh_filter_tutorial.html#how-to-add-sensor-to-arm-in-simulation

## Tutoriel d'une pipeline Gazebo <-> ROS <-> MoveIt
https://moveit.github.io/moveit_tutorials/doc/perception_pipeline/perception_pipeline_tutorial.html
