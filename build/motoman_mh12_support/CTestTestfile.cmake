# CMake generated Testfile for 
# Source directory: /home/etudiant/catkin_ws/src/motoman/motoman_mh12_support
# Build directory: /home/etudiant/catkin_ws/build/motoman_mh12_support
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_motoman_mh12_support_roslaunch-check_test_roslaunch_test_mh12.xml "/home/etudiant/catkin_ws/build/motoman_mh12_support/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/etudiant/catkin_ws/build/motoman_mh12_support/test_results/motoman_mh12_support/roslaunch-check_test_roslaunch_test_mh12.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/etudiant/catkin_ws/build/motoman_mh12_support/test_results/motoman_mh12_support" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/etudiant/catkin_ws/build/motoman_mh12_support/test_results/motoman_mh12_support/roslaunch-check_test_roslaunch_test_mh12.xml.xml\" \"/home/etudiant/catkin_ws/src/motoman/motoman_mh12_support/test/roslaunch_test_mh12.xml\" ")
set_tests_properties(_ctest_motoman_mh12_support_roslaunch-check_test_roslaunch_test_mh12.xml PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/etudiant/catkin_ws/src/motoman/motoman_mh12_support/CMakeLists.txt;11;roslaunch_add_file_check;/home/etudiant/catkin_ws/src/motoman/motoman_mh12_support/CMakeLists.txt;0;")
subdirs("gtest")
