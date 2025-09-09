
(cl:in-package :asdf)

(defsystem "motoman_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DynamicJointPoint" :depends-on ("_package_DynamicJointPoint"))
    (:file "_package_DynamicJointPoint" :depends-on ("_package"))
    (:file "DynamicJointState" :depends-on ("_package_DynamicJointState"))
    (:file "_package_DynamicJointState" :depends-on ("_package"))
    (:file "DynamicJointTrajectory" :depends-on ("_package_DynamicJointTrajectory"))
    (:file "_package_DynamicJointTrajectory" :depends-on ("_package"))
    (:file "DynamicJointTrajectoryFeedback" :depends-on ("_package_DynamicJointTrajectoryFeedback"))
    (:file "_package_DynamicJointTrajectoryFeedback" :depends-on ("_package"))
    (:file "DynamicJointsGroup" :depends-on ("_package_DynamicJointsGroup"))
    (:file "_package_DynamicJointsGroup" :depends-on ("_package"))
  ))