;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::CmdJointTrajectoryEx)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'CmdJointTrajectoryEx (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::CMDJOINTTRAJECTORYEX")
  (make-package "MOTOMAN_MSGS::CMDJOINTTRAJECTORYEX"))
(unless (find-package "MOTOMAN_MSGS::CMDJOINTTRAJECTORYEXREQUEST")
  (make-package "MOTOMAN_MSGS::CMDJOINTTRAJECTORYEXREQUEST"))
(unless (find-package "MOTOMAN_MSGS::CMDJOINTTRAJECTORYEXRESPONSE")
  (make-package "MOTOMAN_MSGS::CMDJOINTTRAJECTORYEXRESPONSE"))

(in-package "ROS")



(if (not (find-package "INDUSTRIAL_MSGS"))
  (ros::roseus-add-msgs "industrial_msgs"))


(defclass motoman_msgs::CmdJointTrajectoryExRequest
  :super ros::object
  :slots (_trajectory ))

(defmethod motoman_msgs::CmdJointTrajectoryExRequest
  (:init
   (&key
    ((:trajectory __trajectory) (instance motoman_msgs::DynamicJointTrajectory :init))
    )
   (send-super :init)
   (setq _trajectory __trajectory)
   self)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; motoman_msgs/DynamicJointTrajectory _trajectory
    (send _trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; motoman_msgs/DynamicJointTrajectory _trajectory
       (send _trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; motoman_msgs/DynamicJointTrajectory _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;;
   self)
  )

(defclass motoman_msgs::CmdJointTrajectoryExResponse
  :super ros::object
  :slots (_code ))

(defmethod motoman_msgs::CmdJointTrajectoryExResponse
  (:init
   (&key
    ((:code __code) (instance industrial_msgs::ServiceReturnCode :init))
    )
   (send-super :init)
   (setq _code __code)
   self)
  (:code
   (&rest __code)
   (if (keywordp (car __code))
       (send* _code __code)
     (progn
       (if __code (setq _code (car __code)))
       _code)))
  (:serialization-length
   ()
   (+
    ;; industrial_msgs/ServiceReturnCode _code
    (send _code :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; industrial_msgs/ServiceReturnCode _code
       (send _code :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; industrial_msgs/ServiceReturnCode _code
     (send _code :deserialize buf ptr-) (incf ptr- (send _code :serialization-length))
   ;;
   self)
  )

(defclass motoman_msgs::CmdJointTrajectoryEx
  :super ros::object
  :slots ())

(setf (get motoman_msgs::CmdJointTrajectoryEx :md5sum-) "2c13063e3bd4783949860a7cb1a82ea9")
(setf (get motoman_msgs::CmdJointTrajectoryEx :datatype-) "motoman_msgs/CmdJointTrajectoryEx")
(setf (get motoman_msgs::CmdJointTrajectoryEx :request) motoman_msgs::CmdJointTrajectoryExRequest)
(setf (get motoman_msgs::CmdJointTrajectoryEx :response) motoman_msgs::CmdJointTrajectoryExResponse)

(defmethod motoman_msgs::CmdJointTrajectoryExRequest
  (:response () (instance motoman_msgs::CmdJointTrajectoryExResponse :init)))

(setf (get motoman_msgs::CmdJointTrajectoryExRequest :md5sum-) "2c13063e3bd4783949860a7cb1a82ea9")
(setf (get motoman_msgs::CmdJointTrajectoryExRequest :datatype-) "motoman_msgs/CmdJointTrajectoryExRequest")
(setf (get motoman_msgs::CmdJointTrajectoryExRequest :definition-)
      "

motoman_msgs/DynamicJointTrajectory trajectory

================================================================================
MSG: motoman_msgs/DynamicJointTrajectory
#length: true message/data length
#header: 
#sequence:
#num_groups: # of motion groups included in this message
#group[]: DynamicJointPoint from DynamicJointPoint.msg

Header header
string[] joint_names
DynamicJointPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: motoman_msgs/DynamicJointPoint
# DynamicJointPoint
#group: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    time_from_start

int16 num_groups
DynamicJointsGroup[] groups

================================================================================
MSG: motoman_msgs/DynamicJointsGroup
# DynamicJointsGroup
#group: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    time_from_start


int16 group_number
int16 num_joints
int16 valid_fields
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
industrial_msgs/ServiceReturnCode code


================================================================================
MSG: industrial_msgs/ServiceReturnCode
# Service return codes for simple requests.  All ROS-Industrial service
# replies are required to have a return code indicating success or failure
# Specific return codes for different failure should be negative.
int8 val

int8 SUCCESS = 1
int8 FAILURE = -1

")

(setf (get motoman_msgs::CmdJointTrajectoryExResponse :md5sum-) "2c13063e3bd4783949860a7cb1a82ea9")
(setf (get motoman_msgs::CmdJointTrajectoryExResponse :datatype-) "motoman_msgs/CmdJointTrajectoryExResponse")
(setf (get motoman_msgs::CmdJointTrajectoryExResponse :definition-)
      "

motoman_msgs/DynamicJointTrajectory trajectory

================================================================================
MSG: motoman_msgs/DynamicJointTrajectory
#length: true message/data length
#header: 
#sequence:
#num_groups: # of motion groups included in this message
#group[]: DynamicJointPoint from DynamicJointPoint.msg

Header header
string[] joint_names
DynamicJointPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: motoman_msgs/DynamicJointPoint
# DynamicJointPoint
#group: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    time_from_start

int16 num_groups
DynamicJointsGroup[] groups

================================================================================
MSG: motoman_msgs/DynamicJointsGroup
# DynamicJointsGroup
#group: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    time_from_start


int16 group_number
int16 num_joints
int16 valid_fields
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
industrial_msgs/ServiceReturnCode code


================================================================================
MSG: industrial_msgs/ServiceReturnCode
# Service return codes for simple requests.  All ROS-Industrial service
# replies are required to have a return code indicating success or failure
# Specific return codes for different failure should be negative.
int8 val

int8 SUCCESS = 1
int8 FAILURE = -1

")



(provide :motoman_msgs/CmdJointTrajectoryEx "2c13063e3bd4783949860a7cb1a82ea9")


