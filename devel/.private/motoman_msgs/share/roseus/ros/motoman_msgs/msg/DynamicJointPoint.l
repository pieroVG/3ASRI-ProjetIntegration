;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::DynamicJointPoint)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'DynamicJointPoint (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::DYNAMICJOINTPOINT")
  (make-package "MOTOMAN_MSGS::DYNAMICJOINTPOINT"))

(in-package "ROS")
;;//! \htmlinclude DynamicJointPoint.msg.html


(defclass motoman_msgs::DynamicJointPoint
  :super ros::object
  :slots (_num_groups _groups ))

(defmethod motoman_msgs::DynamicJointPoint
  (:init
   (&key
    ((:num_groups __num_groups) 0)
    ((:groups __groups) ())
    )
   (send-super :init)
   (setq _num_groups (round __num_groups))
   (setq _groups __groups)
   self)
  (:num_groups
   (&optional __num_groups)
   (if __num_groups (setq _num_groups __num_groups)) _num_groups)
  (:groups
   (&rest __groups)
   (if (keywordp (car __groups))
       (send* _groups __groups)
     (progn
       (if __groups (setq _groups (car __groups)))
       _groups)))
  (:serialization-length
   ()
   (+
    ;; int16 _num_groups
    2
    ;; motoman_msgs/DynamicJointsGroup[] _groups
    (apply #'+ (send-all _groups :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _num_groups
       (write-word _num_groups s)
     ;; motoman_msgs/DynamicJointsGroup[] _groups
     (write-long (length _groups) s)
     (dolist (elem _groups)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _num_groups
     (setq _num_groups (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; motoman_msgs/DynamicJointsGroup[] _groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _groups (let (r) (dotimes (i n) (push (instance motoman_msgs::DynamicJointsGroup :init) r)) r))
     (dolist (elem- _groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get motoman_msgs::DynamicJointPoint :md5sum-) "f91ca86c2821b55c8430ab0088bfe5df")
(setf (get motoman_msgs::DynamicJointPoint :datatype-) "motoman_msgs/DynamicJointPoint")
(setf (get motoman_msgs::DynamicJointPoint :definition-)
      "# DynamicJointPoint
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

")



(provide :motoman_msgs/DynamicJointPoint "f91ca86c2821b55c8430ab0088bfe5df")


