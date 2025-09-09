;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::DynamicJointTrajectoryFeedback)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'DynamicJointTrajectoryFeedback (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::DYNAMICJOINTTRAJECTORYFEEDBACK")
  (make-package "MOTOMAN_MSGS::DYNAMICJOINTTRAJECTORYFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude DynamicJointTrajectoryFeedback.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass motoman_msgs::DynamicJointTrajectoryFeedback
  :super ros::object
  :slots (_header _num_groups _joint_feedbacks ))

(defmethod motoman_msgs::DynamicJointTrajectoryFeedback
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_groups __num_groups) 0)
    ((:joint_feedbacks __joint_feedbacks) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_groups (round __num_groups))
   (setq _joint_feedbacks __joint_feedbacks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_groups
   (&optional __num_groups)
   (if __num_groups (setq _num_groups __num_groups)) _num_groups)
  (:joint_feedbacks
   (&rest __joint_feedbacks)
   (if (keywordp (car __joint_feedbacks))
       (send* _joint_feedbacks __joint_feedbacks)
     (progn
       (if __joint_feedbacks (setq _joint_feedbacks (car __joint_feedbacks)))
       _joint_feedbacks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _num_groups
    2
    ;; motoman_msgs/DynamicJointState[] _joint_feedbacks
    (apply #'+ (send-all _joint_feedbacks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _num_groups
       (write-word _num_groups s)
     ;; motoman_msgs/DynamicJointState[] _joint_feedbacks
     (write-long (length _joint_feedbacks) s)
     (dolist (elem _joint_feedbacks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _num_groups
     (setq _num_groups (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; motoman_msgs/DynamicJointState[] _joint_feedbacks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_feedbacks (let (r) (dotimes (i n) (push (instance motoman_msgs::DynamicJointState :init) r)) r))
     (dolist (elem- _joint_feedbacks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get motoman_msgs::DynamicJointTrajectoryFeedback :md5sum-) "84d3bbf7103790ff0a8946017b895a1a")
(setf (get motoman_msgs::DynamicJointTrajectoryFeedback :datatype-) "motoman_msgs/DynamicJointTrajectoryFeedback")
(setf (get motoman_msgs::DynamicJointTrajectoryFeedback :definition-)
      "Header header
int16 num_groups
DynamicJointState[] joint_feedbacks

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
MSG: motoman_msgs/DynamicJointState
#group[]: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    position_desired[]
#    position_errors[]
#    velocity_desired[]
#    velocity_errors[]
#    effort_desired[]
#    effort_error[]

int16 group_number
int16 num_joints
int16 valid_fields
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
float64[] positions_desired
float64[] positions_errors
float64[] velocities_desired
float64[] velocities_errors
float64[] accelerations_desired
float64[] accelerations_errors
float64[] effort_errors
float64[] effort_desired

")



(provide :motoman_msgs/DynamicJointTrajectoryFeedback "84d3bbf7103790ff0a8946017b895a1a")


