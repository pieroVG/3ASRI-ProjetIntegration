; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-msg)


;//! \htmlinclude DynamicJointTrajectoryFeedback.msg.html

(cl:defclass <DynamicJointTrajectoryFeedback> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_groups
    :reader num_groups
    :initarg :num_groups
    :type cl:fixnum
    :initform 0)
   (joint_feedbacks
    :reader joint_feedbacks
    :initarg :joint_feedbacks
    :type (cl:vector motoman_msgs-msg:DynamicJointState)
   :initform (cl:make-array 0 :element-type 'motoman_msgs-msg:DynamicJointState :initial-element (cl:make-instance 'motoman_msgs-msg:DynamicJointState))))
)

(cl:defclass DynamicJointTrajectoryFeedback (<DynamicJointTrajectoryFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicJointTrajectoryFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicJointTrajectoryFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-msg:<DynamicJointTrajectoryFeedback> is deprecated: use motoman_msgs-msg:DynamicJointTrajectoryFeedback instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DynamicJointTrajectoryFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:header-val is deprecated.  Use motoman_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_groups-val :lambda-list '(m))
(cl:defmethod num_groups-val ((m <DynamicJointTrajectoryFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:num_groups-val is deprecated.  Use motoman_msgs-msg:num_groups instead.")
  (num_groups m))

(cl:ensure-generic-function 'joint_feedbacks-val :lambda-list '(m))
(cl:defmethod joint_feedbacks-val ((m <DynamicJointTrajectoryFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:joint_feedbacks-val is deprecated.  Use motoman_msgs-msg:joint_feedbacks instead.")
  (joint_feedbacks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicJointTrajectoryFeedback>) ostream)
  "Serializes a message object of type '<DynamicJointTrajectoryFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_groups)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_feedbacks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joint_feedbacks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicJointTrajectoryFeedback>) istream)
  "Deserializes a message object of type '<DynamicJointTrajectoryFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_groups) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_feedbacks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_feedbacks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'motoman_msgs-msg:DynamicJointState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicJointTrajectoryFeedback>)))
  "Returns string type for a message object of type '<DynamicJointTrajectoryFeedback>"
  "motoman_msgs/DynamicJointTrajectoryFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicJointTrajectoryFeedback)))
  "Returns string type for a message object of type 'DynamicJointTrajectoryFeedback"
  "motoman_msgs/DynamicJointTrajectoryFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicJointTrajectoryFeedback>)))
  "Returns md5sum for a message object of type '<DynamicJointTrajectoryFeedback>"
  "84d3bbf7103790ff0a8946017b895a1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicJointTrajectoryFeedback)))
  "Returns md5sum for a message object of type 'DynamicJointTrajectoryFeedback"
  "84d3bbf7103790ff0a8946017b895a1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicJointTrajectoryFeedback>)))
  "Returns full string definition for message of type '<DynamicJointTrajectoryFeedback>"
  (cl:format cl:nil "Header header~%int16 num_groups~%DynamicJointState[] joint_feedbacks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motoman_msgs/DynamicJointState~%#group[]: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    position_desired[]~%#    position_errors[]~%#    velocity_desired[]~%#    velocity_errors[]~%#    effort_desired[]~%#    effort_error[]~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%float64[] positions_desired~%float64[] positions_errors~%float64[] velocities_desired~%float64[] velocities_errors~%float64[] accelerations_desired~%float64[] accelerations_errors~%float64[] effort_errors~%float64[] effort_desired~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicJointTrajectoryFeedback)))
  "Returns full string definition for message of type 'DynamicJointTrajectoryFeedback"
  (cl:format cl:nil "Header header~%int16 num_groups~%DynamicJointState[] joint_feedbacks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motoman_msgs/DynamicJointState~%#group[]: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    position_desired[]~%#    position_errors[]~%#    velocity_desired[]~%#    velocity_errors[]~%#    effort_desired[]~%#    effort_error[]~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%float64[] positions_desired~%float64[] positions_errors~%float64[] velocities_desired~%float64[] velocities_errors~%float64[] accelerations_desired~%float64[] accelerations_errors~%float64[] effort_errors~%float64[] effort_desired~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicJointTrajectoryFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_feedbacks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicJointTrajectoryFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicJointTrajectoryFeedback
    (cl:cons ':header (header msg))
    (cl:cons ':num_groups (num_groups msg))
    (cl:cons ':joint_feedbacks (joint_feedbacks msg))
))
