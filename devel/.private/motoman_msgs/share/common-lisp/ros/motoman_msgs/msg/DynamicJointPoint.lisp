; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-msg)


;//! \htmlinclude DynamicJointPoint.msg.html

(cl:defclass <DynamicJointPoint> (roslisp-msg-protocol:ros-message)
  ((num_groups
    :reader num_groups
    :initarg :num_groups
    :type cl:fixnum
    :initform 0)
   (groups
    :reader groups
    :initarg :groups
    :type (cl:vector motoman_msgs-msg:DynamicJointsGroup)
   :initform (cl:make-array 0 :element-type 'motoman_msgs-msg:DynamicJointsGroup :initial-element (cl:make-instance 'motoman_msgs-msg:DynamicJointsGroup))))
)

(cl:defclass DynamicJointPoint (<DynamicJointPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicJointPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicJointPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-msg:<DynamicJointPoint> is deprecated: use motoman_msgs-msg:DynamicJointPoint instead.")))

(cl:ensure-generic-function 'num_groups-val :lambda-list '(m))
(cl:defmethod num_groups-val ((m <DynamicJointPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:num_groups-val is deprecated.  Use motoman_msgs-msg:num_groups instead.")
  (num_groups m))

(cl:ensure-generic-function 'groups-val :lambda-list '(m))
(cl:defmethod groups-val ((m <DynamicJointPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:groups-val is deprecated.  Use motoman_msgs-msg:groups instead.")
  (groups m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicJointPoint>) ostream)
  "Serializes a message object of type '<DynamicJointPoint>"
  (cl:let* ((signed (cl:slot-value msg 'num_groups)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'groups))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'groups))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicJointPoint>) istream)
  "Deserializes a message object of type '<DynamicJointPoint>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_groups) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'groups) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'groups)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'motoman_msgs-msg:DynamicJointsGroup))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicJointPoint>)))
  "Returns string type for a message object of type '<DynamicJointPoint>"
  "motoman_msgs/DynamicJointPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicJointPoint)))
  "Returns string type for a message object of type 'DynamicJointPoint"
  "motoman_msgs/DynamicJointPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicJointPoint>)))
  "Returns md5sum for a message object of type '<DynamicJointPoint>"
  "f91ca86c2821b55c8430ab0088bfe5df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicJointPoint)))
  "Returns md5sum for a message object of type 'DynamicJointPoint"
  "f91ca86c2821b55c8430ab0088bfe5df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicJointPoint>)))
  "Returns full string definition for message of type '<DynamicJointPoint>"
  (cl:format cl:nil "# DynamicJointPoint~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%int16 num_groups~%DynamicJointsGroup[] groups~%~%================================================================================~%MSG: motoman_msgs/DynamicJointsGroup~%# DynamicJointsGroup~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicJointPoint)))
  "Returns full string definition for message of type 'DynamicJointPoint"
  (cl:format cl:nil "# DynamicJointPoint~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%int16 num_groups~%DynamicJointsGroup[] groups~%~%================================================================================~%MSG: motoman_msgs/DynamicJointsGroup~%# DynamicJointsGroup~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicJointPoint>))
  (cl:+ 0
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'groups) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicJointPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicJointPoint
    (cl:cons ':num_groups (num_groups msg))
    (cl:cons ':groups (groups msg))
))
