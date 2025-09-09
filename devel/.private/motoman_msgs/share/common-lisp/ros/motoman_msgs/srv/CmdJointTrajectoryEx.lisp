; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude CmdJointTrajectoryEx-request.msg.html

(cl:defclass <CmdJointTrajectoryEx-request> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type motoman_msgs-msg:DynamicJointTrajectory
    :initform (cl:make-instance 'motoman_msgs-msg:DynamicJointTrajectory)))
)

(cl:defclass CmdJointTrajectoryEx-request (<CmdJointTrajectoryEx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CmdJointTrajectoryEx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CmdJointTrajectoryEx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<CmdJointTrajectoryEx-request> is deprecated: use motoman_msgs-srv:CmdJointTrajectoryEx-request instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <CmdJointTrajectoryEx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:trajectory-val is deprecated.  Use motoman_msgs-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CmdJointTrajectoryEx-request>) ostream)
  "Serializes a message object of type '<CmdJointTrajectoryEx-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CmdJointTrajectoryEx-request>) istream)
  "Deserializes a message object of type '<CmdJointTrajectoryEx-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CmdJointTrajectoryEx-request>)))
  "Returns string type for a service object of type '<CmdJointTrajectoryEx-request>"
  "motoman_msgs/CmdJointTrajectoryExRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdJointTrajectoryEx-request)))
  "Returns string type for a service object of type 'CmdJointTrajectoryEx-request"
  "motoman_msgs/CmdJointTrajectoryExRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CmdJointTrajectoryEx-request>)))
  "Returns md5sum for a message object of type '<CmdJointTrajectoryEx-request>"
  "2c13063e3bd4783949860a7cb1a82ea9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CmdJointTrajectoryEx-request)))
  "Returns md5sum for a message object of type 'CmdJointTrajectoryEx-request"
  "2c13063e3bd4783949860a7cb1a82ea9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CmdJointTrajectoryEx-request>)))
  "Returns full string definition for message of type '<CmdJointTrajectoryEx-request>"
  (cl:format cl:nil "~%~%motoman_msgs/DynamicJointTrajectory trajectory~%~%================================================================================~%MSG: motoman_msgs/DynamicJointTrajectory~%#length: true message/data length~%#header: ~%#sequence:~%#num_groups: # of motion groups included in this message~%#group[]: DynamicJointPoint from DynamicJointPoint.msg~%~%Header header~%string[] joint_names~%DynamicJointPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motoman_msgs/DynamicJointPoint~%# DynamicJointPoint~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%int16 num_groups~%DynamicJointsGroup[] groups~%~%================================================================================~%MSG: motoman_msgs/DynamicJointsGroup~%# DynamicJointsGroup~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CmdJointTrajectoryEx-request)))
  "Returns full string definition for message of type 'CmdJointTrajectoryEx-request"
  (cl:format cl:nil "~%~%motoman_msgs/DynamicJointTrajectory trajectory~%~%================================================================================~%MSG: motoman_msgs/DynamicJointTrajectory~%#length: true message/data length~%#header: ~%#sequence:~%#num_groups: # of motion groups included in this message~%#group[]: DynamicJointPoint from DynamicJointPoint.msg~%~%Header header~%string[] joint_names~%DynamicJointPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motoman_msgs/DynamicJointPoint~%# DynamicJointPoint~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%int16 num_groups~%DynamicJointsGroup[] groups~%~%================================================================================~%MSG: motoman_msgs/DynamicJointsGroup~%# DynamicJointsGroup~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CmdJointTrajectoryEx-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CmdJointTrajectoryEx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CmdJointTrajectoryEx-request
    (cl:cons ':trajectory (trajectory msg))
))
;//! \htmlinclude CmdJointTrajectoryEx-response.msg.html

(cl:defclass <CmdJointTrajectoryEx-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass CmdJointTrajectoryEx-response (<CmdJointTrajectoryEx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CmdJointTrajectoryEx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CmdJointTrajectoryEx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<CmdJointTrajectoryEx-response> is deprecated: use motoman_msgs-srv:CmdJointTrajectoryEx-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <CmdJointTrajectoryEx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:code-val is deprecated.  Use motoman_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CmdJointTrajectoryEx-response>) ostream)
  "Serializes a message object of type '<CmdJointTrajectoryEx-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CmdJointTrajectoryEx-response>) istream)
  "Deserializes a message object of type '<CmdJointTrajectoryEx-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CmdJointTrajectoryEx-response>)))
  "Returns string type for a service object of type '<CmdJointTrajectoryEx-response>"
  "motoman_msgs/CmdJointTrajectoryExResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdJointTrajectoryEx-response)))
  "Returns string type for a service object of type 'CmdJointTrajectoryEx-response"
  "motoman_msgs/CmdJointTrajectoryExResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CmdJointTrajectoryEx-response>)))
  "Returns md5sum for a message object of type '<CmdJointTrajectoryEx-response>"
  "2c13063e3bd4783949860a7cb1a82ea9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CmdJointTrajectoryEx-response)))
  "Returns md5sum for a message object of type 'CmdJointTrajectoryEx-response"
  "2c13063e3bd4783949860a7cb1a82ea9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CmdJointTrajectoryEx-response>)))
  "Returns full string definition for message of type '<CmdJointTrajectoryEx-response>"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CmdJointTrajectoryEx-response)))
  "Returns full string definition for message of type 'CmdJointTrajectoryEx-response"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CmdJointTrajectoryEx-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CmdJointTrajectoryEx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CmdJointTrajectoryEx-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CmdJointTrajectoryEx)))
  'CmdJointTrajectoryEx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CmdJointTrajectoryEx)))
  'CmdJointTrajectoryEx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdJointTrajectoryEx)))
  "Returns string type for a service object of type '<CmdJointTrajectoryEx>"
  "motoman_msgs/CmdJointTrajectoryEx")