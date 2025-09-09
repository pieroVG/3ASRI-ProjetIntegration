; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude SelectTool-request.msg.html

(cl:defclass <SelectTool-request> (roslisp-msg-protocol:ros-message)
  ((group_number
    :reader group_number
    :initarg :group_number
    :type cl:integer
    :initform 0)
   (tool_number
    :reader tool_number
    :initarg :tool_number
    :type cl:integer
    :initform 0))
)

(cl:defclass SelectTool-request (<SelectTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<SelectTool-request> is deprecated: use motoman_msgs-srv:SelectTool-request instead.")))

(cl:ensure-generic-function 'group_number-val :lambda-list '(m))
(cl:defmethod group_number-val ((m <SelectTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:group_number-val is deprecated.  Use motoman_msgs-srv:group_number instead.")
  (group_number m))

(cl:ensure-generic-function 'tool_number-val :lambda-list '(m))
(cl:defmethod tool_number-val ((m <SelectTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:tool_number-val is deprecated.  Use motoman_msgs-srv:tool_number instead.")
  (tool_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectTool-request>) ostream)
  "Serializes a message object of type '<SelectTool-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'group_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'group_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'group_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'group_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tool_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tool_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tool_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tool_number)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectTool-request>) istream)
  "Deserializes a message object of type '<SelectTool-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'group_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'group_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'group_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'group_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tool_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tool_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tool_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tool_number)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectTool-request>)))
  "Returns string type for a service object of type '<SelectTool-request>"
  "motoman_msgs/SelectToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectTool-request)))
  "Returns string type for a service object of type 'SelectTool-request"
  "motoman_msgs/SelectToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectTool-request>)))
  "Returns md5sum for a message object of type '<SelectTool-request>"
  "afb2fb884fcd9c28c81c82056d543156")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectTool-request)))
  "Returns md5sum for a message object of type 'SelectTool-request"
  "afb2fb884fcd9c28c81c82056d543156")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectTool-request>)))
  "Returns full string definition for message of type '<SelectTool-request>"
  (cl:format cl:nil "~%# Change the active tool file on the controller.~%#~%# This changes the tool definition used for both (Moto)ROS-controlled motions~%# and manual jogging.~%#~%#~%# ## Controller support~%#~%# This service wraps two distinct (but related) actions:~%#~%#  1. changing the tool file used for execution of (Moto)ROS-controlled motions~%#  2. changing the tool file used for jogging~%#~%# Action 1 is supported by all controllers supported by MotoROS (ie: DX100,~%# FS100, DX200 and YRC1000(u)).~%#~%# Action 2 is NOT supported on FS100 controllers. This is a limitation of~%# MotoPlus, not of MotoROS.~%#~%# In all cases, make sure to read the following sections carefully to be aware~%# of the behaviour of this service and of the Yaskawa controller after calling~%# this service.~%#~%#~%# ## FSU and PFL~%#~%# Tool switches through this service will be taken into account by the FSU and~%# PFL safety systems if the controller has these options and they are enabled.~%#~%# Tool interference files, if associated with certain tool file definitions,~%# will therefore also switch.~%#~%#~%# ## Tool switch timing~%#~%# The active tool will be switched AFTER the controller has completed execution~%# of the last trajectory segment that was sent to MotoROS BEFORE this service~%# was called.~%#~%# The motion queue internal to MotoROS caches a number of segments in a FIFO.~%# Only segments received AFTER this service was invoked will be executed with~%# the new tool. Any segments received before a tool switch was invoked will use~%# the old tool.~%#~%# To ensure motion will be executed using a certain tool, applications should~%# check the 'in_motion' field (part of the RobotStatus message published on the~%# 'robot_status' topic by the driver) and invoke the service when the robot has~%# come to a stop (and the motion queue of MotoROS is empty). Any subsequent~%# trajectories will use the new tool.~%#~%#~%# ## Effect on trajectory execution~%#~%# As MotoROS currently only executes joint space trajectories, changing tool~%# file with this service DOES NOT affect the execution of those trajectories.~%#~%# As noted earlier though, the active tool file definition will affect FSU and~%# PFL behaviour, as the tool definition is used in calculation of dynamics and~%# safety (see \"FSU and PFL\" above).~%#~%# To clarify: the TCP definition of the tool file is NOT taken into account when~%# calculating manipulator motion based on incoming ROS JointTrajectoryAction~%# goals (as JointTrajectory goals do not include any Cartesian poses, only~%# absolute joint space coordinates for each axis).~%#~%# Instead, ROS applications should use different TF frames to define tool frames~%# on the ROS side and plan their motions with the appropriate TF frame as the~%# active tool.~%#~%# This service could then be used to notify the controller of other tool~%# characteristics, such as weight, CoG and inertia by switching to a tool file~%# configured with those parameters.~%#~%#~%# ## Retrieving the active tool file~%#~%# MotoROS does not currently support retrieving the active tool file.~%#~%#~%#~%# For more information about tool file configuration, selection and use on~%# Yaskawa controllers, refer to the relevant Yaskawa Motoman documentation.~%~%~%# Numeric ID of the group the tool file is defined for.~%#~%# This MUST correspond to a group ID currently defined and active on the~%# controller. The ROS service does not check whether the value specified here~%# is legal. The MotoROS server will however check this, and will refuse to~%# switch to an illegal value and will report this to the ROS driver.~%#~%# Callers will be notified of such failures by 'success' being set to 'false'~%# and an appropriate error message being provided via the 'message' field of~%# the service response.~%#~%# NOTE: this field is 0-based, with 0 corresponding to the first motion group,~%#       1 the second, etc.~%#~%# legal-values: [0, total_nr_of_groups)~%# required: yes (absence-causes-service-failure)~%# default: no-default~%uint32 group_number~%~%# Numeric ID of the tool file to switch to for the specified group.~%#~%# Identical to 'group_number', specification of illegal values will result~%# in failure to invoke the MotoROS service, and an unsuccessful service result~%# being returned.~%#~%# NOTE: this field is 0-based, with 0 corresponding to the first tool file,~%#       1 the second, etc.~%#~%# legal-values: [0, 63]~%# required: yes (absence-causes-service-failure)~%# default: no-default~%uint32 tool_number~%~%# TODO: might want to expose 'sequence_number' here as well~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectTool-request)))
  "Returns full string definition for message of type 'SelectTool-request"
  (cl:format cl:nil "~%# Change the active tool file on the controller.~%#~%# This changes the tool definition used for both (Moto)ROS-controlled motions~%# and manual jogging.~%#~%#~%# ## Controller support~%#~%# This service wraps two distinct (but related) actions:~%#~%#  1. changing the tool file used for execution of (Moto)ROS-controlled motions~%#  2. changing the tool file used for jogging~%#~%# Action 1 is supported by all controllers supported by MotoROS (ie: DX100,~%# FS100, DX200 and YRC1000(u)).~%#~%# Action 2 is NOT supported on FS100 controllers. This is a limitation of~%# MotoPlus, not of MotoROS.~%#~%# In all cases, make sure to read the following sections carefully to be aware~%# of the behaviour of this service and of the Yaskawa controller after calling~%# this service.~%#~%#~%# ## FSU and PFL~%#~%# Tool switches through this service will be taken into account by the FSU and~%# PFL safety systems if the controller has these options and they are enabled.~%#~%# Tool interference files, if associated with certain tool file definitions,~%# will therefore also switch.~%#~%#~%# ## Tool switch timing~%#~%# The active tool will be switched AFTER the controller has completed execution~%# of the last trajectory segment that was sent to MotoROS BEFORE this service~%# was called.~%#~%# The motion queue internal to MotoROS caches a number of segments in a FIFO.~%# Only segments received AFTER this service was invoked will be executed with~%# the new tool. Any segments received before a tool switch was invoked will use~%# the old tool.~%#~%# To ensure motion will be executed using a certain tool, applications should~%# check the 'in_motion' field (part of the RobotStatus message published on the~%# 'robot_status' topic by the driver) and invoke the service when the robot has~%# come to a stop (and the motion queue of MotoROS is empty). Any subsequent~%# trajectories will use the new tool.~%#~%#~%# ## Effect on trajectory execution~%#~%# As MotoROS currently only executes joint space trajectories, changing tool~%# file with this service DOES NOT affect the execution of those trajectories.~%#~%# As noted earlier though, the active tool file definition will affect FSU and~%# PFL behaviour, as the tool definition is used in calculation of dynamics and~%# safety (see \"FSU and PFL\" above).~%#~%# To clarify: the TCP definition of the tool file is NOT taken into account when~%# calculating manipulator motion based on incoming ROS JointTrajectoryAction~%# goals (as JointTrajectory goals do not include any Cartesian poses, only~%# absolute joint space coordinates for each axis).~%#~%# Instead, ROS applications should use different TF frames to define tool frames~%# on the ROS side and plan their motions with the appropriate TF frame as the~%# active tool.~%#~%# This service could then be used to notify the controller of other tool~%# characteristics, such as weight, CoG and inertia by switching to a tool file~%# configured with those parameters.~%#~%#~%# ## Retrieving the active tool file~%#~%# MotoROS does not currently support retrieving the active tool file.~%#~%#~%#~%# For more information about tool file configuration, selection and use on~%# Yaskawa controllers, refer to the relevant Yaskawa Motoman documentation.~%~%~%# Numeric ID of the group the tool file is defined for.~%#~%# This MUST correspond to a group ID currently defined and active on the~%# controller. The ROS service does not check whether the value specified here~%# is legal. The MotoROS server will however check this, and will refuse to~%# switch to an illegal value and will report this to the ROS driver.~%#~%# Callers will be notified of such failures by 'success' being set to 'false'~%# and an appropriate error message being provided via the 'message' field of~%# the service response.~%#~%# NOTE: this field is 0-based, with 0 corresponding to the first motion group,~%#       1 the second, etc.~%#~%# legal-values: [0, total_nr_of_groups)~%# required: yes (absence-causes-service-failure)~%# default: no-default~%uint32 group_number~%~%# Numeric ID of the tool file to switch to for the specified group.~%#~%# Identical to 'group_number', specification of illegal values will result~%# in failure to invoke the MotoROS service, and an unsuccessful service result~%# being returned.~%#~%# NOTE: this field is 0-based, with 0 corresponding to the first tool file,~%#       1 the second, etc.~%#~%# legal-values: [0, 63]~%# required: yes (absence-causes-service-failure)~%# default: no-default~%uint32 tool_number~%~%# TODO: might want to expose 'sequence_number' here as well~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectTool-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectTool-request
    (cl:cons ':group_number (group_number msg))
    (cl:cons ':tool_number (tool_number msg))
))
;//! \htmlinclude SelectTool-response.msg.html

(cl:defclass <SelectTool-response> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SelectTool-response (<SelectTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<SelectTool-response> is deprecated: use motoman_msgs-srv:SelectTool-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SelectTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SelectTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectTool-response>) ostream)
  "Serializes a message object of type '<SelectTool-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectTool-response>) istream)
  "Deserializes a message object of type '<SelectTool-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectTool-response>)))
  "Returns string type for a service object of type '<SelectTool-response>"
  "motoman_msgs/SelectToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectTool-response)))
  "Returns string type for a service object of type 'SelectTool-response"
  "motoman_msgs/SelectToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectTool-response>)))
  "Returns md5sum for a message object of type '<SelectTool-response>"
  "afb2fb884fcd9c28c81c82056d543156")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectTool-response)))
  "Returns md5sum for a message object of type 'SelectTool-response"
  "afb2fb884fcd9c28c81c82056d543156")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectTool-response>)))
  "Returns full string definition for message of type '<SelectTool-response>"
  (cl:format cl:nil "~%# A human-readable error message, or an empty string if there was no error.~%string message~%~%# true IFF invocation of the MotoROS service was successful.~%#~%# NOTE: this is independent of whether the ROS service invocation was~%#       successful. In absence of any ROS middleware failures, a failed MotoROS~%#       service invocation will result in 'success' here being set to 'false',~%#       but a successful ROS service invocation.~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectTool-response)))
  "Returns full string definition for message of type 'SelectTool-response"
  (cl:format cl:nil "~%# A human-readable error message, or an empty string if there was no error.~%string message~%~%# true IFF invocation of the MotoROS service was successful.~%#~%# NOTE: this is independent of whether the ROS service invocation was~%#       successful. In absence of any ROS middleware failures, a failed MotoROS~%#       service invocation will result in 'success' here being set to 'false',~%#       but a successful ROS service invocation.~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectTool-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectTool-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SelectTool)))
  'SelectTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SelectTool)))
  'SelectTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectTool)))
  "Returns string type for a service object of type '<SelectTool>"
  "motoman_msgs/SelectTool")