; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude WriteSingleIO-request.msg.html

(cl:defclass <WriteSingleIO-request> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass WriteSingleIO-request (<WriteSingleIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteSingleIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteSingleIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<WriteSingleIO-request> is deprecated: use motoman_msgs-srv:WriteSingleIO-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <WriteSingleIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:address-val is deprecated.  Use motoman_msgs-srv:address instead.")
  (address m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <WriteSingleIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:value-val is deprecated.  Use motoman_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteSingleIO-request>) ostream)
  "Serializes a message object of type '<WriteSingleIO-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteSingleIO-request>) istream)
  "Deserializes a message object of type '<WriteSingleIO-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteSingleIO-request>)))
  "Returns string type for a service object of type '<WriteSingleIO-request>"
  "motoman_msgs/WriteSingleIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteSingleIO-request)))
  "Returns string type for a service object of type 'WriteSingleIO-request"
  "motoman_msgs/WriteSingleIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteSingleIO-request>)))
  "Returns md5sum for a message object of type '<WriteSingleIO-request>"
  "615ffdbac96f4fdce6e76dce79d5f3b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteSingleIO-request)))
  "Returns md5sum for a message object of type 'WriteSingleIO-request"
  "615ffdbac96f4fdce6e76dce79d5f3b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteSingleIO-request>)))
  "Returns full string definition for message of type '<WriteSingleIO-request>"
  (cl:format cl:nil "~%# Write 'value' to the IO element at 'address'.~%#~%# This service does not return anything.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be written to:~%#~%#  - 27010 and up : Network Inputs (25010 and up on DX100 and FS100)~%#  - 10010 and up : Universal/General Outputs~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteSingleIO-request)))
  "Returns full string definition for message of type 'WriteSingleIO-request"
  (cl:format cl:nil "~%# Write 'value' to the IO element at 'address'.~%#~%# This service does not return anything.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be written to:~%#~%#  - 27010 and up : Network Inputs (25010 and up on DX100 and FS100)~%#  - 10010 and up : Universal/General Outputs~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteSingleIO-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteSingleIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteSingleIO-request
    (cl:cons ':address (address msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude WriteSingleIO-response.msg.html

(cl:defclass <WriteSingleIO-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass WriteSingleIO-response (<WriteSingleIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteSingleIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteSingleIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<WriteSingleIO-response> is deprecated: use motoman_msgs-srv:WriteSingleIO-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <WriteSingleIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <WriteSingleIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteSingleIO-response>) ostream)
  "Serializes a message object of type '<WriteSingleIO-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteSingleIO-response>) istream)
  "Deserializes a message object of type '<WriteSingleIO-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteSingleIO-response>)))
  "Returns string type for a service object of type '<WriteSingleIO-response>"
  "motoman_msgs/WriteSingleIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteSingleIO-response)))
  "Returns string type for a service object of type 'WriteSingleIO-response"
  "motoman_msgs/WriteSingleIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteSingleIO-response>)))
  "Returns md5sum for a message object of type '<WriteSingleIO-response>"
  "615ffdbac96f4fdce6e76dce79d5f3b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteSingleIO-response)))
  "Returns md5sum for a message object of type 'WriteSingleIO-response"
  "615ffdbac96f4fdce6e76dce79d5f3b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteSingleIO-response>)))
  "Returns full string definition for message of type '<WriteSingleIO-response>"
  (cl:format cl:nil "string message~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteSingleIO-response)))
  "Returns full string definition for message of type 'WriteSingleIO-response"
  (cl:format cl:nil "string message~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteSingleIO-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteSingleIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteSingleIO-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WriteSingleIO)))
  'WriteSingleIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WriteSingleIO)))
  'WriteSingleIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteSingleIO)))
  "Returns string type for a service object of type '<WriteSingleIO>"
  "motoman_msgs/WriteSingleIO")