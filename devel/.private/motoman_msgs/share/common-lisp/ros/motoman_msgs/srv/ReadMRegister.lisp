; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude ReadMRegister-request.msg.html

(cl:defclass <ReadMRegister-request> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0))
)

(cl:defclass ReadMRegister-request (<ReadMRegister-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadMRegister-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadMRegister-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<ReadMRegister-request> is deprecated: use motoman_msgs-srv:ReadMRegister-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <ReadMRegister-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:address-val is deprecated.  Use motoman_msgs-srv:address instead.")
  (address m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadMRegister-request>) ostream)
  "Serializes a message object of type '<ReadMRegister-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadMRegister-request>) istream)
  "Deserializes a message object of type '<ReadMRegister-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadMRegister-request>)))
  "Returns string type for a service object of type '<ReadMRegister-request>"
  "motoman_msgs/ReadMRegisterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMRegister-request)))
  "Returns string type for a service object of type 'ReadMRegister-request"
  "motoman_msgs/ReadMRegisterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadMRegister-request>)))
  "Returns md5sum for a message object of type '<ReadMRegister-request>"
  "a3d35ac86126c0934861b8c43f69c8b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadMRegister-request)))
  "Returns md5sum for a message object of type 'ReadMRegister-request"
  "a3d35ac86126c0934861b8c43f69c8b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadMRegister-request>)))
  "Returns full string definition for message of type '<ReadMRegister-request>"
  (cl:format cl:nil "~%# Read (and return) the current value of the M register at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be read from:~%#~%#  - 0 to 999~%#~%# NOTE 1: do not add 1000000 to the address, MotoROS will do this when~%#         necessary.~%#~%# NOTE 2: many programming languages will parse literals starting with '0' as~%#         octal numbers. Do not add leading zeros to register addresses to avoid~%#         specifying the wrong register to read.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadMRegister-request)))
  "Returns full string definition for message of type 'ReadMRegister-request"
  (cl:format cl:nil "~%# Read (and return) the current value of the M register at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be read from:~%#~%#  - 0 to 999~%#~%# NOTE 1: do not add 1000000 to the address, MotoROS will do this when~%#         necessary.~%#~%# NOTE 2: many programming languages will parse literals starting with '0' as~%#         octal numbers. Do not add leading zeros to register addresses to avoid~%#         specifying the wrong register to read.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadMRegister-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadMRegister-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadMRegister-request
    (cl:cons ':address (address msg))
))
;//! \htmlinclude ReadMRegister-response.msg.html

(cl:defclass <ReadMRegister-response> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ReadMRegister-response (<ReadMRegister-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadMRegister-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadMRegister-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<ReadMRegister-response> is deprecated: use motoman_msgs-srv:ReadMRegister-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ReadMRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReadMRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ReadMRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:value-val is deprecated.  Use motoman_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadMRegister-response>) ostream)
  "Serializes a message object of type '<ReadMRegister-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadMRegister-response>) istream)
  "Deserializes a message object of type '<ReadMRegister-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadMRegister-response>)))
  "Returns string type for a service object of type '<ReadMRegister-response>"
  "motoman_msgs/ReadMRegisterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMRegister-response)))
  "Returns string type for a service object of type 'ReadMRegister-response"
  "motoman_msgs/ReadMRegisterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadMRegister-response>)))
  "Returns md5sum for a message object of type '<ReadMRegister-response>"
  "a3d35ac86126c0934861b8c43f69c8b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadMRegister-response)))
  "Returns md5sum for a message object of type 'ReadMRegister-response"
  "a3d35ac86126c0934861b8c43f69c8b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadMRegister-response>)))
  "Returns full string definition for message of type '<ReadMRegister-response>"
  (cl:format cl:nil "string message~%bool success~%uint16 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadMRegister-response)))
  "Returns full string definition for message of type 'ReadMRegister-response"
  (cl:format cl:nil "string message~%bool success~%uint16 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadMRegister-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadMRegister-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadMRegister-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadMRegister)))
  'ReadMRegister-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadMRegister)))
  'ReadMRegister-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadMRegister)))
  "Returns string type for a service object of type '<ReadMRegister>"
  "motoman_msgs/ReadMRegister")