; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude WriteMRegister-request.msg.html

(cl:defclass <WriteMRegister-request> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WriteMRegister-request (<WriteMRegister-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteMRegister-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteMRegister-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<WriteMRegister-request> is deprecated: use motoman_msgs-srv:WriteMRegister-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <WriteMRegister-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:address-val is deprecated.  Use motoman_msgs-srv:address instead.")
  (address m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <WriteMRegister-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:value-val is deprecated.  Use motoman_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteMRegister-request>) ostream)
  "Serializes a message object of type '<WriteMRegister-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteMRegister-request>) istream)
  "Deserializes a message object of type '<WriteMRegister-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteMRegister-request>)))
  "Returns string type for a service object of type '<WriteMRegister-request>"
  "motoman_msgs/WriteMRegisterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteMRegister-request)))
  "Returns string type for a service object of type 'WriteMRegister-request"
  "motoman_msgs/WriteMRegisterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteMRegister-request>)))
  "Returns md5sum for a message object of type '<WriteMRegister-request>"
  "6b8bb3bb740ab645ad65c2abec91e614")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteMRegister-request)))
  "Returns md5sum for a message object of type 'WriteMRegister-request"
  "6b8bb3bb740ab645ad65c2abec91e614")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteMRegister-request>)))
  "Returns full string definition for message of type '<WriteMRegister-request>"
  (cl:format cl:nil "~%# Write 'value' to the M register at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be written to:~%#~%#  - 0 to 559~%#~%# NOTE 1: do not add 1000000 to the address, MotoROS will do this when~%#         necessary.~%#~%# NOTE 2: many programming languages will parse literals starting with '0' as~%#         octal numbers. Do not add leading zeros to register addresses to avoid~%#         specifying the wrong register to write to.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%uint16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteMRegister-request)))
  "Returns full string definition for message of type 'WriteMRegister-request"
  (cl:format cl:nil "~%# Write 'value' to the M register at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be written to:~%#~%#  - 0 to 559~%#~%# NOTE 1: do not add 1000000 to the address, MotoROS will do this when~%#         necessary.~%#~%# NOTE 2: many programming languages will parse literals starting with '0' as~%#         octal numbers. Do not add leading zeros to register addresses to avoid~%#         specifying the wrong register to write to.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%uint16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteMRegister-request>))
  (cl:+ 0
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteMRegister-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteMRegister-request
    (cl:cons ':address (address msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude WriteMRegister-response.msg.html

(cl:defclass <WriteMRegister-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass WriteMRegister-response (<WriteMRegister-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteMRegister-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteMRegister-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<WriteMRegister-response> is deprecated: use motoman_msgs-srv:WriteMRegister-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <WriteMRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <WriteMRegister-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteMRegister-response>) ostream)
  "Serializes a message object of type '<WriteMRegister-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteMRegister-response>) istream)
  "Deserializes a message object of type '<WriteMRegister-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteMRegister-response>)))
  "Returns string type for a service object of type '<WriteMRegister-response>"
  "motoman_msgs/WriteMRegisterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteMRegister-response)))
  "Returns string type for a service object of type 'WriteMRegister-response"
  "motoman_msgs/WriteMRegisterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteMRegister-response>)))
  "Returns md5sum for a message object of type '<WriteMRegister-response>"
  "6b8bb3bb740ab645ad65c2abec91e614")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteMRegister-response)))
  "Returns md5sum for a message object of type 'WriteMRegister-response"
  "6b8bb3bb740ab645ad65c2abec91e614")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteMRegister-response>)))
  "Returns full string definition for message of type '<WriteMRegister-response>"
  (cl:format cl:nil "string message~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteMRegister-response)))
  "Returns full string definition for message of type 'WriteMRegister-response"
  (cl:format cl:nil "string message~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteMRegister-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteMRegister-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteMRegister-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WriteMRegister)))
  'WriteMRegister-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WriteMRegister)))
  'WriteMRegister-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteMRegister)))
  "Returns string type for a service object of type '<WriteMRegister>"
  "motoman_msgs/WriteMRegister")