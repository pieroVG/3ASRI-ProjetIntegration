; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude ReadSingleIO-request.msg.html

(cl:defclass <ReadSingleIO-request> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0))
)

(cl:defclass ReadSingleIO-request (<ReadSingleIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadSingleIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadSingleIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<ReadSingleIO-request> is deprecated: use motoman_msgs-srv:ReadSingleIO-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <ReadSingleIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:address-val is deprecated.  Use motoman_msgs-srv:address instead.")
  (address m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadSingleIO-request>) ostream)
  "Serializes a message object of type '<ReadSingleIO-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadSingleIO-request>) istream)
  "Deserializes a message object of type '<ReadSingleIO-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadSingleIO-request>)))
  "Returns string type for a service object of type '<ReadSingleIO-request>"
  "motoman_msgs/ReadSingleIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadSingleIO-request)))
  "Returns string type for a service object of type 'ReadSingleIO-request"
  "motoman_msgs/ReadSingleIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadSingleIO-request>)))
  "Returns md5sum for a message object of type '<ReadSingleIO-request>"
  "46cc8843a2b05da36149122d04a523ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadSingleIO-request)))
  "Returns md5sum for a message object of type 'ReadSingleIO-request"
  "46cc8843a2b05da36149122d04a523ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadSingleIO-request>)))
  "Returns full string definition for message of type '<ReadSingleIO-request>"
  (cl:format cl:nil "~%# Read (and return) the current value of the IO element at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# There are no restrictions as to which IO elements can be read, but they have~%# to exist on the controller and be configured correctly.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadSingleIO-request)))
  "Returns full string definition for message of type 'ReadSingleIO-request"
  (cl:format cl:nil "~%# Read (and return) the current value of the IO element at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# There are no restrictions as to which IO elements can be read, but they have~%# to exist on the controller and be configured correctly.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadSingleIO-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadSingleIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadSingleIO-request
    (cl:cons ':address (address msg))
))
;//! \htmlinclude ReadSingleIO-response.msg.html

(cl:defclass <ReadSingleIO-response> (roslisp-msg-protocol:ros-message)
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
    :type cl:integer
    :initform 0))
)

(cl:defclass ReadSingleIO-response (<ReadSingleIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadSingleIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadSingleIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<ReadSingleIO-response> is deprecated: use motoman_msgs-srv:ReadSingleIO-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ReadSingleIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReadSingleIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ReadSingleIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:value-val is deprecated.  Use motoman_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadSingleIO-response>) ostream)
  "Serializes a message object of type '<ReadSingleIO-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadSingleIO-response>) istream)
  "Deserializes a message object of type '<ReadSingleIO-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadSingleIO-response>)))
  "Returns string type for a service object of type '<ReadSingleIO-response>"
  "motoman_msgs/ReadSingleIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadSingleIO-response)))
  "Returns string type for a service object of type 'ReadSingleIO-response"
  "motoman_msgs/ReadSingleIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadSingleIO-response>)))
  "Returns md5sum for a message object of type '<ReadSingleIO-response>"
  "46cc8843a2b05da36149122d04a523ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadSingleIO-response)))
  "Returns md5sum for a message object of type 'ReadSingleIO-response"
  "46cc8843a2b05da36149122d04a523ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadSingleIO-response>)))
  "Returns full string definition for message of type '<ReadSingleIO-response>"
  (cl:format cl:nil "string message~%bool success~%int32 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadSingleIO-response)))
  "Returns full string definition for message of type 'ReadSingleIO-response"
  (cl:format cl:nil "string message~%bool success~%int32 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadSingleIO-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadSingleIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadSingleIO-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadSingleIO)))
  'ReadSingleIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadSingleIO)))
  'ReadSingleIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadSingleIO)))
  "Returns string type for a service object of type '<ReadSingleIO>"
  "motoman_msgs/ReadSingleIO")