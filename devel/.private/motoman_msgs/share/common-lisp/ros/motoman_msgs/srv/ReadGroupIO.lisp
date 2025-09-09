; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude ReadGroupIO-request.msg.html

(cl:defclass <ReadGroupIO-request> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0))
)

(cl:defclass ReadGroupIO-request (<ReadGroupIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadGroupIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadGroupIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<ReadGroupIO-request> is deprecated: use motoman_msgs-srv:ReadGroupIO-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <ReadGroupIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:address-val is deprecated.  Use motoman_msgs-srv:address instead.")
  (address m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadGroupIO-request>) ostream)
  "Serializes a message object of type '<ReadGroupIO-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadGroupIO-request>) istream)
  "Deserializes a message object of type '<ReadGroupIO-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadGroupIO-request>)))
  "Returns string type for a service object of type '<ReadGroupIO-request>"
  "motoman_msgs/ReadGroupIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadGroupIO-request)))
  "Returns string type for a service object of type 'ReadGroupIO-request"
  "motoman_msgs/ReadGroupIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadGroupIO-request>)))
  "Returns md5sum for a message object of type '<ReadGroupIO-request>"
  "03d3163d23ccb5ce5e6b4c3d424efed3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadGroupIO-request)))
  "Returns md5sum for a message object of type 'ReadGroupIO-request"
  "03d3163d23ccb5ce5e6b4c3d424efed3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadGroupIO-request>)))
  "Returns full string definition for message of type '<ReadGroupIO-request>"
  (cl:format cl:nil "~%# Read (and return) the current value of the Group IO element at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# There are no restrictions as to which group IO elements can be read, but they~%# have to exist on the controller and be configured correctly.~%#~%# NOTE: many programming languages will parse literals starting with '0' as~%#       octal numbers. Do not add leading zeros to group addresses to avoid~%#       specifying the wrong address to read.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadGroupIO-request)))
  "Returns full string definition for message of type 'ReadGroupIO-request"
  (cl:format cl:nil "~%# Read (and return) the current value of the Group IO element at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# There are no restrictions as to which group IO elements can be read, but they~%# have to exist on the controller and be configured correctly.~%#~%# NOTE: many programming languages will parse literals starting with '0' as~%#       octal numbers. Do not add leading zeros to group addresses to avoid~%#       specifying the wrong address to read.~%#~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadGroupIO-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadGroupIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadGroupIO-request
    (cl:cons ':address (address msg))
))
;//! \htmlinclude ReadGroupIO-response.msg.html

(cl:defclass <ReadGroupIO-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ReadGroupIO-response (<ReadGroupIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadGroupIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadGroupIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<ReadGroupIO-response> is deprecated: use motoman_msgs-srv:ReadGroupIO-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ReadGroupIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReadGroupIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ReadGroupIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:value-val is deprecated.  Use motoman_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadGroupIO-response>) ostream)
  "Serializes a message object of type '<ReadGroupIO-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadGroupIO-response>) istream)
  "Deserializes a message object of type '<ReadGroupIO-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadGroupIO-response>)))
  "Returns string type for a service object of type '<ReadGroupIO-response>"
  "motoman_msgs/ReadGroupIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadGroupIO-response)))
  "Returns string type for a service object of type 'ReadGroupIO-response"
  "motoman_msgs/ReadGroupIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadGroupIO-response>)))
  "Returns md5sum for a message object of type '<ReadGroupIO-response>"
  "03d3163d23ccb5ce5e6b4c3d424efed3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadGroupIO-response)))
  "Returns md5sum for a message object of type 'ReadGroupIO-response"
  "03d3163d23ccb5ce5e6b4c3d424efed3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadGroupIO-response>)))
  "Returns full string definition for message of type '<ReadGroupIO-response>"
  (cl:format cl:nil "string message~%bool success~%uint8 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadGroupIO-response)))
  "Returns full string definition for message of type 'ReadGroupIO-response"
  (cl:format cl:nil "string message~%bool success~%uint8 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadGroupIO-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadGroupIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadGroupIO-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadGroupIO)))
  'ReadGroupIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadGroupIO)))
  'ReadGroupIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadGroupIO)))
  "Returns string type for a service object of type '<ReadGroupIO>"
  "motoman_msgs/ReadGroupIO")