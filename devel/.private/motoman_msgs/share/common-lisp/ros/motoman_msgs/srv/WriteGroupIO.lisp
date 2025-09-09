; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-srv)


;//! \htmlinclude WriteGroupIO-request.msg.html

(cl:defclass <WriteGroupIO-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass WriteGroupIO-request (<WriteGroupIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteGroupIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteGroupIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<WriteGroupIO-request> is deprecated: use motoman_msgs-srv:WriteGroupIO-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <WriteGroupIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:address-val is deprecated.  Use motoman_msgs-srv:address instead.")
  (address m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <WriteGroupIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:value-val is deprecated.  Use motoman_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteGroupIO-request>) ostream)
  "Serializes a message object of type '<WriteGroupIO-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteGroupIO-request>) istream)
  "Deserializes a message object of type '<WriteGroupIO-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteGroupIO-request>)))
  "Returns string type for a service object of type '<WriteGroupIO-request>"
  "motoman_msgs/WriteGroupIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteGroupIO-request)))
  "Returns string type for a service object of type 'WriteGroupIO-request"
  "motoman_msgs/WriteGroupIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteGroupIO-request>)))
  "Returns md5sum for a message object of type '<WriteGroupIO-request>"
  "442d77ce292ab13600e884d7860398a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteGroupIO-request)))
  "Returns md5sum for a message object of type 'WriteGroupIO-request"
  "442d77ce292ab13600e884d7860398a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteGroupIO-request>)))
  "Returns full string definition for message of type '<WriteGroupIO-request>"
  (cl:format cl:nil "~%# Write 'value' to the Group IO element at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be written to:~%#~%#  - 2701 and up : Network Inputs (2501 and up on DX100 and FS100)~%#  - 1001 and up : Universal/General Outputs~%#~%# NOTE: many programming languages will parse literals starting with '0' as~%#       octal numbers. Do not add leading zeros to group addresses to avoid~%#       specifying the wrong address to write to.~%~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%uint8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteGroupIO-request)))
  "Returns full string definition for message of type 'WriteGroupIO-request"
  (cl:format cl:nil "~%# Write 'value' to the Group IO element at 'address'.~%#~%# Addresses are plain, base-10 integers, as used and displayed by the controller~%# (on the teach pendant for instance).~%#~%# Only the following addresses can be written to:~%#~%#  - 2701 and up : Network Inputs (2501 and up on DX100 and FS100)~%#  - 1001 and up : Universal/General Outputs~%#~%# NOTE: many programming languages will parse literals starting with '0' as~%#       octal numbers. Do not add leading zeros to group addresses to avoid~%#       specifying the wrong address to write to.~%~%# Refer also the Yaskawa Motoman documentation on IO addressing and~%# configuration.~%~%uint32 address~%uint8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteGroupIO-request>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteGroupIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteGroupIO-request
    (cl:cons ':address (address msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude WriteGroupIO-response.msg.html

(cl:defclass <WriteGroupIO-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass WriteGroupIO-response (<WriteGroupIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteGroupIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteGroupIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-srv:<WriteGroupIO-response> is deprecated: use motoman_msgs-srv:WriteGroupIO-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <WriteGroupIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:message-val is deprecated.  Use motoman_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <WriteGroupIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-srv:success-val is deprecated.  Use motoman_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteGroupIO-response>) ostream)
  "Serializes a message object of type '<WriteGroupIO-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteGroupIO-response>) istream)
  "Deserializes a message object of type '<WriteGroupIO-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteGroupIO-response>)))
  "Returns string type for a service object of type '<WriteGroupIO-response>"
  "motoman_msgs/WriteGroupIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteGroupIO-response)))
  "Returns string type for a service object of type 'WriteGroupIO-response"
  "motoman_msgs/WriteGroupIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteGroupIO-response>)))
  "Returns md5sum for a message object of type '<WriteGroupIO-response>"
  "442d77ce292ab13600e884d7860398a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteGroupIO-response)))
  "Returns md5sum for a message object of type 'WriteGroupIO-response"
  "442d77ce292ab13600e884d7860398a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteGroupIO-response>)))
  "Returns full string definition for message of type '<WriteGroupIO-response>"
  (cl:format cl:nil "string message~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteGroupIO-response)))
  "Returns full string definition for message of type 'WriteGroupIO-response"
  (cl:format cl:nil "string message~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteGroupIO-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteGroupIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteGroupIO-response
    (cl:cons ':message (message msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WriteGroupIO)))
  'WriteGroupIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WriteGroupIO)))
  'WriteGroupIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteGroupIO)))
  "Returns string type for a service object of type '<WriteGroupIO>"
  "motoman_msgs/WriteGroupIO")