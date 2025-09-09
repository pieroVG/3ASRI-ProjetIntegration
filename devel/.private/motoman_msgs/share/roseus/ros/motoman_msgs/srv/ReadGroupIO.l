;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::ReadGroupIO)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'ReadGroupIO (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::READGROUPIO")
  (make-package "MOTOMAN_MSGS::READGROUPIO"))
(unless (find-package "MOTOMAN_MSGS::READGROUPIOREQUEST")
  (make-package "MOTOMAN_MSGS::READGROUPIOREQUEST"))
(unless (find-package "MOTOMAN_MSGS::READGROUPIORESPONSE")
  (make-package "MOTOMAN_MSGS::READGROUPIORESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::ReadGroupIORequest
  :super ros::object
  :slots (_address ))

(defmethod motoman_msgs::ReadGroupIORequest
  (:init
   (&key
    ((:address __address) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:serialization-length
   ()
   (+
    ;; uint32 _address
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _address
       (write-long _address s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motoman_msgs::ReadGroupIOResponse
  :super ros::object
  :slots (_message _success _value ))

(defmethod motoman_msgs::ReadGroupIOResponse
  (:init
   (&key
    ((:message __message) "")
    ((:success __success) nil)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _success __success)
   (setq _value (round __value))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; bool _success
    1
    ;; uint8 _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _value
       (write-byte _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass motoman_msgs::ReadGroupIO
  :super ros::object
  :slots ())

(setf (get motoman_msgs::ReadGroupIO :md5sum-) "03d3163d23ccb5ce5e6b4c3d424efed3")
(setf (get motoman_msgs::ReadGroupIO :datatype-) "motoman_msgs/ReadGroupIO")
(setf (get motoman_msgs::ReadGroupIO :request) motoman_msgs::ReadGroupIORequest)
(setf (get motoman_msgs::ReadGroupIO :response) motoman_msgs::ReadGroupIOResponse)

(defmethod motoman_msgs::ReadGroupIORequest
  (:response () (instance motoman_msgs::ReadGroupIOResponse :init)))

(setf (get motoman_msgs::ReadGroupIORequest :md5sum-) "03d3163d23ccb5ce5e6b4c3d424efed3")
(setf (get motoman_msgs::ReadGroupIORequest :datatype-) "motoman_msgs/ReadGroupIORequest")
(setf (get motoman_msgs::ReadGroupIORequest :definition-)
      "
# Read (and return) the current value of the Group IO element at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# There are no restrictions as to which group IO elements can be read, but they
# have to exist on the controller and be configured correctly.
#
# NOTE: many programming languages will parse literals starting with '0' as
#       octal numbers. Do not add leading zeros to group addresses to avoid
#       specifying the wrong address to read.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
---
string message
bool success
uint8 value

")

(setf (get motoman_msgs::ReadGroupIOResponse :md5sum-) "03d3163d23ccb5ce5e6b4c3d424efed3")
(setf (get motoman_msgs::ReadGroupIOResponse :datatype-) "motoman_msgs/ReadGroupIOResponse")
(setf (get motoman_msgs::ReadGroupIOResponse :definition-)
      "
# Read (and return) the current value of the Group IO element at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# There are no restrictions as to which group IO elements can be read, but they
# have to exist on the controller and be configured correctly.
#
# NOTE: many programming languages will parse literals starting with '0' as
#       octal numbers. Do not add leading zeros to group addresses to avoid
#       specifying the wrong address to read.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
---
string message
bool success
uint8 value

")



(provide :motoman_msgs/ReadGroupIO "03d3163d23ccb5ce5e6b4c3d424efed3")


