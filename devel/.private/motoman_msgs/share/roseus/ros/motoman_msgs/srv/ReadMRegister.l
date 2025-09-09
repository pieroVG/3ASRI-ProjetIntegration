;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::ReadMRegister)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'ReadMRegister (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::READMREGISTER")
  (make-package "MOTOMAN_MSGS::READMREGISTER"))
(unless (find-package "MOTOMAN_MSGS::READMREGISTERREQUEST")
  (make-package "MOTOMAN_MSGS::READMREGISTERREQUEST"))
(unless (find-package "MOTOMAN_MSGS::READMREGISTERRESPONSE")
  (make-package "MOTOMAN_MSGS::READMREGISTERRESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::ReadMRegisterRequest
  :super ros::object
  :slots (_address ))

(defmethod motoman_msgs::ReadMRegisterRequest
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

(defclass motoman_msgs::ReadMRegisterResponse
  :super ros::object
  :slots (_message _success _value ))

(defmethod motoman_msgs::ReadMRegisterResponse
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
    ;; uint16 _value
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _value
       (write-word _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass motoman_msgs::ReadMRegister
  :super ros::object
  :slots ())

(setf (get motoman_msgs::ReadMRegister :md5sum-) "a3d35ac86126c0934861b8c43f69c8b8")
(setf (get motoman_msgs::ReadMRegister :datatype-) "motoman_msgs/ReadMRegister")
(setf (get motoman_msgs::ReadMRegister :request) motoman_msgs::ReadMRegisterRequest)
(setf (get motoman_msgs::ReadMRegister :response) motoman_msgs::ReadMRegisterResponse)

(defmethod motoman_msgs::ReadMRegisterRequest
  (:response () (instance motoman_msgs::ReadMRegisterResponse :init)))

(setf (get motoman_msgs::ReadMRegisterRequest :md5sum-) "a3d35ac86126c0934861b8c43f69c8b8")
(setf (get motoman_msgs::ReadMRegisterRequest :datatype-) "motoman_msgs/ReadMRegisterRequest")
(setf (get motoman_msgs::ReadMRegisterRequest :definition-)
      "
# Read (and return) the current value of the M register at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be read from:
#
#  - 0 to 999
#
# NOTE 1: do not add 1000000 to the address, MotoROS will do this when
#         necessary.
#
# NOTE 2: many programming languages will parse literals starting with '0' as
#         octal numbers. Do not add leading zeros to register addresses to avoid
#         specifying the wrong register to read.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
---
string message
bool success
uint16 value

")

(setf (get motoman_msgs::ReadMRegisterResponse :md5sum-) "a3d35ac86126c0934861b8c43f69c8b8")
(setf (get motoman_msgs::ReadMRegisterResponse :datatype-) "motoman_msgs/ReadMRegisterResponse")
(setf (get motoman_msgs::ReadMRegisterResponse :definition-)
      "
# Read (and return) the current value of the M register at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be read from:
#
#  - 0 to 999
#
# NOTE 1: do not add 1000000 to the address, MotoROS will do this when
#         necessary.
#
# NOTE 2: many programming languages will parse literals starting with '0' as
#         octal numbers. Do not add leading zeros to register addresses to avoid
#         specifying the wrong register to read.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
---
string message
bool success
uint16 value

")



(provide :motoman_msgs/ReadMRegister "a3d35ac86126c0934861b8c43f69c8b8")


