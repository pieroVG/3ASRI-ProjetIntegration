;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::WriteMRegister)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'WriteMRegister (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::WRITEMREGISTER")
  (make-package "MOTOMAN_MSGS::WRITEMREGISTER"))
(unless (find-package "MOTOMAN_MSGS::WRITEMREGISTERREQUEST")
  (make-package "MOTOMAN_MSGS::WRITEMREGISTERREQUEST"))
(unless (find-package "MOTOMAN_MSGS::WRITEMREGISTERRESPONSE")
  (make-package "MOTOMAN_MSGS::WRITEMREGISTERRESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::WriteMRegisterRequest
  :super ros::object
  :slots (_address _value ))

(defmethod motoman_msgs::WriteMRegisterRequest
  (:init
   (&key
    ((:address __address) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _value (round __value))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _address
    4
    ;; uint16 _value
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _address
       (write-long _address s)
     ;; uint16 _value
       (write-word _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass motoman_msgs::WriteMRegisterResponse
  :super ros::object
  :slots (_message _success ))

(defmethod motoman_msgs::WriteMRegisterResponse
  (:init
   (&key
    ((:message __message) "")
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _success __success)
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; bool _success
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass motoman_msgs::WriteMRegister
  :super ros::object
  :slots ())

(setf (get motoman_msgs::WriteMRegister :md5sum-) "6b8bb3bb740ab645ad65c2abec91e614")
(setf (get motoman_msgs::WriteMRegister :datatype-) "motoman_msgs/WriteMRegister")
(setf (get motoman_msgs::WriteMRegister :request) motoman_msgs::WriteMRegisterRequest)
(setf (get motoman_msgs::WriteMRegister :response) motoman_msgs::WriteMRegisterResponse)

(defmethod motoman_msgs::WriteMRegisterRequest
  (:response () (instance motoman_msgs::WriteMRegisterResponse :init)))

(setf (get motoman_msgs::WriteMRegisterRequest :md5sum-) "6b8bb3bb740ab645ad65c2abec91e614")
(setf (get motoman_msgs::WriteMRegisterRequest :datatype-) "motoman_msgs/WriteMRegisterRequest")
(setf (get motoman_msgs::WriteMRegisterRequest :definition-)
      "
# Write 'value' to the M register at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be written to:
#
#  - 0 to 559
#
# NOTE 1: do not add 1000000 to the address, MotoROS will do this when
#         necessary.
#
# NOTE 2: many programming languages will parse literals starting with '0' as
#         octal numbers. Do not add leading zeros to register addresses to avoid
#         specifying the wrong register to write to.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
uint16 value
---
string message
bool success

")

(setf (get motoman_msgs::WriteMRegisterResponse :md5sum-) "6b8bb3bb740ab645ad65c2abec91e614")
(setf (get motoman_msgs::WriteMRegisterResponse :datatype-) "motoman_msgs/WriteMRegisterResponse")
(setf (get motoman_msgs::WriteMRegisterResponse :definition-)
      "
# Write 'value' to the M register at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be written to:
#
#  - 0 to 559
#
# NOTE 1: do not add 1000000 to the address, MotoROS will do this when
#         necessary.
#
# NOTE 2: many programming languages will parse literals starting with '0' as
#         octal numbers. Do not add leading zeros to register addresses to avoid
#         specifying the wrong register to write to.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
uint16 value
---
string message
bool success

")



(provide :motoman_msgs/WriteMRegister "6b8bb3bb740ab645ad65c2abec91e614")


