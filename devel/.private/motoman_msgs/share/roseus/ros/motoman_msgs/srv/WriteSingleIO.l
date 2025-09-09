;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::WriteSingleIO)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'WriteSingleIO (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::WRITESINGLEIO")
  (make-package "MOTOMAN_MSGS::WRITESINGLEIO"))
(unless (find-package "MOTOMAN_MSGS::WRITESINGLEIOREQUEST")
  (make-package "MOTOMAN_MSGS::WRITESINGLEIOREQUEST"))
(unless (find-package "MOTOMAN_MSGS::WRITESINGLEIORESPONSE")
  (make-package "MOTOMAN_MSGS::WRITESINGLEIORESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::WriteSingleIORequest
  :super ros::object
  :slots (_address _value ))

(defmethod motoman_msgs::WriteSingleIORequest
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
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _address
       (write-long _address s)
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motoman_msgs::WriteSingleIOResponse
  :super ros::object
  :slots (_message _success ))

(defmethod motoman_msgs::WriteSingleIOResponse
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

(defclass motoman_msgs::WriteSingleIO
  :super ros::object
  :slots ())

(setf (get motoman_msgs::WriteSingleIO :md5sum-) "615ffdbac96f4fdce6e76dce79d5f3b5")
(setf (get motoman_msgs::WriteSingleIO :datatype-) "motoman_msgs/WriteSingleIO")
(setf (get motoman_msgs::WriteSingleIO :request) motoman_msgs::WriteSingleIORequest)
(setf (get motoman_msgs::WriteSingleIO :response) motoman_msgs::WriteSingleIOResponse)

(defmethod motoman_msgs::WriteSingleIORequest
  (:response () (instance motoman_msgs::WriteSingleIOResponse :init)))

(setf (get motoman_msgs::WriteSingleIORequest :md5sum-) "615ffdbac96f4fdce6e76dce79d5f3b5")
(setf (get motoman_msgs::WriteSingleIORequest :datatype-) "motoman_msgs/WriteSingleIORequest")
(setf (get motoman_msgs::WriteSingleIORequest :definition-)
      "
# Write 'value' to the IO element at 'address'.
#
# This service does not return anything.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be written to:
#
#  - 27010 and up : Network Inputs (25010 and up on DX100 and FS100)
#  - 10010 and up : Universal/General Outputs
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
int32 value
---
string message
bool success

")

(setf (get motoman_msgs::WriteSingleIOResponse :md5sum-) "615ffdbac96f4fdce6e76dce79d5f3b5")
(setf (get motoman_msgs::WriteSingleIOResponse :datatype-) "motoman_msgs/WriteSingleIOResponse")
(setf (get motoman_msgs::WriteSingleIOResponse :definition-)
      "
# Write 'value' to the IO element at 'address'.
#
# This service does not return anything.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be written to:
#
#  - 27010 and up : Network Inputs (25010 and up on DX100 and FS100)
#  - 10010 and up : Universal/General Outputs
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
int32 value
---
string message
bool success

")



(provide :motoman_msgs/WriteSingleIO "615ffdbac96f4fdce6e76dce79d5f3b5")


