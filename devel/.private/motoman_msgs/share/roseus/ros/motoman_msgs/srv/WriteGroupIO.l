;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::WriteGroupIO)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'WriteGroupIO (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::WRITEGROUPIO")
  (make-package "MOTOMAN_MSGS::WRITEGROUPIO"))
(unless (find-package "MOTOMAN_MSGS::WRITEGROUPIOREQUEST")
  (make-package "MOTOMAN_MSGS::WRITEGROUPIOREQUEST"))
(unless (find-package "MOTOMAN_MSGS::WRITEGROUPIORESPONSE")
  (make-package "MOTOMAN_MSGS::WRITEGROUPIORESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::WriteGroupIORequest
  :super ros::object
  :slots (_address _value ))

(defmethod motoman_msgs::WriteGroupIORequest
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
    ;; uint8 _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _address
       (write-long _address s)
     ;; uint8 _value
       (write-byte _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass motoman_msgs::WriteGroupIOResponse
  :super ros::object
  :slots (_message _success ))

(defmethod motoman_msgs::WriteGroupIOResponse
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

(defclass motoman_msgs::WriteGroupIO
  :super ros::object
  :slots ())

(setf (get motoman_msgs::WriteGroupIO :md5sum-) "442d77ce292ab13600e884d7860398a9")
(setf (get motoman_msgs::WriteGroupIO :datatype-) "motoman_msgs/WriteGroupIO")
(setf (get motoman_msgs::WriteGroupIO :request) motoman_msgs::WriteGroupIORequest)
(setf (get motoman_msgs::WriteGroupIO :response) motoman_msgs::WriteGroupIOResponse)

(defmethod motoman_msgs::WriteGroupIORequest
  (:response () (instance motoman_msgs::WriteGroupIOResponse :init)))

(setf (get motoman_msgs::WriteGroupIORequest :md5sum-) "442d77ce292ab13600e884d7860398a9")
(setf (get motoman_msgs::WriteGroupIORequest :datatype-) "motoman_msgs/WriteGroupIORequest")
(setf (get motoman_msgs::WriteGroupIORequest :definition-)
      "
# Write 'value' to the Group IO element at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be written to:
#
#  - 2701 and up : Network Inputs (2501 and up on DX100 and FS100)
#  - 1001 and up : Universal/General Outputs
#
# NOTE: many programming languages will parse literals starting with '0' as
#       octal numbers. Do not add leading zeros to group addresses to avoid
#       specifying the wrong address to write to.

# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
uint8 value
---
string message
bool success

")

(setf (get motoman_msgs::WriteGroupIOResponse :md5sum-) "442d77ce292ab13600e884d7860398a9")
(setf (get motoman_msgs::WriteGroupIOResponse :datatype-) "motoman_msgs/WriteGroupIOResponse")
(setf (get motoman_msgs::WriteGroupIOResponse :definition-)
      "
# Write 'value' to the Group IO element at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# Only the following addresses can be written to:
#
#  - 2701 and up : Network Inputs (2501 and up on DX100 and FS100)
#  - 1001 and up : Universal/General Outputs
#
# NOTE: many programming languages will parse literals starting with '0' as
#       octal numbers. Do not add leading zeros to group addresses to avoid
#       specifying the wrong address to write to.

# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
uint8 value
---
string message
bool success

")



(provide :motoman_msgs/WriteGroupIO "442d77ce292ab13600e884d7860398a9")


