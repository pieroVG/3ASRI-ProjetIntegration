;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::ReadSingleIO)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'ReadSingleIO (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::READSINGLEIO")
  (make-package "MOTOMAN_MSGS::READSINGLEIO"))
(unless (find-package "MOTOMAN_MSGS::READSINGLEIOREQUEST")
  (make-package "MOTOMAN_MSGS::READSINGLEIOREQUEST"))
(unless (find-package "MOTOMAN_MSGS::READSINGLEIORESPONSE")
  (make-package "MOTOMAN_MSGS::READSINGLEIORESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::ReadSingleIORequest
  :super ros::object
  :slots (_address ))

(defmethod motoman_msgs::ReadSingleIORequest
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

(defclass motoman_msgs::ReadSingleIOResponse
  :super ros::object
  :slots (_message _success _value ))

(defmethod motoman_msgs::ReadSingleIOResponse
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
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motoman_msgs::ReadSingleIO
  :super ros::object
  :slots ())

(setf (get motoman_msgs::ReadSingleIO :md5sum-) "46cc8843a2b05da36149122d04a523ad")
(setf (get motoman_msgs::ReadSingleIO :datatype-) "motoman_msgs/ReadSingleIO")
(setf (get motoman_msgs::ReadSingleIO :request) motoman_msgs::ReadSingleIORequest)
(setf (get motoman_msgs::ReadSingleIO :response) motoman_msgs::ReadSingleIOResponse)

(defmethod motoman_msgs::ReadSingleIORequest
  (:response () (instance motoman_msgs::ReadSingleIOResponse :init)))

(setf (get motoman_msgs::ReadSingleIORequest :md5sum-) "46cc8843a2b05da36149122d04a523ad")
(setf (get motoman_msgs::ReadSingleIORequest :datatype-) "motoman_msgs/ReadSingleIORequest")
(setf (get motoman_msgs::ReadSingleIORequest :definition-)
      "
# Read (and return) the current value of the IO element at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# There are no restrictions as to which IO elements can be read, but they have
# to exist on the controller and be configured correctly.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
---
string message
bool success
int32 value

")

(setf (get motoman_msgs::ReadSingleIOResponse :md5sum-) "46cc8843a2b05da36149122d04a523ad")
(setf (get motoman_msgs::ReadSingleIOResponse :datatype-) "motoman_msgs/ReadSingleIOResponse")
(setf (get motoman_msgs::ReadSingleIOResponse :definition-)
      "
# Read (and return) the current value of the IO element at 'address'.
#
# Addresses are plain, base-10 integers, as used and displayed by the controller
# (on the teach pendant for instance).
#
# There are no restrictions as to which IO elements can be read, but they have
# to exist on the controller and be configured correctly.
#
# Refer also the Yaskawa Motoman documentation on IO addressing and
# configuration.

uint32 address
---
string message
bool success
int32 value

")



(provide :motoman_msgs/ReadSingleIO "46cc8843a2b05da36149122d04a523ad")


