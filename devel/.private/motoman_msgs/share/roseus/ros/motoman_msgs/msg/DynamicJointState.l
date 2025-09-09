;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::DynamicJointState)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'DynamicJointState (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::DYNAMICJOINTSTATE")
  (make-package "MOTOMAN_MSGS::DYNAMICJOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude DynamicJointState.msg.html


(defclass motoman_msgs::DynamicJointState
  :super ros::object
  :slots (_group_number _num_joints _valid_fields _positions _velocities _accelerations _effort _positions_desired _positions_errors _velocities_desired _velocities_errors _accelerations_desired _accelerations_errors _effort_errors _effort_desired ))

(defmethod motoman_msgs::DynamicJointState
  (:init
   (&key
    ((:group_number __group_number) 0)
    ((:num_joints __num_joints) 0)
    ((:valid_fields __valid_fields) 0)
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:accelerations __accelerations) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort __effort) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:positions_desired __positions_desired) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:positions_errors __positions_errors) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities_desired __velocities_desired) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities_errors __velocities_errors) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:accelerations_desired __accelerations_desired) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:accelerations_errors __accelerations_errors) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort_errors __effort_errors) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort_desired __effort_desired) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _group_number (round __group_number))
   (setq _num_joints (round __num_joints))
   (setq _valid_fields (round __valid_fields))
   (setq _positions __positions)
   (setq _velocities __velocities)
   (setq _accelerations __accelerations)
   (setq _effort __effort)
   (setq _positions_desired __positions_desired)
   (setq _positions_errors __positions_errors)
   (setq _velocities_desired __velocities_desired)
   (setq _velocities_errors __velocities_errors)
   (setq _accelerations_desired __accelerations_desired)
   (setq _accelerations_errors __accelerations_errors)
   (setq _effort_errors __effort_errors)
   (setq _effort_desired __effort_desired)
   self)
  (:group_number
   (&optional __group_number)
   (if __group_number (setq _group_number __group_number)) _group_number)
  (:num_joints
   (&optional __num_joints)
   (if __num_joints (setq _num_joints __num_joints)) _num_joints)
  (:valid_fields
   (&optional __valid_fields)
   (if __valid_fields (setq _valid_fields __valid_fields)) _valid_fields)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:accelerations
   (&optional __accelerations)
   (if __accelerations (setq _accelerations __accelerations)) _accelerations)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:positions_desired
   (&optional __positions_desired)
   (if __positions_desired (setq _positions_desired __positions_desired)) _positions_desired)
  (:positions_errors
   (&optional __positions_errors)
   (if __positions_errors (setq _positions_errors __positions_errors)) _positions_errors)
  (:velocities_desired
   (&optional __velocities_desired)
   (if __velocities_desired (setq _velocities_desired __velocities_desired)) _velocities_desired)
  (:velocities_errors
   (&optional __velocities_errors)
   (if __velocities_errors (setq _velocities_errors __velocities_errors)) _velocities_errors)
  (:accelerations_desired
   (&optional __accelerations_desired)
   (if __accelerations_desired (setq _accelerations_desired __accelerations_desired)) _accelerations_desired)
  (:accelerations_errors
   (&optional __accelerations_errors)
   (if __accelerations_errors (setq _accelerations_errors __accelerations_errors)) _accelerations_errors)
  (:effort_errors
   (&optional __effort_errors)
   (if __effort_errors (setq _effort_errors __effort_errors)) _effort_errors)
  (:effort_desired
   (&optional __effort_desired)
   (if __effort_desired (setq _effort_desired __effort_desired)) _effort_desired)
  (:serialization-length
   ()
   (+
    ;; int16 _group_number
    2
    ;; int16 _num_joints
    2
    ;; int16 _valid_fields
    2
    ;; float64[] _positions
    (* 8    (length _positions)) 4
    ;; float64[] _velocities
    (* 8    (length _velocities)) 4
    ;; float64[] _accelerations
    (* 8    (length _accelerations)) 4
    ;; float64[] _effort
    (* 8    (length _effort)) 4
    ;; float64[] _positions_desired
    (* 8    (length _positions_desired)) 4
    ;; float64[] _positions_errors
    (* 8    (length _positions_errors)) 4
    ;; float64[] _velocities_desired
    (* 8    (length _velocities_desired)) 4
    ;; float64[] _velocities_errors
    (* 8    (length _velocities_errors)) 4
    ;; float64[] _accelerations_desired
    (* 8    (length _accelerations_desired)) 4
    ;; float64[] _accelerations_errors
    (* 8    (length _accelerations_errors)) 4
    ;; float64[] _effort_errors
    (* 8    (length _effort_errors)) 4
    ;; float64[] _effort_desired
    (* 8    (length _effort_desired)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _group_number
       (write-word _group_number s)
     ;; int16 _num_joints
       (write-word _num_joints s)
     ;; int16 _valid_fields
       (write-word _valid_fields s)
     ;; float64[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (sys::poke (elt _positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _accelerations
     (write-long (length _accelerations) s)
     (dotimes (i (length _accelerations))
       (sys::poke (elt _accelerations i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort
     (write-long (length _effort) s)
     (dotimes (i (length _effort))
       (sys::poke (elt _effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _positions_desired
     (write-long (length _positions_desired) s)
     (dotimes (i (length _positions_desired))
       (sys::poke (elt _positions_desired i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _positions_errors
     (write-long (length _positions_errors) s)
     (dotimes (i (length _positions_errors))
       (sys::poke (elt _positions_errors i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities_desired
     (write-long (length _velocities_desired) s)
     (dotimes (i (length _velocities_desired))
       (sys::poke (elt _velocities_desired i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities_errors
     (write-long (length _velocities_errors) s)
     (dotimes (i (length _velocities_errors))
       (sys::poke (elt _velocities_errors i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _accelerations_desired
     (write-long (length _accelerations_desired) s)
     (dotimes (i (length _accelerations_desired))
       (sys::poke (elt _accelerations_desired i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _accelerations_errors
     (write-long (length _accelerations_errors) s)
     (dotimes (i (length _accelerations_errors))
       (sys::poke (elt _accelerations_errors i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort_errors
     (write-long (length _effort_errors) s)
     (dotimes (i (length _effort_errors))
       (sys::poke (elt _effort_errors i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort_desired
     (write-long (length _effort_desired) s)
     (dotimes (i (length _effort_desired))
       (sys::poke (elt _effort_desired i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _group_number
     (setq _group_number (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _num_joints
     (setq _num_joints (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _valid_fields
     (setq _valid_fields (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _accelerations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accelerations (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _accelerations i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _positions_desired
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions_desired (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions_desired i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _positions_errors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions_errors (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions_errors i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities_desired
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities_desired (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities_desired i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities_errors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities_errors (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities_errors i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _accelerations_desired
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accelerations_desired (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _accelerations_desired i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _accelerations_errors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accelerations_errors (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _accelerations_errors i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort_errors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort_errors (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort_errors i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort_desired
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort_desired (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort_desired i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get motoman_msgs::DynamicJointState :md5sum-) "c44649b8de969b98f15adea419fa49a4")
(setf (get motoman_msgs::DynamicJointState :datatype-) "motoman_msgs/DynamicJointState")
(setf (get motoman_msgs::DynamicJointState :definition-)
      "#group[]: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    position_desired[]
#    position_errors[]
#    velocity_desired[]
#    velocity_errors[]
#    effort_desired[]
#    effort_error[]

int16 group_number
int16 num_joints
int16 valid_fields
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
float64[] positions_desired
float64[] positions_errors
float64[] velocities_desired
float64[] velocities_errors
float64[] accelerations_desired
float64[] accelerations_errors
float64[] effort_errors
float64[] effort_desired

")



(provide :motoman_msgs/DynamicJointState "c44649b8de969b98f15adea419fa49a4")


