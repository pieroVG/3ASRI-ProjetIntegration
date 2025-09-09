;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::DynamicJointsGroup)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'DynamicJointsGroup (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::DYNAMICJOINTSGROUP")
  (make-package "MOTOMAN_MSGS::DYNAMICJOINTSGROUP"))

(in-package "ROS")
;;//! \htmlinclude DynamicJointsGroup.msg.html


(defclass motoman_msgs::DynamicJointsGroup
  :super ros::object
  :slots (_group_number _num_joints _valid_fields _positions _velocities _accelerations _effort _time_from_start ))

(defmethod motoman_msgs::DynamicJointsGroup
  (:init
   (&key
    ((:group_number __group_number) 0)
    ((:num_joints __num_joints) 0)
    ((:valid_fields __valid_fields) 0)
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:accelerations __accelerations) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort __effort) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time_from_start __time_from_start) (instance ros::time :init))
    )
   (send-super :init)
   (setq _group_number (round __group_number))
   (setq _num_joints (round __num_joints))
   (setq _valid_fields (round __valid_fields))
   (setq _positions __positions)
   (setq _velocities __velocities)
   (setq _accelerations __accelerations)
   (setq _effort __effort)
   (setq _time_from_start __time_from_start)
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
  (:time_from_start
   (&optional __time_from_start)
   (if __time_from_start (setq _time_from_start __time_from_start)) _time_from_start)
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
    ;; duration _time_from_start
    8
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
     ;; duration _time_from_start
       (write-long (send _time_from_start :sec) s) (write-long (send _time_from_start :nsec) s)
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
   ;; duration _time_from_start
     (send _time_from_start :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time_from_start :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motoman_msgs::DynamicJointsGroup :md5sum-) "fb56acaba1b90a9d7640af3e785681ca")
(setf (get motoman_msgs::DynamicJointsGroup :datatype-) "motoman_msgs/DynamicJointsGroup")
(setf (get motoman_msgs::DynamicJointsGroup :definition-)
      "# DynamicJointsGroup
#group: # length of this array must match num_groups
#    id:   control-group ID for use on-controller
#    num_joints: # of joints in this motion group
#    valid_fields: #bit field for following items
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
#    positions[]
#    velocities[]
#    accelerations[]
#    effort[]
#    time_from_start


int16 group_number
int16 num_joints
int16 valid_fields
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start

")



(provide :motoman_msgs/DynamicJointsGroup "fb56acaba1b90a9d7640af3e785681ca")


