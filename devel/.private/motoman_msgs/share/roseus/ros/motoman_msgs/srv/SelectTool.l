;; Auto-generated. Do not edit!


(when (boundp 'motoman_msgs::SelectTool)
  (if (not (find-package "MOTOMAN_MSGS"))
    (make-package "MOTOMAN_MSGS"))
  (shadow 'SelectTool (find-package "MOTOMAN_MSGS")))
(unless (find-package "MOTOMAN_MSGS::SELECTTOOL")
  (make-package "MOTOMAN_MSGS::SELECTTOOL"))
(unless (find-package "MOTOMAN_MSGS::SELECTTOOLREQUEST")
  (make-package "MOTOMAN_MSGS::SELECTTOOLREQUEST"))
(unless (find-package "MOTOMAN_MSGS::SELECTTOOLRESPONSE")
  (make-package "MOTOMAN_MSGS::SELECTTOOLRESPONSE"))

(in-package "ROS")





(defclass motoman_msgs::SelectToolRequest
  :super ros::object
  :slots (_group_number _tool_number ))

(defmethod motoman_msgs::SelectToolRequest
  (:init
   (&key
    ((:group_number __group_number) 0)
    ((:tool_number __tool_number) 0)
    )
   (send-super :init)
   (setq _group_number (round __group_number))
   (setq _tool_number (round __tool_number))
   self)
  (:group_number
   (&optional __group_number)
   (if __group_number (setq _group_number __group_number)) _group_number)
  (:tool_number
   (&optional __tool_number)
   (if __tool_number (setq _tool_number __tool_number)) _tool_number)
  (:serialization-length
   ()
   (+
    ;; uint32 _group_number
    4
    ;; uint32 _tool_number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _group_number
       (write-long _group_number s)
     ;; uint32 _tool_number
       (write-long _tool_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _group_number
     (setq _group_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _tool_number
     (setq _tool_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motoman_msgs::SelectToolResponse
  :super ros::object
  :slots (_message _success ))

(defmethod motoman_msgs::SelectToolResponse
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

(defclass motoman_msgs::SelectTool
  :super ros::object
  :slots ())

(setf (get motoman_msgs::SelectTool :md5sum-) "afb2fb884fcd9c28c81c82056d543156")
(setf (get motoman_msgs::SelectTool :datatype-) "motoman_msgs/SelectTool")
(setf (get motoman_msgs::SelectTool :request) motoman_msgs::SelectToolRequest)
(setf (get motoman_msgs::SelectTool :response) motoman_msgs::SelectToolResponse)

(defmethod motoman_msgs::SelectToolRequest
  (:response () (instance motoman_msgs::SelectToolResponse :init)))

(setf (get motoman_msgs::SelectToolRequest :md5sum-) "afb2fb884fcd9c28c81c82056d543156")
(setf (get motoman_msgs::SelectToolRequest :datatype-) "motoman_msgs/SelectToolRequest")
(setf (get motoman_msgs::SelectToolRequest :definition-)
      "
# Change the active tool file on the controller.
#
# This changes the tool definition used for both (Moto)ROS-controlled motions
# and manual jogging.
#
#
# ## Controller support
#
# This service wraps two distinct (but related) actions:
#
#  1. changing the tool file used for execution of (Moto)ROS-controlled motions
#  2. changing the tool file used for jogging
#
# Action 1 is supported by all controllers supported by MotoROS (ie: DX100,
# FS100, DX200 and YRC1000(u)).
#
# Action 2 is NOT supported on FS100 controllers. This is a limitation of
# MotoPlus, not of MotoROS.
#
# In all cases, make sure to read the following sections carefully to be aware
# of the behaviour of this service and of the Yaskawa controller after calling
# this service.
#
#
# ## FSU and PFL
#
# Tool switches through this service will be taken into account by the FSU and
# PFL safety systems if the controller has these options and they are enabled.
#
# Tool interference files, if associated with certain tool file definitions,
# will therefore also switch.
#
#
# ## Tool switch timing
#
# The active tool will be switched AFTER the controller has completed execution
# of the last trajectory segment that was sent to MotoROS BEFORE this service
# was called.
#
# The motion queue internal to MotoROS caches a number of segments in a FIFO.
# Only segments received AFTER this service was invoked will be executed with
# the new tool. Any segments received before a tool switch was invoked will use
# the old tool.
#
# To ensure motion will be executed using a certain tool, applications should
# check the 'in_motion' field (part of the RobotStatus message published on the
# 'robot_status' topic by the driver) and invoke the service when the robot has
# come to a stop (and the motion queue of MotoROS is empty). Any subsequent
# trajectories will use the new tool.
#
#
# ## Effect on trajectory execution
#
# As MotoROS currently only executes joint space trajectories, changing tool
# file with this service DOES NOT affect the execution of those trajectories.
#
# As noted earlier though, the active tool file definition will affect FSU and
# PFL behaviour, as the tool definition is used in calculation of dynamics and
# safety (see \"FSU and PFL\" above).
#
# To clarify: the TCP definition of the tool file is NOT taken into account when
# calculating manipulator motion based on incoming ROS JointTrajectoryAction
# goals (as JointTrajectory goals do not include any Cartesian poses, only
# absolute joint space coordinates for each axis).
#
# Instead, ROS applications should use different TF frames to define tool frames
# on the ROS side and plan their motions with the appropriate TF frame as the
# active tool.
#
# This service could then be used to notify the controller of other tool
# characteristics, such as weight, CoG and inertia by switching to a tool file
# configured with those parameters.
#
#
# ## Retrieving the active tool file
#
# MotoROS does not currently support retrieving the active tool file.
#
#
#
# For more information about tool file configuration, selection and use on
# Yaskawa controllers, refer to the relevant Yaskawa Motoman documentation.


# Numeric ID of the group the tool file is defined for.
#
# This MUST correspond to a group ID currently defined and active on the
# controller. The ROS service does not check whether the value specified here
# is legal. The MotoROS server will however check this, and will refuse to
# switch to an illegal value and will report this to the ROS driver.
#
# Callers will be notified of such failures by 'success' being set to 'false'
# and an appropriate error message being provided via the 'message' field of
# the service response.
#
# NOTE: this field is 0-based, with 0 corresponding to the first motion group,
#       1 the second, etc.
#
# legal-values: [0, total_nr_of_groups)
# required: yes (absence-causes-service-failure)
# default: no-default
uint32 group_number

# Numeric ID of the tool file to switch to for the specified group.
#
# Identical to 'group_number', specification of illegal values will result
# in failure to invoke the MotoROS service, and an unsuccessful service result
# being returned.
#
# NOTE: this field is 0-based, with 0 corresponding to the first tool file,
#       1 the second, etc.
#
# legal-values: [0, 63]
# required: yes (absence-causes-service-failure)
# default: no-default
uint32 tool_number

# TODO: might want to expose 'sequence_number' here as well

---

# A human-readable error message, or an empty string if there was no error.
string message

# true IFF invocation of the MotoROS service was successful.
#
# NOTE: this is independent of whether the ROS service invocation was
#       successful. In absence of any ROS middleware failures, a failed MotoROS
#       service invocation will result in 'success' here being set to 'false',
#       but a successful ROS service invocation.
bool success

")

(setf (get motoman_msgs::SelectToolResponse :md5sum-) "afb2fb884fcd9c28c81c82056d543156")
(setf (get motoman_msgs::SelectToolResponse :datatype-) "motoman_msgs/SelectToolResponse")
(setf (get motoman_msgs::SelectToolResponse :definition-)
      "
# Change the active tool file on the controller.
#
# This changes the tool definition used for both (Moto)ROS-controlled motions
# and manual jogging.
#
#
# ## Controller support
#
# This service wraps two distinct (but related) actions:
#
#  1. changing the tool file used for execution of (Moto)ROS-controlled motions
#  2. changing the tool file used for jogging
#
# Action 1 is supported by all controllers supported by MotoROS (ie: DX100,
# FS100, DX200 and YRC1000(u)).
#
# Action 2 is NOT supported on FS100 controllers. This is a limitation of
# MotoPlus, not of MotoROS.
#
# In all cases, make sure to read the following sections carefully to be aware
# of the behaviour of this service and of the Yaskawa controller after calling
# this service.
#
#
# ## FSU and PFL
#
# Tool switches through this service will be taken into account by the FSU and
# PFL safety systems if the controller has these options and they are enabled.
#
# Tool interference files, if associated with certain tool file definitions,
# will therefore also switch.
#
#
# ## Tool switch timing
#
# The active tool will be switched AFTER the controller has completed execution
# of the last trajectory segment that was sent to MotoROS BEFORE this service
# was called.
#
# The motion queue internal to MotoROS caches a number of segments in a FIFO.
# Only segments received AFTER this service was invoked will be executed with
# the new tool. Any segments received before a tool switch was invoked will use
# the old tool.
#
# To ensure motion will be executed using a certain tool, applications should
# check the 'in_motion' field (part of the RobotStatus message published on the
# 'robot_status' topic by the driver) and invoke the service when the robot has
# come to a stop (and the motion queue of MotoROS is empty). Any subsequent
# trajectories will use the new tool.
#
#
# ## Effect on trajectory execution
#
# As MotoROS currently only executes joint space trajectories, changing tool
# file with this service DOES NOT affect the execution of those trajectories.
#
# As noted earlier though, the active tool file definition will affect FSU and
# PFL behaviour, as the tool definition is used in calculation of dynamics and
# safety (see \"FSU and PFL\" above).
#
# To clarify: the TCP definition of the tool file is NOT taken into account when
# calculating manipulator motion based on incoming ROS JointTrajectoryAction
# goals (as JointTrajectory goals do not include any Cartesian poses, only
# absolute joint space coordinates for each axis).
#
# Instead, ROS applications should use different TF frames to define tool frames
# on the ROS side and plan their motions with the appropriate TF frame as the
# active tool.
#
# This service could then be used to notify the controller of other tool
# characteristics, such as weight, CoG and inertia by switching to a tool file
# configured with those parameters.
#
#
# ## Retrieving the active tool file
#
# MotoROS does not currently support retrieving the active tool file.
#
#
#
# For more information about tool file configuration, selection and use on
# Yaskawa controllers, refer to the relevant Yaskawa Motoman documentation.


# Numeric ID of the group the tool file is defined for.
#
# This MUST correspond to a group ID currently defined and active on the
# controller. The ROS service does not check whether the value specified here
# is legal. The MotoROS server will however check this, and will refuse to
# switch to an illegal value and will report this to the ROS driver.
#
# Callers will be notified of such failures by 'success' being set to 'false'
# and an appropriate error message being provided via the 'message' field of
# the service response.
#
# NOTE: this field is 0-based, with 0 corresponding to the first motion group,
#       1 the second, etc.
#
# legal-values: [0, total_nr_of_groups)
# required: yes (absence-causes-service-failure)
# default: no-default
uint32 group_number

# Numeric ID of the tool file to switch to for the specified group.
#
# Identical to 'group_number', specification of illegal values will result
# in failure to invoke the MotoROS service, and an unsuccessful service result
# being returned.
#
# NOTE: this field is 0-based, with 0 corresponding to the first tool file,
#       1 the second, etc.
#
# legal-values: [0, 63]
# required: yes (absence-causes-service-failure)
# default: no-default
uint32 tool_number

# TODO: might want to expose 'sequence_number' here as well

---

# A human-readable error message, or an empty string if there was no error.
string message

# true IFF invocation of the MotoROS service was successful.
#
# NOTE: this is independent of whether the ROS service invocation was
#       successful. In absence of any ROS middleware failures, a failed MotoROS
#       service invocation will result in 'success' here being set to 'false',
#       but a successful ROS service invocation.
bool success

")



(provide :motoman_msgs/SelectTool "afb2fb884fcd9c28c81c82056d543156")


