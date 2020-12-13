;; Auto-generated. Do not edit!


(when (boundp 'controller_modules::JointControl)
  (if (not (find-package "CONTROLLER_MODULES"))
    (make-package "CONTROLLER_MODULES"))
  (shadow 'JointControl (find-package "CONTROLLER_MODULES")))
(unless (find-package "CONTROLLER_MODULES::JOINTCONTROL")
  (make-package "CONTROLLER_MODULES::JOINTCONTROL"))
(unless (find-package "CONTROLLER_MODULES::JOINTCONTROLREQUEST")
  (make-package "CONTROLLER_MODULES::JOINTCONTROLREQUEST"))
(unless (find-package "CONTROLLER_MODULES::JOINTCONTROLRESPONSE")
  (make-package "CONTROLLER_MODULES::JOINTCONTROLRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))
(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass controller_modules::JointControlRequest
  :super ros::object
  :slots (_header _joint_names _controller_name _desired _actual ))

(defmethod controller_modules::JointControlRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:controller_name __controller_name) "")
    ((:desired __desired) (instance trajectory_msgs::JointTrajectoryPoint :init))
    ((:actual __actual) (instance trajectory_msgs::JointTrajectoryPoint :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_names __joint_names)
   (setq _controller_name (string __controller_name))
   (setq _desired __desired)
   (setq _actual __actual)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_names
   (&optional __joint_names)
   (if __joint_names (setq _joint_names __joint_names)) _joint_names)
  (:controller_name
   (&optional __controller_name)
   (if __controller_name (setq _controller_name __controller_name)) _controller_name)
  (:desired
   (&rest __desired)
   (if (keywordp (car __desired))
       (send* _desired __desired)
     (progn
       (if __desired (setq _desired (car __desired)))
       _desired)))
  (:actual
   (&rest __actual)
   (if (keywordp (car __actual))
       (send* _actual __actual)
     (progn
       (if __actual (setq _actual (car __actual)))
       _actual)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; string _controller_name
    4 (length _controller_name)
    ;; trajectory_msgs/JointTrajectoryPoint _desired
    (send _desired :serialization-length)
    ;; trajectory_msgs/JointTrajectoryPoint _actual
    (send _actual :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _joint_names
     (write-long (length _joint_names) s)
     (dolist (elem _joint_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _controller_name
       (write-long (length _controller_name) s) (princ _controller_name s)
     ;; trajectory_msgs/JointTrajectoryPoint _desired
       (send _desired :serialize s)
     ;; trajectory_msgs/JointTrajectoryPoint _actual
       (send _actual :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _joint_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _controller_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _controller_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; trajectory_msgs/JointTrajectoryPoint _desired
     (send _desired :deserialize buf ptr-) (incf ptr- (send _desired :serialization-length))
   ;; trajectory_msgs/JointTrajectoryPoint _actual
     (send _actual :deserialize buf ptr-) (incf ptr- (send _actual :serialization-length))
   ;;
   self)
  )

(defclass controller_modules::JointControlResponse
  :super ros::object
  :slots (_control_output ))

(defmethod controller_modules::JointControlResponse
  (:init
   (&key
    ((:control_output __control_output) (instance trajectory_msgs::JointTrajectoryPoint :init))
    )
   (send-super :init)
   (setq _control_output __control_output)
   self)
  (:control_output
   (&rest __control_output)
   (if (keywordp (car __control_output))
       (send* _control_output __control_output)
     (progn
       (if __control_output (setq _control_output (car __control_output)))
       _control_output)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectoryPoint _control_output
    (send _control_output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectoryPoint _control_output
       (send _control_output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectoryPoint _control_output
     (send _control_output :deserialize buf ptr-) (incf ptr- (send _control_output :serialization-length))
   ;;
   self)
  )

(defclass controller_modules::JointControl
  :super ros::object
  :slots ())

(setf (get controller_modules::JointControl :md5sum-) "48f8bb00f33b7039bd35678b2c6204e5")
(setf (get controller_modules::JointControl :datatype-) "controller_modules/JointControl")
(setf (get controller_modules::JointControl :request) controller_modules::JointControlRequest)
(setf (get controller_modules::JointControl :response) controller_modules::JointControlResponse)

(defmethod controller_modules::JointControlRequest
  (:response () (instance controller_modules::JointControlResponse :init)))

(setf (get controller_modules::JointControlRequest :md5sum-) "48f8bb00f33b7039bd35678b2c6204e5")
(setf (get controller_modules::JointControlRequest :datatype-) "controller_modules/JointControlRequest")
(setf (get controller_modules::JointControlRequest :definition-)
      "Header header
string[] joint_names
string controller_name
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
trajectory_msgs/JointTrajectoryPoint control_output

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")

(setf (get controller_modules::JointControlResponse :md5sum-) "48f8bb00f33b7039bd35678b2c6204e5")
(setf (get controller_modules::JointControlResponse :datatype-) "controller_modules/JointControlResponse")
(setf (get controller_modules::JointControlResponse :definition-)
      "Header header
string[] joint_names
string controller_name
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
trajectory_msgs/JointTrajectoryPoint control_output

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")



(provide :controller_modules/JointControl "48f8bb00f33b7039bd35678b2c6204e5")


