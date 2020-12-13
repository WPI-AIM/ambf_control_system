; Auto-generated. Do not edit!


(cl:in-package controller_modules-srv)


;//! \htmlinclude JointControl-request.msg.html

(cl:defclass <JointControl-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (joint_names
    :reader joint_names
    :initarg :joint_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (controller_name
    :reader controller_name
    :initarg :controller_name
    :type cl:string
    :initform "")
   (desired
    :reader desired
    :initarg :desired
    :type trajectory_msgs-msg:JointTrajectoryPoint
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint))
   (actual
    :reader actual
    :initarg :actual
    :type trajectory_msgs-msg:JointTrajectoryPoint
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint)))
)

(cl:defclass JointControl-request (<JointControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller_modules-srv:<JointControl-request> is deprecated: use controller_modules-srv:JointControl-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JointControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:header-val is deprecated.  Use controller_modules-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_names-val :lambda-list '(m))
(cl:defmethod joint_names-val ((m <JointControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:joint_names-val is deprecated.  Use controller_modules-srv:joint_names instead.")
  (joint_names m))

(cl:ensure-generic-function 'controller_name-val :lambda-list '(m))
(cl:defmethod controller_name-val ((m <JointControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:controller_name-val is deprecated.  Use controller_modules-srv:controller_name instead.")
  (controller_name m))

(cl:ensure-generic-function 'desired-val :lambda-list '(m))
(cl:defmethod desired-val ((m <JointControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:desired-val is deprecated.  Use controller_modules-srv:desired instead.")
  (desired m))

(cl:ensure-generic-function 'actual-val :lambda-list '(m))
(cl:defmethod actual-val ((m <JointControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:actual-val is deprecated.  Use controller_modules-srv:actual instead.")
  (actual m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointControl-request>) ostream)
  "Serializes a message object of type '<JointControl-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_names))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'actual) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointControl-request>) istream)
  "Deserializes a message object of type '<JointControl-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'actual) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointControl-request>)))
  "Returns string type for a service object of type '<JointControl-request>"
  "controller_modules/JointControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointControl-request)))
  "Returns string type for a service object of type 'JointControl-request"
  "controller_modules/JointControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointControl-request>)))
  "Returns md5sum for a message object of type '<JointControl-request>"
  "48f8bb00f33b7039bd35678b2c6204e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointControl-request)))
  "Returns md5sum for a message object of type 'JointControl-request"
  "48f8bb00f33b7039bd35678b2c6204e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointControl-request>)))
  "Returns full string definition for message of type '<JointControl-request>"
  (cl:format cl:nil "Header header~%string[] joint_names~%string controller_name~%trajectory_msgs/JointTrajectoryPoint desired~%trajectory_msgs/JointTrajectoryPoint actual~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointControl-request)))
  "Returns full string definition for message of type 'JointControl-request"
  (cl:format cl:nil "Header header~%string[] joint_names~%string controller_name~%trajectory_msgs/JointTrajectoryPoint desired~%trajectory_msgs/JointTrajectoryPoint actual~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointControl-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'controller_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'actual))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'JointControl-request
    (cl:cons ':header (header msg))
    (cl:cons ':joint_names (joint_names msg))
    (cl:cons ':controller_name (controller_name msg))
    (cl:cons ':desired (desired msg))
    (cl:cons ':actual (actual msg))
))
;//! \htmlinclude JointControl-response.msg.html

(cl:defclass <JointControl-response> (roslisp-msg-protocol:ros-message)
  ((control_output
    :reader control_output
    :initarg :control_output
    :type trajectory_msgs-msg:JointTrajectoryPoint
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint)))
)

(cl:defclass JointControl-response (<JointControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller_modules-srv:<JointControl-response> is deprecated: use controller_modules-srv:JointControl-response instead.")))

(cl:ensure-generic-function 'control_output-val :lambda-list '(m))
(cl:defmethod control_output-val ((m <JointControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:control_output-val is deprecated.  Use controller_modules-srv:control_output instead.")
  (control_output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointControl-response>) ostream)
  "Serializes a message object of type '<JointControl-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'control_output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointControl-response>) istream)
  "Deserializes a message object of type '<JointControl-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'control_output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointControl-response>)))
  "Returns string type for a service object of type '<JointControl-response>"
  "controller_modules/JointControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointControl-response)))
  "Returns string type for a service object of type 'JointControl-response"
  "controller_modules/JointControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointControl-response>)))
  "Returns md5sum for a message object of type '<JointControl-response>"
  "48f8bb00f33b7039bd35678b2c6204e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointControl-response)))
  "Returns md5sum for a message object of type 'JointControl-response"
  "48f8bb00f33b7039bd35678b2c6204e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointControl-response>)))
  "Returns full string definition for message of type '<JointControl-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectoryPoint control_output~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointControl-response)))
  "Returns full string definition for message of type 'JointControl-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectoryPoint control_output~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointControl-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'control_output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'JointControl-response
    (cl:cons ':control_output (control_output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'JointControl)))
  'JointControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'JointControl)))
  'JointControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointControl)))
  "Returns string type for a service object of type '<JointControl>"
  "controller_modules/JointControl")