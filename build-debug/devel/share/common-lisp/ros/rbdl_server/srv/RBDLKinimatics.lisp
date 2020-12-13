; Auto-generated. Do not edit!


(cl:in-package rbdl_server-srv)


;//! \htmlinclude RBDLKinimatics-request.msg.html

(cl:defclass <RBDLKinimatics-request> (roslisp-msg-protocol:ros-message)
  ((q
    :reader q
    :initarg :q
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RBDLKinimatics-request (<RBDLKinimatics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLKinimatics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLKinimatics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLKinimatics-request> is deprecated: use rbdl_server-srv:RBDLKinimatics-request instead.")))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <RBDLKinimatics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:q-val is deprecated.  Use rbdl_server-srv:q instead.")
  (q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLKinimatics-request>) ostream)
  "Serializes a message object of type '<RBDLKinimatics-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'q))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLKinimatics-request>) istream)
  "Deserializes a message object of type '<RBDLKinimatics-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLKinimatics-request>)))
  "Returns string type for a service object of type '<RBDLKinimatics-request>"
  "rbdl_server/RBDLKinimaticsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLKinimatics-request)))
  "Returns string type for a service object of type 'RBDLKinimatics-request"
  "rbdl_server/RBDLKinimaticsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLKinimatics-request>)))
  "Returns md5sum for a message object of type '<RBDLKinimatics-request>"
  "9ffe8145497585ad9a8e683635ee6416")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLKinimatics-request)))
  "Returns md5sum for a message object of type 'RBDLKinimatics-request"
  "9ffe8145497585ad9a8e683635ee6416")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLKinimatics-request>)))
  "Returns full string definition for message of type '<RBDLKinimatics-request>"
  (cl:format cl:nil "float64[] q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLKinimatics-request)))
  "Returns full string definition for message of type 'RBDLKinimatics-request"
  (cl:format cl:nil "float64[] q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLKinimatics-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLKinimatics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLKinimatics-request
    (cl:cons ':q (q msg))
))
;//! \htmlinclude RBDLKinimatics-response.msg.html

(cl:defclass <RBDLKinimatics-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass RBDLKinimatics-response (<RBDLKinimatics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLKinimatics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLKinimatics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLKinimatics-response> is deprecated: use rbdl_server-srv:RBDLKinimatics-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <RBDLKinimatics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:names-val is deprecated.  Use rbdl_server-srv:names instead.")
  (names m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <RBDLKinimatics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:points-val is deprecated.  Use rbdl_server-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLKinimatics-response>) ostream)
  "Serializes a message object of type '<RBDLKinimatics-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
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
   (cl:slot-value msg 'names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLKinimatics-response>) istream)
  "Deserializes a message object of type '<RBDLKinimatics-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLKinimatics-response>)))
  "Returns string type for a service object of type '<RBDLKinimatics-response>"
  "rbdl_server/RBDLKinimaticsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLKinimatics-response)))
  "Returns string type for a service object of type 'RBDLKinimatics-response"
  "rbdl_server/RBDLKinimaticsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLKinimatics-response>)))
  "Returns md5sum for a message object of type '<RBDLKinimatics-response>"
  "9ffe8145497585ad9a8e683635ee6416")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLKinimatics-response)))
  "Returns md5sum for a message object of type 'RBDLKinimatics-response"
  "9ffe8145497585ad9a8e683635ee6416")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLKinimatics-response>)))
  "Returns full string definition for message of type '<RBDLKinimatics-response>"
  (cl:format cl:nil "string[] names~%geometry_msgs/Point[] points~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLKinimatics-response)))
  "Returns full string definition for message of type 'RBDLKinimatics-response"
  (cl:format cl:nil "string[] names~%geometry_msgs/Point[] points~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLKinimatics-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLKinimatics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLKinimatics-response
    (cl:cons ':names (names msg))
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RBDLKinimatics)))
  'RBDLKinimatics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RBDLKinimatics)))
  'RBDLKinimatics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLKinimatics)))
  "Returns string type for a service object of type '<RBDLKinimatics>"
  "rbdl_server/RBDLKinimatics")