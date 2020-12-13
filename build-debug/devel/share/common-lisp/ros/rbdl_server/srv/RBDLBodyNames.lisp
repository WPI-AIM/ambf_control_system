; Auto-generated. Do not edit!


(cl:in-package rbdl_server-srv)


;//! \htmlinclude RBDLBodyNames-request.msg.html

(cl:defclass <RBDLBodyNames-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RBDLBodyNames-request (<RBDLBodyNames-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLBodyNames-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLBodyNames-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLBodyNames-request> is deprecated: use rbdl_server-srv:RBDLBodyNames-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLBodyNames-request>) ostream)
  "Serializes a message object of type '<RBDLBodyNames-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLBodyNames-request>) istream)
  "Deserializes a message object of type '<RBDLBodyNames-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLBodyNames-request>)))
  "Returns string type for a service object of type '<RBDLBodyNames-request>"
  "rbdl_server/RBDLBodyNamesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLBodyNames-request)))
  "Returns string type for a service object of type 'RBDLBodyNames-request"
  "rbdl_server/RBDLBodyNamesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLBodyNames-request>)))
  "Returns md5sum for a message object of type '<RBDLBodyNames-request>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLBodyNames-request)))
  "Returns md5sum for a message object of type 'RBDLBodyNames-request"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLBodyNames-request>)))
  "Returns full string definition for message of type '<RBDLBodyNames-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLBodyNames-request)))
  "Returns full string definition for message of type 'RBDLBodyNames-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLBodyNames-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLBodyNames-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLBodyNames-request
))
;//! \htmlinclude RBDLBodyNames-response.msg.html

(cl:defclass <RBDLBodyNames-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass RBDLBodyNames-response (<RBDLBodyNames-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLBodyNames-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLBodyNames-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLBodyNames-response> is deprecated: use rbdl_server-srv:RBDLBodyNames-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <RBDLBodyNames-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:names-val is deprecated.  Use rbdl_server-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLBodyNames-response>) ostream)
  "Serializes a message object of type '<RBDLBodyNames-response>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLBodyNames-response>) istream)
  "Deserializes a message object of type '<RBDLBodyNames-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLBodyNames-response>)))
  "Returns string type for a service object of type '<RBDLBodyNames-response>"
  "rbdl_server/RBDLBodyNamesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLBodyNames-response)))
  "Returns string type for a service object of type 'RBDLBodyNames-response"
  "rbdl_server/RBDLBodyNamesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLBodyNames-response>)))
  "Returns md5sum for a message object of type '<RBDLBodyNames-response>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLBodyNames-response)))
  "Returns md5sum for a message object of type 'RBDLBodyNames-response"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLBodyNames-response>)))
  "Returns full string definition for message of type '<RBDLBodyNames-response>"
  (cl:format cl:nil "string[] names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLBodyNames-response)))
  "Returns full string definition for message of type 'RBDLBodyNames-response"
  (cl:format cl:nil "string[] names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLBodyNames-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLBodyNames-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLBodyNames-response
    (cl:cons ':names (names msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RBDLBodyNames)))
  'RBDLBodyNames-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RBDLBodyNames)))
  'RBDLBodyNames-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLBodyNames)))
  "Returns string type for a service object of type '<RBDLBodyNames>"
  "rbdl_server/RBDLBodyNames")