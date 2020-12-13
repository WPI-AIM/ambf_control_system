; Auto-generated. Do not edit!


(cl:in-package rbdl_server-srv)


;//! \htmlinclude RBDLModel-request.msg.html

(cl:defclass <RBDLModel-request> (roslisp-msg-protocol:ros-message)
  ((model
    :reader model
    :initarg :model
    :type cl:string
    :initform ""))
)

(cl:defclass RBDLModel-request (<RBDLModel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLModel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLModel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLModel-request> is deprecated: use rbdl_server-srv:RBDLModel-request instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <RBDLModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:model-val is deprecated.  Use rbdl_server-srv:model instead.")
  (model m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLModel-request>) ostream)
  "Serializes a message object of type '<RBDLModel-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLModel-request>) istream)
  "Deserializes a message object of type '<RBDLModel-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLModel-request>)))
  "Returns string type for a service object of type '<RBDLModel-request>"
  "rbdl_server/RBDLModelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLModel-request)))
  "Returns string type for a service object of type 'RBDLModel-request"
  "rbdl_server/RBDLModelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLModel-request>)))
  "Returns md5sum for a message object of type '<RBDLModel-request>"
  "d092f66b4213534ea05b76cd5ee71314")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLModel-request)))
  "Returns md5sum for a message object of type 'RBDLModel-request"
  "d092f66b4213534ea05b76cd5ee71314")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLModel-request>)))
  "Returns full string definition for message of type '<RBDLModel-request>"
  (cl:format cl:nil "string model~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLModel-request)))
  "Returns full string definition for message of type 'RBDLModel-request"
  (cl:format cl:nil "string model~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLModel-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLModel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLModel-request
    (cl:cons ':model (model msg))
))
;//! \htmlinclude RBDLModel-response.msg.html

(cl:defclass <RBDLModel-response> (roslisp-msg-protocol:ros-message)
  ((good
    :reader good
    :initarg :good
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RBDLModel-response (<RBDLModel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLModel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLModel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLModel-response> is deprecated: use rbdl_server-srv:RBDLModel-response instead.")))

(cl:ensure-generic-function 'good-val :lambda-list '(m))
(cl:defmethod good-val ((m <RBDLModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:good-val is deprecated.  Use rbdl_server-srv:good instead.")
  (good m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLModel-response>) ostream)
  "Serializes a message object of type '<RBDLModel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'good) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLModel-response>) istream)
  "Deserializes a message object of type '<RBDLModel-response>"
    (cl:setf (cl:slot-value msg 'good) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLModel-response>)))
  "Returns string type for a service object of type '<RBDLModel-response>"
  "rbdl_server/RBDLModelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLModel-response)))
  "Returns string type for a service object of type 'RBDLModel-response"
  "rbdl_server/RBDLModelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLModel-response>)))
  "Returns md5sum for a message object of type '<RBDLModel-response>"
  "d092f66b4213534ea05b76cd5ee71314")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLModel-response)))
  "Returns md5sum for a message object of type 'RBDLModel-response"
  "d092f66b4213534ea05b76cd5ee71314")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLModel-response>)))
  "Returns full string definition for message of type '<RBDLModel-response>"
  (cl:format cl:nil "bool good~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLModel-response)))
  "Returns full string definition for message of type 'RBDLModel-response"
  (cl:format cl:nil "bool good~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLModel-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLModel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLModel-response
    (cl:cons ':good (good msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RBDLModel)))
  'RBDLModel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RBDLModel)))
  'RBDLModel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLModel)))
  "Returns string type for a service object of type '<RBDLModel>"
  "rbdl_server/RBDLModel")