; Auto-generated. Do not edit!


(cl:in-package controller_modules-srv)


;//! \htmlinclude ControllerList-request.msg.html

(cl:defclass <ControllerList-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerList-request (<ControllerList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller_modules-srv:<ControllerList-request> is deprecated: use controller_modules-srv:ControllerList-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerList-request>) ostream)
  "Serializes a message object of type '<ControllerList-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerList-request>) istream)
  "Deserializes a message object of type '<ControllerList-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerList-request>)))
  "Returns string type for a service object of type '<ControllerList-request>"
  "controller_modules/ControllerListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerList-request)))
  "Returns string type for a service object of type 'ControllerList-request"
  "controller_modules/ControllerListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerList-request>)))
  "Returns md5sum for a message object of type '<ControllerList-request>"
  "8242783a458107f102bce0800c333f0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerList-request)))
  "Returns md5sum for a message object of type 'ControllerList-request"
  "8242783a458107f102bce0800c333f0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerList-request>)))
  "Returns full string definition for message of type '<ControllerList-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerList-request)))
  "Returns full string definition for message of type 'ControllerList-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerList-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerList-request
))
;//! \htmlinclude ControllerList-response.msg.html

(cl:defclass <ControllerList-response> (roslisp-msg-protocol:ros-message)
  ((controllers
    :reader controllers
    :initarg :controllers
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ControllerList-response (<ControllerList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller_modules-srv:<ControllerList-response> is deprecated: use controller_modules-srv:ControllerList-response instead.")))

(cl:ensure-generic-function 'controllers-val :lambda-list '(m))
(cl:defmethod controllers-val ((m <ControllerList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller_modules-srv:controllers-val is deprecated.  Use controller_modules-srv:controllers instead.")
  (controllers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerList-response>) ostream)
  "Serializes a message object of type '<ControllerList-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controllers))))
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
   (cl:slot-value msg 'controllers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerList-response>) istream)
  "Deserializes a message object of type '<ControllerList-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controllers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controllers)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerList-response>)))
  "Returns string type for a service object of type '<ControllerList-response>"
  "controller_modules/ControllerListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerList-response)))
  "Returns string type for a service object of type 'ControllerList-response"
  "controller_modules/ControllerListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerList-response>)))
  "Returns md5sum for a message object of type '<ControllerList-response>"
  "8242783a458107f102bce0800c333f0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerList-response)))
  "Returns md5sum for a message object of type 'ControllerList-response"
  "8242783a458107f102bce0800c333f0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerList-response>)))
  "Returns full string definition for message of type '<ControllerList-response>"
  (cl:format cl:nil "string[] controllers~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerList-response)))
  "Returns full string definition for message of type 'ControllerList-response"
  (cl:format cl:nil "string[] controllers~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerList-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controllers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerList-response
    (cl:cons ':controllers (controllers msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControllerList)))
  'ControllerList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControllerList)))
  'ControllerList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerList)))
  "Returns string type for a service object of type '<ControllerList>"
  "controller_modules/ControllerList")