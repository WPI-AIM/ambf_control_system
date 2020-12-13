; Auto-generated. Do not edit!


(cl:in-package rbdl_server-srv)


;//! \htmlinclude RBDLJacobian-request.msg.html

(cl:defclass <RBDLJacobian-request> (roslisp-msg-protocol:ros-message)
  ((body_name
    :reader body_name
    :initarg :body_name
    :type cl:string
    :initform "")
   (q
    :reader q
    :initarg :q
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass RBDLJacobian-request (<RBDLJacobian-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLJacobian-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLJacobian-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLJacobian-request> is deprecated: use rbdl_server-srv:RBDLJacobian-request instead.")))

(cl:ensure-generic-function 'body_name-val :lambda-list '(m))
(cl:defmethod body_name-val ((m <RBDLJacobian-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:body_name-val is deprecated.  Use rbdl_server-srv:body_name instead.")
  (body_name m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <RBDLJacobian-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:q-val is deprecated.  Use rbdl_server-srv:q instead.")
  (q m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <RBDLJacobian-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:point-val is deprecated.  Use rbdl_server-srv:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLJacobian-request>) ostream)
  "Serializes a message object of type '<RBDLJacobian-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'body_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'body_name))
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLJacobian-request>) istream)
  "Deserializes a message object of type '<RBDLJacobian-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'body_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'body_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLJacobian-request>)))
  "Returns string type for a service object of type '<RBDLJacobian-request>"
  "rbdl_server/RBDLJacobianRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLJacobian-request)))
  "Returns string type for a service object of type 'RBDLJacobian-request"
  "rbdl_server/RBDLJacobianRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLJacobian-request>)))
  "Returns md5sum for a message object of type '<RBDLJacobian-request>"
  "4b2ed355812dee1bac3ae1b7e2f6c059")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLJacobian-request)))
  "Returns md5sum for a message object of type 'RBDLJacobian-request"
  "4b2ed355812dee1bac3ae1b7e2f6c059")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLJacobian-request>)))
  "Returns full string definition for message of type '<RBDLJacobian-request>"
  (cl:format cl:nil "string body_name~%float64[] q~%geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLJacobian-request)))
  "Returns full string definition for message of type 'RBDLJacobian-request"
  (cl:format cl:nil "string body_name~%float64[] q~%geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLJacobian-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'body_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLJacobian-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLJacobian-request
    (cl:cons ':body_name (body_name msg))
    (cl:cons ':q (q msg))
    (cl:cons ':point (point msg))
))
;//! \htmlinclude RBDLJacobian-response.msg.html

(cl:defclass <RBDLJacobian-response> (roslisp-msg-protocol:ros-message)
  ((jacobian
    :reader jacobian
    :initarg :jacobian
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray)))
)

(cl:defclass RBDLJacobian-response (<RBDLJacobian-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RBDLJacobian-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RBDLJacobian-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rbdl_server-srv:<RBDLJacobian-response> is deprecated: use rbdl_server-srv:RBDLJacobian-response instead.")))

(cl:ensure-generic-function 'jacobian-val :lambda-list '(m))
(cl:defmethod jacobian-val ((m <RBDLJacobian-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rbdl_server-srv:jacobian-val is deprecated.  Use rbdl_server-srv:jacobian instead.")
  (jacobian m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RBDLJacobian-response>) ostream)
  "Serializes a message object of type '<RBDLJacobian-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'jacobian) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RBDLJacobian-response>) istream)
  "Deserializes a message object of type '<RBDLJacobian-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'jacobian) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RBDLJacobian-response>)))
  "Returns string type for a service object of type '<RBDLJacobian-response>"
  "rbdl_server/RBDLJacobianResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLJacobian-response)))
  "Returns string type for a service object of type 'RBDLJacobian-response"
  "rbdl_server/RBDLJacobianResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RBDLJacobian-response>)))
  "Returns md5sum for a message object of type '<RBDLJacobian-response>"
  "4b2ed355812dee1bac3ae1b7e2f6c059")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RBDLJacobian-response)))
  "Returns md5sum for a message object of type 'RBDLJacobian-response"
  "4b2ed355812dee1bac3ae1b7e2f6c059")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RBDLJacobian-response>)))
  "Returns full string definition for message of type '<RBDLJacobian-response>"
  (cl:format cl:nil "std_msgs/Float64MultiArray jacobian~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RBDLJacobian-response)))
  "Returns full string definition for message of type 'RBDLJacobian-response"
  (cl:format cl:nil "std_msgs/Float64MultiArray jacobian~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RBDLJacobian-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'jacobian))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RBDLJacobian-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RBDLJacobian-response
    (cl:cons ':jacobian (jacobian msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RBDLJacobian)))
  'RBDLJacobian-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RBDLJacobian)))
  'RBDLJacobian-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RBDLJacobian)))
  "Returns string type for a service object of type '<RBDLJacobian>"
  "rbdl_server/RBDLJacobian")