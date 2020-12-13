;; Auto-generated. Do not edit!


(when (boundp 'rbdl_server::RBDLJacobian)
  (if (not (find-package "RBDL_SERVER"))
    (make-package "RBDL_SERVER"))
  (shadow 'RBDLJacobian (find-package "RBDL_SERVER")))
(unless (find-package "RBDL_SERVER::RBDLJACOBIAN")
  (make-package "RBDL_SERVER::RBDLJACOBIAN"))
(unless (find-package "RBDL_SERVER::RBDLJACOBIANREQUEST")
  (make-package "RBDL_SERVER::RBDLJACOBIANREQUEST"))
(unless (find-package "RBDL_SERVER::RBDLJACOBIANRESPONSE")
  (make-package "RBDL_SERVER::RBDLJACOBIANRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rbdl_server::RBDLJacobianRequest
  :super ros::object
  :slots (_body_name _q _point ))

(defmethod rbdl_server::RBDLJacobianRequest
  (:init
   (&key
    ((:body_name __body_name) "")
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _body_name (string __body_name))
   (setq _q __q)
   (setq _point __point)
   self)
  (:body_name
   (&optional __body_name)
   (if __body_name (setq _body_name __body_name)) _body_name)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; string _body_name
    4 (length _body_name)
    ;; float64[] _q
    (* 8    (length _q)) 4
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _body_name
       (write-long (length _body_name) s) (princ _body_name s)
     ;; float64[] _q
     (write-long (length _q) s)
     (dotimes (i (length _q))
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _body_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _body_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(defclass rbdl_server::RBDLJacobianResponse
  :super ros::object
  :slots (_jacobian ))

(defmethod rbdl_server::RBDLJacobianResponse
  (:init
   (&key
    ((:jacobian __jacobian) (instance std_msgs::Float64MultiArray :init))
    )
   (send-super :init)
   (setq _jacobian __jacobian)
   self)
  (:jacobian
   (&rest __jacobian)
   (if (keywordp (car __jacobian))
       (send* _jacobian __jacobian)
     (progn
       (if __jacobian (setq _jacobian (car __jacobian)))
       _jacobian)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray _jacobian
    (send _jacobian :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray _jacobian
       (send _jacobian :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray _jacobian
     (send _jacobian :deserialize buf ptr-) (incf ptr- (send _jacobian :serialization-length))
   ;;
   self)
  )

(defclass rbdl_server::RBDLJacobian
  :super ros::object
  :slots ())

(setf (get rbdl_server::RBDLJacobian :md5sum-) "4b2ed355812dee1bac3ae1b7e2f6c059")
(setf (get rbdl_server::RBDLJacobian :datatype-) "rbdl_server/RBDLJacobian")
(setf (get rbdl_server::RBDLJacobian :request) rbdl_server::RBDLJacobianRequest)
(setf (get rbdl_server::RBDLJacobian :response) rbdl_server::RBDLJacobianResponse)

(defmethod rbdl_server::RBDLJacobianRequest
  (:response () (instance rbdl_server::RBDLJacobianResponse :init)))

(setf (get rbdl_server::RBDLJacobianRequest :md5sum-) "4b2ed355812dee1bac3ae1b7e2f6c059")
(setf (get rbdl_server::RBDLJacobianRequest :datatype-) "rbdl_server/RBDLJacobianRequest")
(setf (get rbdl_server::RBDLJacobianRequest :definition-)
      "string body_name
float64[] q
geometry_msgs/Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
std_msgs/Float64MultiArray jacobian

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")

(setf (get rbdl_server::RBDLJacobianResponse :md5sum-) "4b2ed355812dee1bac3ae1b7e2f6c059")
(setf (get rbdl_server::RBDLJacobianResponse :datatype-) "rbdl_server/RBDLJacobianResponse")
(setf (get rbdl_server::RBDLJacobianResponse :definition-)
      "string body_name
float64[] q
geometry_msgs/Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
std_msgs/Float64MultiArray jacobian

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")



(provide :rbdl_server/RBDLJacobian "4b2ed355812dee1bac3ae1b7e2f6c059")


