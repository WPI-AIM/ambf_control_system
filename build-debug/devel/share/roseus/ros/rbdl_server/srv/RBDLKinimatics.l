;; Auto-generated. Do not edit!


(when (boundp 'rbdl_server::RBDLKinimatics)
  (if (not (find-package "RBDL_SERVER"))
    (make-package "RBDL_SERVER"))
  (shadow 'RBDLKinimatics (find-package "RBDL_SERVER")))
(unless (find-package "RBDL_SERVER::RBDLKINIMATICS")
  (make-package "RBDL_SERVER::RBDLKINIMATICS"))
(unless (find-package "RBDL_SERVER::RBDLKINIMATICSREQUEST")
  (make-package "RBDL_SERVER::RBDLKINIMATICSREQUEST"))
(unless (find-package "RBDL_SERVER::RBDLKINIMATICSRESPONSE")
  (make-package "RBDL_SERVER::RBDLKINIMATICSRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rbdl_server::RBDLKinimaticsRequest
  :super ros::object
  :slots (_q ))

(defmethod rbdl_server::RBDLKinimaticsRequest
  (:init
   (&key
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _q __q)
   self)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:serialization-length
   ()
   (+
    ;; float64[] _q
    (* 8    (length _q)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _q
     (write-long (length _q) s)
     (dotimes (i (length _q))
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass rbdl_server::RBDLKinimaticsResponse
  :super ros::object
  :slots (_names _points ))

(defmethod rbdl_server::RBDLKinimaticsResponse
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:points __points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _names __names)
   (setq _points __points)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass rbdl_server::RBDLKinimatics
  :super ros::object
  :slots ())

(setf (get rbdl_server::RBDLKinimatics :md5sum-) "9ffe8145497585ad9a8e683635ee6416")
(setf (get rbdl_server::RBDLKinimatics :datatype-) "rbdl_server/RBDLKinimatics")
(setf (get rbdl_server::RBDLKinimatics :request) rbdl_server::RBDLKinimaticsRequest)
(setf (get rbdl_server::RBDLKinimatics :response) rbdl_server::RBDLKinimaticsResponse)

(defmethod rbdl_server::RBDLKinimaticsRequest
  (:response () (instance rbdl_server::RBDLKinimaticsResponse :init)))

(setf (get rbdl_server::RBDLKinimaticsRequest :md5sum-) "9ffe8145497585ad9a8e683635ee6416")
(setf (get rbdl_server::RBDLKinimaticsRequest :datatype-) "rbdl_server/RBDLKinimaticsRequest")
(setf (get rbdl_server::RBDLKinimaticsRequest :definition-)
      "float64[] q
---
string[] names
geometry_msgs/Point[] points

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get rbdl_server::RBDLKinimaticsResponse :md5sum-) "9ffe8145497585ad9a8e683635ee6416")
(setf (get rbdl_server::RBDLKinimaticsResponse :datatype-) "rbdl_server/RBDLKinimaticsResponse")
(setf (get rbdl_server::RBDLKinimaticsResponse :definition-)
      "float64[] q
---
string[] names
geometry_msgs/Point[] points

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :rbdl_server/RBDLKinimatics "9ffe8145497585ad9a8e683635ee6416")


