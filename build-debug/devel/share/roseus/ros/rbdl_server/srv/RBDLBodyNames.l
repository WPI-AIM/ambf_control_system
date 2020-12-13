;; Auto-generated. Do not edit!


(when (boundp 'rbdl_server::RBDLBodyNames)
  (if (not (find-package "RBDL_SERVER"))
    (make-package "RBDL_SERVER"))
  (shadow 'RBDLBodyNames (find-package "RBDL_SERVER")))
(unless (find-package "RBDL_SERVER::RBDLBODYNAMES")
  (make-package "RBDL_SERVER::RBDLBODYNAMES"))
(unless (find-package "RBDL_SERVER::RBDLBODYNAMESREQUEST")
  (make-package "RBDL_SERVER::RBDLBODYNAMESREQUEST"))
(unless (find-package "RBDL_SERVER::RBDLBODYNAMESRESPONSE")
  (make-package "RBDL_SERVER::RBDLBODYNAMESRESPONSE"))

(in-package "ROS")





(defclass rbdl_server::RBDLBodyNamesRequest
  :super ros::object
  :slots ())

(defmethod rbdl_server::RBDLBodyNamesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass rbdl_server::RBDLBodyNamesResponse
  :super ros::object
  :slots (_names ))

(defmethod rbdl_server::RBDLBodyNamesResponse
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _names __names)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
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
   ;;
   self)
  )

(defclass rbdl_server::RBDLBodyNames
  :super ros::object
  :slots ())

(setf (get rbdl_server::RBDLBodyNames :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get rbdl_server::RBDLBodyNames :datatype-) "rbdl_server/RBDLBodyNames")
(setf (get rbdl_server::RBDLBodyNames :request) rbdl_server::RBDLBodyNamesRequest)
(setf (get rbdl_server::RBDLBodyNames :response) rbdl_server::RBDLBodyNamesResponse)

(defmethod rbdl_server::RBDLBodyNamesRequest
  (:response () (instance rbdl_server::RBDLBodyNamesResponse :init)))

(setf (get rbdl_server::RBDLBodyNamesRequest :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get rbdl_server::RBDLBodyNamesRequest :datatype-) "rbdl_server/RBDLBodyNamesRequest")
(setf (get rbdl_server::RBDLBodyNamesRequest :definition-)
      "---
string[] names
")

(setf (get rbdl_server::RBDLBodyNamesResponse :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get rbdl_server::RBDLBodyNamesResponse :datatype-) "rbdl_server/RBDLBodyNamesResponse")
(setf (get rbdl_server::RBDLBodyNamesResponse :definition-)
      "---
string[] names
")



(provide :rbdl_server/RBDLBodyNames "dc7ae3609524b18034e49294a4ce670e")


