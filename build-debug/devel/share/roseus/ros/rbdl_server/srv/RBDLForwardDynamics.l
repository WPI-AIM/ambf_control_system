;; Auto-generated. Do not edit!


(when (boundp 'rbdl_server::RBDLForwardDynamics)
  (if (not (find-package "RBDL_SERVER"))
    (make-package "RBDL_SERVER"))
  (shadow 'RBDLForwardDynamics (find-package "RBDL_SERVER")))
(unless (find-package "RBDL_SERVER::RBDLFORWARDDYNAMICS")
  (make-package "RBDL_SERVER::RBDLFORWARDDYNAMICS"))
(unless (find-package "RBDL_SERVER::RBDLFORWARDDYNAMICSREQUEST")
  (make-package "RBDL_SERVER::RBDLFORWARDDYNAMICSREQUEST"))
(unless (find-package "RBDL_SERVER::RBDLFORWARDDYNAMICSRESPONSE")
  (make-package "RBDL_SERVER::RBDLFORWARDDYNAMICSRESPONSE"))

(in-package "ROS")





(defclass rbdl_server::RBDLForwardDynamicsRequest
  :super ros::object
  :slots (_q _qd _tau ))

(defmethod rbdl_server::RBDLForwardDynamicsRequest
  (:init
   (&key
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:qd __qd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tau __tau) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _q __q)
   (setq _qd __qd)
   (setq _tau __tau)
   self)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:qd
   (&optional __qd)
   (if __qd (setq _qd __qd)) _qd)
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:serialization-length
   ()
   (+
    ;; float64[] _q
    (* 8    (length _q)) 4
    ;; float64[] _qd
    (* 8    (length _qd)) 4
    ;; float64[] _tau
    (* 8    (length _tau)) 4
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
     ;; float64[] _qd
     (write-long (length _qd) s)
     (dotimes (i (length _qd))
       (sys::poke (elt _qd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _tau
     (write-long (length _tau) s)
     (dotimes (i (length _tau))
       (sys::poke (elt _tau i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64[] _qd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _qd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _qd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _tau
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tau (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tau i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass rbdl_server::RBDLForwardDynamicsResponse
  :super ros::object
  :slots (_qdd ))

(defmethod rbdl_server::RBDLForwardDynamicsResponse
  (:init
   (&key
    ((:qdd __qdd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _qdd __qdd)
   self)
  (:qdd
   (&optional __qdd)
   (if __qdd (setq _qdd __qdd)) _qdd)
  (:serialization-length
   ()
   (+
    ;; float64[] _qdd
    (* 8    (length _qdd)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _qdd
     (write-long (length _qdd) s)
     (dotimes (i (length _qdd))
       (sys::poke (elt _qdd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _qdd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _qdd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _qdd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass rbdl_server::RBDLForwardDynamics
  :super ros::object
  :slots ())

(setf (get rbdl_server::RBDLForwardDynamics :md5sum-) "2510e5eefce24dbe51d450b7f1432820")
(setf (get rbdl_server::RBDLForwardDynamics :datatype-) "rbdl_server/RBDLForwardDynamics")
(setf (get rbdl_server::RBDLForwardDynamics :request) rbdl_server::RBDLForwardDynamicsRequest)
(setf (get rbdl_server::RBDLForwardDynamics :response) rbdl_server::RBDLForwardDynamicsResponse)

(defmethod rbdl_server::RBDLForwardDynamicsRequest
  (:response () (instance rbdl_server::RBDLForwardDynamicsResponse :init)))

(setf (get rbdl_server::RBDLForwardDynamicsRequest :md5sum-) "2510e5eefce24dbe51d450b7f1432820")
(setf (get rbdl_server::RBDLForwardDynamicsRequest :datatype-) "rbdl_server/RBDLForwardDynamicsRequest")
(setf (get rbdl_server::RBDLForwardDynamicsRequest :definition-)
      "float64[] q
float64[] qd
float64[] tau
---
float64[] qdd

")

(setf (get rbdl_server::RBDLForwardDynamicsResponse :md5sum-) "2510e5eefce24dbe51d450b7f1432820")
(setf (get rbdl_server::RBDLForwardDynamicsResponse :datatype-) "rbdl_server/RBDLForwardDynamicsResponse")
(setf (get rbdl_server::RBDLForwardDynamicsResponse :definition-)
      "float64[] q
float64[] qd
float64[] tau
---
float64[] qdd

")



(provide :rbdl_server/RBDLForwardDynamics "2510e5eefce24dbe51d450b7f1432820")


