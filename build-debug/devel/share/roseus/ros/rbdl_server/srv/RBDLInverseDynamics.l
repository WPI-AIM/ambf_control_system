;; Auto-generated. Do not edit!


(when (boundp 'rbdl_server::RBDLInverseDynamics)
  (if (not (find-package "RBDL_SERVER"))
    (make-package "RBDL_SERVER"))
  (shadow 'RBDLInverseDynamics (find-package "RBDL_SERVER")))
(unless (find-package "RBDL_SERVER::RBDLINVERSEDYNAMICS")
  (make-package "RBDL_SERVER::RBDLINVERSEDYNAMICS"))
(unless (find-package "RBDL_SERVER::RBDLINVERSEDYNAMICSREQUEST")
  (make-package "RBDL_SERVER::RBDLINVERSEDYNAMICSREQUEST"))
(unless (find-package "RBDL_SERVER::RBDLINVERSEDYNAMICSRESPONSE")
  (make-package "RBDL_SERVER::RBDLINVERSEDYNAMICSRESPONSE"))

(in-package "ROS")





(defclass rbdl_server::RBDLInverseDynamicsRequest
  :super ros::object
  :slots (_q _qd _qdd ))

(defmethod rbdl_server::RBDLInverseDynamicsRequest
  (:init
   (&key
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:qd __qd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:qdd __qdd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _q __q)
   (setq _qd __qd)
   (setq _qdd __qdd)
   self)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:qd
   (&optional __qd)
   (if __qd (setq _qd __qd)) _qd)
  (:qdd
   (&optional __qdd)
   (if __qdd (setq _qdd __qdd)) _qdd)
  (:serialization-length
   ()
   (+
    ;; float64[] _q
    (* 8    (length _q)) 4
    ;; float64[] _qd
    (* 8    (length _qd)) 4
    ;; float64[] _qdd
    (* 8    (length _qdd)) 4
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
     ;; float64[] _qdd
     (write-long (length _qdd) s)
     (dotimes (i (length _qdd))
       (sys::poke (elt _qdd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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

(defclass rbdl_server::RBDLInverseDynamicsResponse
  :super ros::object
  :slots (_tau ))

(defmethod rbdl_server::RBDLInverseDynamicsResponse
  (:init
   (&key
    ((:tau __tau) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _tau __tau)
   self)
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:serialization-length
   ()
   (+
    ;; float64[] _tau
    (* 8    (length _tau)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _tau
     (write-long (length _tau) s)
     (dotimes (i (length _tau))
       (sys::poke (elt _tau i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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

(defclass rbdl_server::RBDLInverseDynamics
  :super ros::object
  :slots ())

(setf (get rbdl_server::RBDLInverseDynamics :md5sum-) "910ba3a2a185792e4edeaab48cb0189d")
(setf (get rbdl_server::RBDLInverseDynamics :datatype-) "rbdl_server/RBDLInverseDynamics")
(setf (get rbdl_server::RBDLInverseDynamics :request) rbdl_server::RBDLInverseDynamicsRequest)
(setf (get rbdl_server::RBDLInverseDynamics :response) rbdl_server::RBDLInverseDynamicsResponse)

(defmethod rbdl_server::RBDLInverseDynamicsRequest
  (:response () (instance rbdl_server::RBDLInverseDynamicsResponse :init)))

(setf (get rbdl_server::RBDLInverseDynamicsRequest :md5sum-) "910ba3a2a185792e4edeaab48cb0189d")
(setf (get rbdl_server::RBDLInverseDynamicsRequest :datatype-) "rbdl_server/RBDLInverseDynamicsRequest")
(setf (get rbdl_server::RBDLInverseDynamicsRequest :definition-)
      "float64[] q
float64[] qd
float64[] qdd
---
float64[] tau

")

(setf (get rbdl_server::RBDLInverseDynamicsResponse :md5sum-) "910ba3a2a185792e4edeaab48cb0189d")
(setf (get rbdl_server::RBDLInverseDynamicsResponse :datatype-) "rbdl_server/RBDLInverseDynamicsResponse")
(setf (get rbdl_server::RBDLInverseDynamicsResponse :definition-)
      "float64[] q
float64[] qd
float64[] qdd
---
float64[] tau

")



(provide :rbdl_server/RBDLInverseDynamics "910ba3a2a185792e4edeaab48cb0189d")


