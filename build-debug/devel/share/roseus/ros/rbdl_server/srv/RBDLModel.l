;; Auto-generated. Do not edit!


(when (boundp 'rbdl_server::RBDLModel)
  (if (not (find-package "RBDL_SERVER"))
    (make-package "RBDL_SERVER"))
  (shadow 'RBDLModel (find-package "RBDL_SERVER")))
(unless (find-package "RBDL_SERVER::RBDLMODEL")
  (make-package "RBDL_SERVER::RBDLMODEL"))
(unless (find-package "RBDL_SERVER::RBDLMODELREQUEST")
  (make-package "RBDL_SERVER::RBDLMODELREQUEST"))
(unless (find-package "RBDL_SERVER::RBDLMODELRESPONSE")
  (make-package "RBDL_SERVER::RBDLMODELRESPONSE"))

(in-package "ROS")





(defclass rbdl_server::RBDLModelRequest
  :super ros::object
  :slots (_model ))

(defmethod rbdl_server::RBDLModelRequest
  (:init
   (&key
    ((:model __model) "")
    )
   (send-super :init)
   (setq _model (string __model))
   self)
  (:model
   (&optional __model)
   (if __model (setq _model __model)) _model)
  (:serialization-length
   ()
   (+
    ;; string _model
    4 (length _model)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model
       (write-long (length _model) s) (princ _model s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rbdl_server::RBDLModelResponse
  :super ros::object
  :slots (_good ))

(defmethod rbdl_server::RBDLModelResponse
  (:init
   (&key
    ((:good __good) nil)
    )
   (send-super :init)
   (setq _good __good)
   self)
  (:good
   (&optional __good)
   (if __good (setq _good __good)) _good)
  (:serialization-length
   ()
   (+
    ;; bool _good
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _good
       (if _good (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _good
     (setq _good (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rbdl_server::RBDLModel
  :super ros::object
  :slots ())

(setf (get rbdl_server::RBDLModel :md5sum-) "d092f66b4213534ea05b76cd5ee71314")
(setf (get rbdl_server::RBDLModel :datatype-) "rbdl_server/RBDLModel")
(setf (get rbdl_server::RBDLModel :request) rbdl_server::RBDLModelRequest)
(setf (get rbdl_server::RBDLModel :response) rbdl_server::RBDLModelResponse)

(defmethod rbdl_server::RBDLModelRequest
  (:response () (instance rbdl_server::RBDLModelResponse :init)))

(setf (get rbdl_server::RBDLModelRequest :md5sum-) "d092f66b4213534ea05b76cd5ee71314")
(setf (get rbdl_server::RBDLModelRequest :datatype-) "rbdl_server/RBDLModelRequest")
(setf (get rbdl_server::RBDLModelRequest :definition-)
      "string model
---
bool good
")

(setf (get rbdl_server::RBDLModelResponse :md5sum-) "d092f66b4213534ea05b76cd5ee71314")
(setf (get rbdl_server::RBDLModelResponse :datatype-) "rbdl_server/RBDLModelResponse")
(setf (get rbdl_server::RBDLModelResponse :definition-)
      "string model
---
bool good
")



(provide :rbdl_server/RBDLModel "d092f66b4213534ea05b76cd5ee71314")


