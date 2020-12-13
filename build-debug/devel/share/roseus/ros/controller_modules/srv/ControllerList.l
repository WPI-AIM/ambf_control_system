;; Auto-generated. Do not edit!


(when (boundp 'controller_modules::ControllerList)
  (if (not (find-package "CONTROLLER_MODULES"))
    (make-package "CONTROLLER_MODULES"))
  (shadow 'ControllerList (find-package "CONTROLLER_MODULES")))
(unless (find-package "CONTROLLER_MODULES::CONTROLLERLIST")
  (make-package "CONTROLLER_MODULES::CONTROLLERLIST"))
(unless (find-package "CONTROLLER_MODULES::CONTROLLERLISTREQUEST")
  (make-package "CONTROLLER_MODULES::CONTROLLERLISTREQUEST"))
(unless (find-package "CONTROLLER_MODULES::CONTROLLERLISTRESPONSE")
  (make-package "CONTROLLER_MODULES::CONTROLLERLISTRESPONSE"))

(in-package "ROS")





(defclass controller_modules::ControllerListRequest
  :super ros::object
  :slots ())

(defmethod controller_modules::ControllerListRequest
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

(defclass controller_modules::ControllerListResponse
  :super ros::object
  :slots (_controllers ))

(defmethod controller_modules::ControllerListResponse
  (:init
   (&key
    ((:controllers __controllers) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _controllers __controllers)
   self)
  (:controllers
   (&optional __controllers)
   (if __controllers (setq _controllers __controllers)) _controllers)
  (:serialization-length
   ()
   (+
    ;; string[] _controllers
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _controllers)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _controllers
     (write-long (length _controllers) s)
     (dolist (elem _controllers)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _controllers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controllers (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _controllers i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass controller_modules::ControllerList
  :super ros::object
  :slots ())

(setf (get controller_modules::ControllerList :md5sum-) "8242783a458107f102bce0800c333f0a")
(setf (get controller_modules::ControllerList :datatype-) "controller_modules/ControllerList")
(setf (get controller_modules::ControllerList :request) controller_modules::ControllerListRequest)
(setf (get controller_modules::ControllerList :response) controller_modules::ControllerListResponse)

(defmethod controller_modules::ControllerListRequest
  (:response () (instance controller_modules::ControllerListResponse :init)))

(setf (get controller_modules::ControllerListRequest :md5sum-) "8242783a458107f102bce0800c333f0a")
(setf (get controller_modules::ControllerListRequest :datatype-) "controller_modules/ControllerListRequest")
(setf (get controller_modules::ControllerListRequest :definition-)
      "---
string[] controllers
")

(setf (get controller_modules::ControllerListResponse :md5sum-) "8242783a458107f102bce0800c333f0a")
(setf (get controller_modules::ControllerListResponse :datatype-) "controller_modules/ControllerListResponse")
(setf (get controller_modules::ControllerListResponse :definition-)
      "---
string[] controllers
")



(provide :controller_modules/ControllerList "8242783a458107f102bce0800c333f0a")


