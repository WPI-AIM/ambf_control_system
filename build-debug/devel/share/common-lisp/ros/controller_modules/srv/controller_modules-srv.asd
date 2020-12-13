
(cl:in-package :asdf)

(defsystem "controller_modules-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "ControllerList" :depends-on ("_package_ControllerList"))
    (:file "_package_ControllerList" :depends-on ("_package"))
    (:file "JointControl" :depends-on ("_package_JointControl"))
    (:file "_package_JointControl" :depends-on ("_package"))
  ))