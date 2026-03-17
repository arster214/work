
(cl:in-package :asdf)

(defsystem "servo_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ServoAngle" :depends-on ("_package_ServoAngle"))
    (:file "_package_ServoAngle" :depends-on ("_package"))
    (:file "ServoMove" :depends-on ("_package_ServoMove"))
    (:file "_package_ServoMove" :depends-on ("_package"))
  ))