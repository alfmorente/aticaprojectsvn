
(cl:in-package :asdf)

(defsystem "Driving_Bobcat-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_switcher" :depends-on ("_package_msg_switcher"))
    (:file "_package_msg_switcher" :depends-on ("_package"))
    (:file "msg_command" :depends-on ("_package_msg_command"))
    (:file "_package_msg_command" :depends-on ("_package"))
    (:file "msg_vehicleInfo" :depends-on ("_package_msg_vehicleInfo"))
    (:file "_package_msg_vehicleInfo" :depends-on ("_package"))
  ))