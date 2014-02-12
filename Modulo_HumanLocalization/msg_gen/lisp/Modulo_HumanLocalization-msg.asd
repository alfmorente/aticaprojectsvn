
(cl:in-package :asdf)

(defsystem "Modulo_HumanLocalization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_module_enable" :depends-on ("_package_msg_module_enable"))
    (:file "_package_msg_module_enable" :depends-on ("_package"))
    (:file "msg_waypoint" :depends-on ("_package_msg_waypoint"))
    (:file "_package_msg_waypoint" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))