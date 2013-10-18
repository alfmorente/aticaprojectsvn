
(cl:in-package :asdf)

(defsystem "Modulo_Laser2D-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_laser" :depends-on ("_package_msg_laser"))
    (:file "_package_msg_laser" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))