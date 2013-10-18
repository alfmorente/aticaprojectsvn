
(cl:in-package :asdf)

(defsystem "Modulo_TeachMapping-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_laser" :depends-on ("_package_msg_laser"))
    (:file "_package_msg_laser" :depends-on ("_package"))
    (:file "msg_habilitacion_modulo" :depends-on ("_package_msg_habilitacion_modulo"))
    (:file "_package_msg_habilitacion_modulo" :depends-on ("_package"))
    (:file "msg_gps" :depends-on ("_package_msg_gps"))
    (:file "_package_msg_gps" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))