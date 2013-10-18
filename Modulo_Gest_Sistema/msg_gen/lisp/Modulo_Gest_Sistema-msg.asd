
(cl:in-package :asdf)

(defsystem "Modulo_Gest_Sistema-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_habilitacion_modulo" :depends-on ("_package_msg_habilitacion_modulo"))
    (:file "_package_msg_habilitacion_modulo" :depends-on ("_package"))
    (:file "msg_modo" :depends-on ("_package_msg_modo"))
    (:file "_package_msg_modo" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))