
(cl:in-package :asdf)

(defsystem "Modulo_Camaras-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
    (:file "msg_camaras" :depends-on ("_package_msg_camaras"))
    (:file "_package_msg_camaras" :depends-on ("_package"))
  ))