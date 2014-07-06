
(cl:in-package :asdf)

(defsystem "Modulo_Gest_Sistema-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_mode" :depends-on ("_package_msg_mode"))
    (:file "_package_msg_mode" :depends-on ("_package"))
    (:file "msg_prueba" :depends-on ("_package_msg_prueba"))
    (:file "_package_msg_prueba" :depends-on ("_package"))
    (:file "msg_error" :depends-on ("_package_msg_error"))
    (:file "_package_msg_error" :depends-on ("_package"))
    (:file "msg_available_mode" :depends-on ("_package_msg_available_mode"))
    (:file "_package_msg_available_mode" :depends-on ("_package"))
    (:file "msg_module_enable" :depends-on ("_package_msg_module_enable"))
    (:file "_package_msg_module_enable" :depends-on ("_package"))
  ))