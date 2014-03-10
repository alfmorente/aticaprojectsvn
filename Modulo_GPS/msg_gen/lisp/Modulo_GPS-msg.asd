
(cl:in-package :asdf)

(defsystem "Modulo_GPS-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_backup" :depends-on ("_package_msg_backup"))
    (:file "_package_msg_backup" :depends-on ("_package"))
    (:file "msg_error" :depends-on ("_package_msg_error"))
    (:file "_package_msg_error" :depends-on ("_package"))
    (:file "msg_gps" :depends-on ("_package_msg_gps"))
    (:file "_package_msg_gps" :depends-on ("_package"))
    (:file "msg_module_enable" :depends-on ("_package_msg_module_enable"))
    (:file "_package_msg_module_enable" :depends-on ("_package"))
  ))