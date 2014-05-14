
(cl:in-package :asdf)

(defsystem "CITIUS_Control_Manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srv_status" :depends-on ("_package_srv_status"))
    (:file "_package_srv_status" :depends-on ("_package"))
  ))