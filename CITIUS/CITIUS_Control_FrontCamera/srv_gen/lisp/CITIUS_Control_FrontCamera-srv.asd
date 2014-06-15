
(cl:in-package :asdf)

(defsystem "CITIUS_Control_FrontCamera-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srv_nodeStatus" :depends-on ("_package_srv_nodeStatus"))
    (:file "_package_srv_nodeStatus" :depends-on ("_package"))
  ))