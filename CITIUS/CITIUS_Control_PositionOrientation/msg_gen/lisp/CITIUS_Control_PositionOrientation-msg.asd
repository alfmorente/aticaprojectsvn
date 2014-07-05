
(cl:in-package :asdf)

(defsystem "CITIUS_Control_PositionOrientation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_posOriInfo" :depends-on ("_package_msg_posOriInfo"))
    (:file "_package_msg_posOriInfo" :depends-on ("_package"))
  ))