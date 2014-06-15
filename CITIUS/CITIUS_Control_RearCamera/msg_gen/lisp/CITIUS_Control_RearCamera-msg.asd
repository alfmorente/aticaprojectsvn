
(cl:in-package :asdf)

(defsystem "CITIUS_Control_RearCamera-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_rearCameraInfo" :depends-on ("_package_msg_rearCameraInfo"))
    (:file "_package_msg_rearCameraInfo" :depends-on ("_package"))
    (:file "msg_ctrlRearCamera" :depends-on ("_package_msg_ctrlRearCamera"))
    (:file "_package_msg_ctrlRearCamera" :depends-on ("_package"))
  ))