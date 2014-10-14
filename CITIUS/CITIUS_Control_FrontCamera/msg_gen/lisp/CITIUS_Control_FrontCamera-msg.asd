
(cl:in-package :asdf)

(defsystem "CITIUS_Control_FrontCamera-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_ctrlFrontCamera" :depends-on ("_package_msg_ctrlFrontCamera"))
    (:file "_package_msg_ctrlFrontCamera" :depends-on ("_package"))
    (:file "msg_frontCameraInfo" :depends-on ("_package_msg_frontCameraInfo"))
    (:file "_package_msg_frontCameraInfo" :depends-on ("_package"))
  ))