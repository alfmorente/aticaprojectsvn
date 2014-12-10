
(cl:in-package :asdf)

(defsystem "CITIUS_Control_Manager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_switcher" :depends-on ("_package_msg_switcher"))
    (:file "_package_msg_switcher" :depends-on ("_package"))
    (:file "msg_lastExec" :depends-on ("_package_msg_lastExec"))
    (:file "_package_msg_lastExec" :depends-on ("_package"))
  ))