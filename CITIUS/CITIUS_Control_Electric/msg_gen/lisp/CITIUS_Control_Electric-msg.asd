
(cl:in-package :asdf)

(defsystem "CITIUS_Control_Electric-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_electricInfo" :depends-on ("_package_msg_electricInfo"))
    (:file "_package_msg_electricInfo" :depends-on ("_package"))
    (:file "msg_switcher" :depends-on ("_package_msg_switcher"))
    (:file "_package_msg_switcher" :depends-on ("_package"))
    (:file "msg_command" :depends-on ("_package_msg_command"))
    (:file "_package_msg_command" :depends-on ("_package"))
  ))