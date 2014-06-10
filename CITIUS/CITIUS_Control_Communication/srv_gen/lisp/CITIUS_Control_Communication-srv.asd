
(cl:in-package :asdf)

(defsystem "CITIUS_Control_Communication-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srv_vehicleStatus" :depends-on ("_package_srv_vehicleStatus"))
    (:file "_package_srv_vehicleStatus" :depends-on ("_package"))
  ))