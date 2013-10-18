
(cl:in-package :asdf)

(defsystem "Modulo_Conduccion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
               :tf-msg
)
  :components ((:file "_package")
    (:file "msg_gest_navegacion" :depends-on ("_package_msg_gest_navegacion"))
    (:file "_package_msg_gest_navegacion" :depends-on ("_package"))
    (:file "msg_com_teleoperado" :depends-on ("_package_msg_com_teleoperado"))
    (:file "_package_msg_com_teleoperado" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))