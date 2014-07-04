
(cl:in-package :asdf)

(defsystem "Modulo_Navegacion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
               :tf-msg
)
  :components ((:file "_package")
    (:file "msg_laser" :depends-on ("_package_msg_laser"))
    (:file "_package_msg_laser" :depends-on ("_package"))
    (:file "msg_habilitacion_modulo" :depends-on ("_package_msg_habilitacion_modulo"))
    (:file "_package_msg_habilitacion_modulo" :depends-on ("_package"))
    (:file "msg_gest_navegacion" :depends-on ("_package_msg_gest_navegacion"))
    (:file "_package_msg_gest_navegacion" :depends-on ("_package"))
    (:file "msg_modo" :depends-on ("_package_msg_modo"))
    (:file "_package_msg_modo" :depends-on ("_package"))
    (:file "msg_waypoint" :depends-on ("_package_msg_waypoint"))
    (:file "_package_msg_waypoint" :depends-on ("_package"))
    (:file "msg_gps" :depends-on ("_package_msg_gps"))
    (:file "_package_msg_gps" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))