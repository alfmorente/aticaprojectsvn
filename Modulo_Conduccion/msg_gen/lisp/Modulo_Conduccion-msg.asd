
(cl:in-package :asdf)

(defsystem "Modulo_Conduccion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "mastil" :depends-on ("_package_mastil"))
    (:file "_package_mastil" :depends-on ("_package"))
    (:file "nivelBomba" :depends-on ("_package_nivelBomba"))
    (:file "_package_nivelBomba" :depends-on ("_package"))
    (:file "messageCAN" :depends-on ("_package_messageCAN"))
    (:file "_package_messageCAN" :depends-on ("_package"))
    (:file "bomba" :depends-on ("_package_bomba"))
    (:file "_package_bomba" :depends-on ("_package"))
  ))