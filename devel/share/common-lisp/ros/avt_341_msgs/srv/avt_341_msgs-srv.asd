
(cl:in-package :asdf)

(defsystem "avt_341_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LiorfSaveMap" :depends-on ("_package_LiorfSaveMap"))
    (:file "_package_LiorfSaveMap" :depends-on ("_package"))
  ))