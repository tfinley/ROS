
(cl:in-package :asdf)

(defsystem "pick-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Reset_Button" :depends-on ("_package_Reset_Button"))
    (:file "_package_Reset_Button" :depends-on ("_package"))
    (:file "Confirm_Button" :depends-on ("_package_Confirm_Button"))
    (:file "_package_Confirm_Button" :depends-on ("_package"))
  ))