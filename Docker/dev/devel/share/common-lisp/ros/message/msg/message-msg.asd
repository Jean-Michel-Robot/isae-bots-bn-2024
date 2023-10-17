
(cl:in-package :asdf)

(defsystem "message-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ActionnersMsg" :depends-on ("_package_ActionnersMsg"))
    (:file "_package_ActionnersMsg" :depends-on ("_package"))
    (:file "EndOfActionMsg" :depends-on ("_package_EndOfActionMsg"))
    (:file "_package_EndOfActionMsg" :depends-on ("_package"))
    (:file "InfoMsg" :depends-on ("_package_InfoMsg"))
    (:file "_package_InfoMsg" :depends-on ("_package"))
  ))