;; Auto-generated. Do not edit!


(when (boundp 'message::EndOfActionMsg)
  (if (not (find-package "MESSAGE"))
    (make-package "MESSAGE"))
  (shadow 'EndOfActionMsg (find-package "MESSAGE")))
(unless (find-package "MESSAGE::ENDOFACTIONMSG")
  (make-package "MESSAGE::ENDOFACTIONMSG"))

(in-package "ROS")
;;//! \htmlinclude EndOfActionMsg.msg.html


(defclass message::EndOfActionMsg
  :super ros::object
  :slots (_exit _reason ))

(defmethod message::EndOfActionMsg
  (:init
   (&key
    ((:exit __exit) 0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _exit (round __exit))
   (setq _reason (string __reason))
   self)
  (:exit
   (&optional __exit)
   (if __exit (setq _exit __exit)) _exit)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; int16 _exit
    2
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _exit
       (write-word _exit s)
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _exit
     (setq _exit (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get message::EndOfActionMsg :md5sum-) "6620ca083d96e08d9ae896bc1472aad2")
(setf (get message::EndOfActionMsg :datatype-) "message/EndOfActionMsg")
(setf (get message::EndOfActionMsg :definition-)
      "int16 exit
string reason
")



(provide :message/EndOfActionMsg "6620ca083d96e08d9ae896bc1472aad2")


