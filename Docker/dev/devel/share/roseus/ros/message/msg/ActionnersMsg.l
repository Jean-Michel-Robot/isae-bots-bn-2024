;; Auto-generated. Do not edit!


(when (boundp 'message::ActionnersMsg)
  (if (not (find-package "MESSAGE"))
    (make-package "MESSAGE"))
  (shadow 'ActionnersMsg (find-package "MESSAGE")))
(unless (find-package "MESSAGE::ACTIONNERSMSG")
  (make-package "MESSAGE::ACTIONNERSMSG"))

(in-package "ROS")
;;//! \htmlinclude ActionnersMsg.msg.html


(defclass message::ActionnersMsg
  :super ros::object
  :slots (_act _fail _end ))

(defmethod message::ActionnersMsg
  (:init
   (&key
    ((:act __act) "")
    ((:fail __fail) nil)
    ((:end __end) nil)
    )
   (send-super :init)
   (setq _act (string __act))
   (setq _fail __fail)
   (setq _end __end)
   self)
  (:act
   (&optional __act)
   (if __act (setq _act __act)) _act)
  (:fail
   (&optional (__fail :null))
   (if (not (eq __fail :null)) (setq _fail __fail)) _fail)
  (:end
   (&optional (__end :null))
   (if (not (eq __end :null)) (setq _end __end)) _end)
  (:serialization-length
   ()
   (+
    ;; string _act
    4 (length _act)
    ;; bool _fail
    1
    ;; bool _end
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _act
       (write-long (length _act) s) (princ _act s)
     ;; bool _fail
       (if _fail (write-byte -1 s) (write-byte 0 s))
     ;; bool _end
       (if _end (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _act
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _act (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _fail
     (setq _fail (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _end
     (setq _end (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get message::ActionnersMsg :md5sum-) "3dd2e8d97f70009cf8e47cbf3e745186")
(setf (get message::ActionnersMsg :datatype-) "message/ActionnersMsg")
(setf (get message::ActionnersMsg :definition-)
      "string act
bool fail
bool end
")



(provide :message/ActionnersMsg "3dd2e8d97f70009cf8e47cbf3e745186")


