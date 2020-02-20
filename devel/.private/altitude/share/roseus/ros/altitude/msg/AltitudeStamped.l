;; Auto-generated. Do not edit!


(when (boundp 'altitude::AltitudeStamped)
  (if (not (find-package "ALTITUDE"))
    (make-package "ALTITUDE"))
  (shadow 'AltitudeStamped (find-package "ALTITUDE")))
(unless (find-package "ALTITUDE::ALTITUDESTAMPED")
  (make-package "ALTITUDE::ALTITUDESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude AltitudeStamped.msg.html


(defclass altitude::AltitudeStamped
  :super ros::object
  :slots (_value _stamp ))

(defmethod altitude::AltitudeStamped
  (:init
   (&key
    ((:value __value) 0.0)
    ((:stamp __stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _value (float __value))
   (setq _stamp __stamp)
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:serialization-length
   ()
   (+
    ;; float64 _value
    8
    ;; time _stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get altitude::AltitudeStamped :md5sum-) "0402997c6c1a6ed75158f906f42e8944")
(setf (get altitude::AltitudeStamped :datatype-) "altitude/AltitudeStamped")
(setf (get altitude::AltitudeStamped :definition-)
      "float64 value
time stamp

")



(provide :altitude/AltitudeStamped "0402997c6c1a6ed75158f906f42e8944")


