;; Auto-generated. Do not edit!


(when (boundp 'servo_ros::ServoMove)
  (if (not (find-package "SERVO_ROS"))
    (make-package "SERVO_ROS"))
  (shadow 'ServoMove (find-package "SERVO_ROS")))
(unless (find-package "SERVO_ROS::SERVOMOVE")
  (make-package "SERVO_ROS::SERVOMOVE"))

(in-package "ROS")
;;//! \htmlinclude ServoMove.msg.html


(defclass servo_ros::ServoMove
  :super ros::object
  :slots (_servo_id _angle _time ))

(defmethod servo_ros::ServoMove
  (:init
   (&key
    ((:servo_id __servo_id) 0)
    ((:angle __angle) 0)
    ((:time __time) 0)
    )
   (send-super :init)
   (setq _servo_id (round __servo_id))
   (setq _angle (round __angle))
   (setq _time (round __time))
   self)
  (:servo_id
   (&optional __servo_id)
   (if __servo_id (setq _servo_id __servo_id)) _servo_id)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; uint16 _servo_id
    2
    ;; uint16 _angle
    2
    ;; uint16 _time
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _servo_id
       (write-word _servo_id s)
     ;; uint16 _angle
       (write-word _angle s)
     ;; uint16 _time
       (write-word _time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _servo_id
     (setq _servo_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _angle
     (setq _angle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _time
     (setq _time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get servo_ros::ServoMove :md5sum-) "fd942452571bdf83e3fa8f79a5e9a7d3")
(setf (get servo_ros::ServoMove :datatype-) "servo_ros/ServoMove")
(setf (get servo_ros::ServoMove :definition-)
      "#舵机转动控制
uint16 servo_id	#舵机ID
uint16 angle	#角度位置0~1000
uint16 time     #运行时间(ms)

")



(provide :servo_ros/ServoMove "fd942452571bdf83e3fa8f79a5e9a7d3")


