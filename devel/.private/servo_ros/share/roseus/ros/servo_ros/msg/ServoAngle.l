;; Auto-generated. Do not edit!


(when (boundp 'servo_ros::ServoAngle)
  (if (not (find-package "SERVO_ROS"))
    (make-package "SERVO_ROS"))
  (shadow 'ServoAngle (find-package "SERVO_ROS")))
(unless (find-package "SERVO_ROS::SERVOANGLE")
  (make-package "SERVO_ROS::SERVOANGLE"))

(in-package "ROS")
;;//! \htmlinclude ServoAngle.msg.html


(defclass servo_ros::ServoAngle
  :super ros::object
  :slots (_servo_id_1 _angle_1 _servo_id_2 _angle_2 ))

(defmethod servo_ros::ServoAngle
  (:init
   (&key
    ((:servo_id_1 __servo_id_1) 0)
    ((:angle_1 __angle_1) 0)
    ((:servo_id_2 __servo_id_2) 0)
    ((:angle_2 __angle_2) 0)
    )
   (send-super :init)
   (setq _servo_id_1 (round __servo_id_1))
   (setq _angle_1 (round __angle_1))
   (setq _servo_id_2 (round __servo_id_2))
   (setq _angle_2 (round __angle_2))
   self)
  (:servo_id_1
   (&optional __servo_id_1)
   (if __servo_id_1 (setq _servo_id_1 __servo_id_1)) _servo_id_1)
  (:angle_1
   (&optional __angle_1)
   (if __angle_1 (setq _angle_1 __angle_1)) _angle_1)
  (:servo_id_2
   (&optional __servo_id_2)
   (if __servo_id_2 (setq _servo_id_2 __servo_id_2)) _servo_id_2)
  (:angle_2
   (&optional __angle_2)
   (if __angle_2 (setq _angle_2 __angle_2)) _angle_2)
  (:serialization-length
   ()
   (+
    ;; uint16 _servo_id_1
    2
    ;; uint16 _angle_1
    2
    ;; uint16 _servo_id_2
    2
    ;; uint16 _angle_2
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _servo_id_1
       (write-word _servo_id_1 s)
     ;; uint16 _angle_1
       (write-word _angle_1 s)
     ;; uint16 _servo_id_2
       (write-word _servo_id_2 s)
     ;; uint16 _angle_2
       (write-word _angle_2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _servo_id_1
     (setq _servo_id_1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _angle_1
     (setq _angle_1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _servo_id_2
     (setq _servo_id_2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _angle_2
     (setq _angle_2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get servo_ros::ServoAngle :md5sum-) "9a8103b4cee119f9447f2067818f6250")
(setf (get servo_ros::ServoAngle :datatype-) "servo_ros/ServoAngle")
(setf (get servo_ros::ServoAngle :definition-)
      "#舵机返回角度
uint16 servo_id_1	#舵机ID
uint16 angle_1	#角度位置0~1000
uint16 servo_id_2	#舵机ID
uint16 angle_2	#角度位置0~1000

")



(provide :servo_ros/ServoAngle "9a8103b4cee119f9447f2067818f6250")


