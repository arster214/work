;; Auto-generated. Do not edit!


(when (boundp 'dual_arm_msgs::Joint_Max_Speed)
  (if (not (find-package "DUAL_ARM_MSGS"))
    (make-package "DUAL_ARM_MSGS"))
  (shadow 'Joint_Max_Speed (find-package "DUAL_ARM_MSGS")))
(unless (find-package "DUAL_ARM_MSGS::JOINT_MAX_SPEED")
  (make-package "DUAL_ARM_MSGS::JOINT_MAX_SPEED"))

(in-package "ROS")
;;//! \htmlinclude Joint_Max_Speed.msg.html


(defclass dual_arm_msgs::Joint_Max_Speed
  :super ros::object
  :slots (_joint_num _joint_max_speed ))

(defmethod dual_arm_msgs::Joint_Max_Speed
  (:init
   (&key
    ((:joint_num __joint_num) 0)
    ((:joint_max_speed __joint_max_speed) 0.0)
    )
   (send-super :init)
   (setq _joint_num (round __joint_num))
   (setq _joint_max_speed (float __joint_max_speed))
   self)
  (:joint_num
   (&optional __joint_num)
   (if __joint_num (setq _joint_num __joint_num)) _joint_num)
  (:joint_max_speed
   (&optional __joint_max_speed)
   (if __joint_max_speed (setq _joint_max_speed __joint_max_speed)) _joint_max_speed)
  (:serialization-length
   ()
   (+
    ;; uint8 _joint_num
    1
    ;; float32 _joint_max_speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _joint_num
       (write-byte _joint_num s)
     ;; float32 _joint_max_speed
       (sys::poke _joint_max_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _joint_num
     (setq _joint_num (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _joint_max_speed
     (setq _joint_max_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dual_arm_msgs::Joint_Max_Speed :md5sum-) "9a15b693ccbb220eba8aa0b693b24585")
(setf (get dual_arm_msgs::Joint_Max_Speed :datatype-) "dual_arm_msgs/Joint_Max_Speed")
(setf (get dual_arm_msgs::Joint_Max_Speed :definition-)
      "uint8 joint_num
float32 joint_max_speed

")



(provide :dual_arm_msgs/Joint_Max_Speed "9a15b693ccbb220eba8aa0b693b24585")


