;; Auto-generated. Do not edit!


(when (boundp 'dual_arm_msgs::Arm_Pose_Euler)
  (if (not (find-package "DUAL_ARM_MSGS"))
    (make-package "DUAL_ARM_MSGS"))
  (shadow 'Arm_Pose_Euler (find-package "DUAL_ARM_MSGS")))
(unless (find-package "DUAL_ARM_MSGS::ARM_POSE_EULER")
  (make-package "DUAL_ARM_MSGS::ARM_POSE_EULER"))

(in-package "ROS")
;;//! \htmlinclude Arm_Pose_Euler.msg.html


(defclass dual_arm_msgs::Arm_Pose_Euler
  :super ros::object
  :slots (_Px _Py _Pz _Rx _Ry _Rz ))

(defmethod dual_arm_msgs::Arm_Pose_Euler
  (:init
   (&key
    ((:Px __Px) 0.0)
    ((:Py __Py) 0.0)
    ((:Pz __Pz) 0.0)
    ((:Rx __Rx) 0.0)
    ((:Ry __Ry) 0.0)
    ((:Rz __Rz) 0.0)
    )
   (send-super :init)
   (setq _Px (float __Px))
   (setq _Py (float __Py))
   (setq _Pz (float __Pz))
   (setq _Rx (float __Rx))
   (setq _Ry (float __Ry))
   (setq _Rz (float __Rz))
   self)
  (:Px
   (&optional __Px)
   (if __Px (setq _Px __Px)) _Px)
  (:Py
   (&optional __Py)
   (if __Py (setq _Py __Py)) _Py)
  (:Pz
   (&optional __Pz)
   (if __Pz (setq _Pz __Pz)) _Pz)
  (:Rx
   (&optional __Rx)
   (if __Rx (setq _Rx __Rx)) _Rx)
  (:Ry
   (&optional __Ry)
   (if __Ry (setq _Ry __Ry)) _Ry)
  (:Rz
   (&optional __Rz)
   (if __Rz (setq _Rz __Rz)) _Rz)
  (:serialization-length
   ()
   (+
    ;; float32 _Px
    4
    ;; float32 _Py
    4
    ;; float32 _Pz
    4
    ;; float32 _Rx
    4
    ;; float32 _Ry
    4
    ;; float32 _Rz
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _Px
       (sys::poke _Px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Py
       (sys::poke _Py (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Pz
       (sys::poke _Pz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Rx
       (sys::poke _Rx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Ry
       (sys::poke _Ry (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Rz
       (sys::poke _Rz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _Px
     (setq _Px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Py
     (setq _Py (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Pz
     (setq _Pz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Rx
     (setq _Rx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Ry
     (setq _Ry (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Rz
     (setq _Rz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dual_arm_msgs::Arm_Pose_Euler :md5sum-) "14cf69e90ba61a0c7ae2933996244f58")
(setf (get dual_arm_msgs::Arm_Pose_Euler :datatype-) "dual_arm_msgs/Arm_Pose_Euler")
(setf (get dual_arm_msgs::Arm_Pose_Euler :definition-)
      "float32 Px
float32 Py
float32 Pz
float32 Rx
float32 Ry
float32 Rz

")



(provide :dual_arm_msgs/Arm_Pose_Euler "14cf69e90ba61a0c7ae2933996244f58")


