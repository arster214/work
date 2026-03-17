;; Auto-generated. Do not edit!


(when (boundp 'dual_arm_msgs::CarteFdPose)
  (if (not (find-package "DUAL_ARM_MSGS"))
    (make-package "DUAL_ARM_MSGS"))
  (shadow 'CarteFdPose (find-package "DUAL_ARM_MSGS")))
(unless (find-package "DUAL_ARM_MSGS::CARTEFDPOSE")
  (make-package "DUAL_ARM_MSGS::CARTEFDPOSE"))

(in-package "ROS")
;;//! \htmlinclude CarteFdPose.msg.html


(defclass dual_arm_msgs::CarteFdPose
  :super ros::object
  :slots (_Pose_Euler ))

(defmethod dual_arm_msgs::CarteFdPose
  (:init
   (&key
    ((:Pose_Euler __Pose_Euler) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _Pose_Euler __Pose_Euler)
   self)
  (:Pose_Euler
   (&optional __Pose_Euler)
   (if __Pose_Euler (setq _Pose_Euler __Pose_Euler)) _Pose_Euler)
  (:serialization-length
   ()
   (+
    ;; float32[6] _Pose_Euler
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[6] _Pose_Euler
     (dotimes (i 6)
       (sys::poke (elt _Pose_Euler i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[6] _Pose_Euler
   (dotimes (i (length _Pose_Euler))
     (setf (elt _Pose_Euler i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get dual_arm_msgs::CarteFdPose :md5sum-) "525ed60ead11361f867b19c646e09073")
(setf (get dual_arm_msgs::CarteFdPose :datatype-) "dual_arm_msgs/CarteFdPose")
(setf (get dual_arm_msgs::CarteFdPose :definition-)
      "float32[6] Pose_Euler

")



(provide :dual_arm_msgs/CarteFdPose "525ed60ead11361f867b19c646e09073")


