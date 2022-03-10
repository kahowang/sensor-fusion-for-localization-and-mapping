;; Auto-generated. Do not edit!


(when (boundp 'lidar_localization::ESKFStd)
  (if (not (find-package "LIDAR_LOCALIZATION"))
    (make-package "LIDAR_LOCALIZATION"))
  (shadow 'ESKFStd (find-package "LIDAR_LOCALIZATION")))
(unless (find-package "LIDAR_LOCALIZATION::ESKFSTD")
  (make-package "LIDAR_LOCALIZATION::ESKFSTD"))

(in-package "ROS")
;;//! \htmlinclude ESKFStd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_localization::ESKFStd
  :super ros::object
  :slots (_header _delta_pos_x_std _delta_pos_y_std _delta_pos_z_std _delta_vel_x_std _delta_vel_y_std _delta_vel_z_std _delta_ori_x_std _delta_ori_y_std _delta_ori_z_std _gyro_bias_x_std _gyro_bias_y_std _gyro_bias_z_std _accel_bias_x_std _accel_bias_y_std _accel_bias_z_std ))

(defmethod lidar_localization::ESKFStd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:delta_pos_x_std __delta_pos_x_std) 0.0)
    ((:delta_pos_y_std __delta_pos_y_std) 0.0)
    ((:delta_pos_z_std __delta_pos_z_std) 0.0)
    ((:delta_vel_x_std __delta_vel_x_std) 0.0)
    ((:delta_vel_y_std __delta_vel_y_std) 0.0)
    ((:delta_vel_z_std __delta_vel_z_std) 0.0)
    ((:delta_ori_x_std __delta_ori_x_std) 0.0)
    ((:delta_ori_y_std __delta_ori_y_std) 0.0)
    ((:delta_ori_z_std __delta_ori_z_std) 0.0)
    ((:gyro_bias_x_std __gyro_bias_x_std) 0.0)
    ((:gyro_bias_y_std __gyro_bias_y_std) 0.0)
    ((:gyro_bias_z_std __gyro_bias_z_std) 0.0)
    ((:accel_bias_x_std __accel_bias_x_std) 0.0)
    ((:accel_bias_y_std __accel_bias_y_std) 0.0)
    ((:accel_bias_z_std __accel_bias_z_std) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _delta_pos_x_std (float __delta_pos_x_std))
   (setq _delta_pos_y_std (float __delta_pos_y_std))
   (setq _delta_pos_z_std (float __delta_pos_z_std))
   (setq _delta_vel_x_std (float __delta_vel_x_std))
   (setq _delta_vel_y_std (float __delta_vel_y_std))
   (setq _delta_vel_z_std (float __delta_vel_z_std))
   (setq _delta_ori_x_std (float __delta_ori_x_std))
   (setq _delta_ori_y_std (float __delta_ori_y_std))
   (setq _delta_ori_z_std (float __delta_ori_z_std))
   (setq _gyro_bias_x_std (float __gyro_bias_x_std))
   (setq _gyro_bias_y_std (float __gyro_bias_y_std))
   (setq _gyro_bias_z_std (float __gyro_bias_z_std))
   (setq _accel_bias_x_std (float __accel_bias_x_std))
   (setq _accel_bias_y_std (float __accel_bias_y_std))
   (setq _accel_bias_z_std (float __accel_bias_z_std))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:delta_pos_x_std
   (&optional __delta_pos_x_std)
   (if __delta_pos_x_std (setq _delta_pos_x_std __delta_pos_x_std)) _delta_pos_x_std)
  (:delta_pos_y_std
   (&optional __delta_pos_y_std)
   (if __delta_pos_y_std (setq _delta_pos_y_std __delta_pos_y_std)) _delta_pos_y_std)
  (:delta_pos_z_std
   (&optional __delta_pos_z_std)
   (if __delta_pos_z_std (setq _delta_pos_z_std __delta_pos_z_std)) _delta_pos_z_std)
  (:delta_vel_x_std
   (&optional __delta_vel_x_std)
   (if __delta_vel_x_std (setq _delta_vel_x_std __delta_vel_x_std)) _delta_vel_x_std)
  (:delta_vel_y_std
   (&optional __delta_vel_y_std)
   (if __delta_vel_y_std (setq _delta_vel_y_std __delta_vel_y_std)) _delta_vel_y_std)
  (:delta_vel_z_std
   (&optional __delta_vel_z_std)
   (if __delta_vel_z_std (setq _delta_vel_z_std __delta_vel_z_std)) _delta_vel_z_std)
  (:delta_ori_x_std
   (&optional __delta_ori_x_std)
   (if __delta_ori_x_std (setq _delta_ori_x_std __delta_ori_x_std)) _delta_ori_x_std)
  (:delta_ori_y_std
   (&optional __delta_ori_y_std)
   (if __delta_ori_y_std (setq _delta_ori_y_std __delta_ori_y_std)) _delta_ori_y_std)
  (:delta_ori_z_std
   (&optional __delta_ori_z_std)
   (if __delta_ori_z_std (setq _delta_ori_z_std __delta_ori_z_std)) _delta_ori_z_std)
  (:gyro_bias_x_std
   (&optional __gyro_bias_x_std)
   (if __gyro_bias_x_std (setq _gyro_bias_x_std __gyro_bias_x_std)) _gyro_bias_x_std)
  (:gyro_bias_y_std
   (&optional __gyro_bias_y_std)
   (if __gyro_bias_y_std (setq _gyro_bias_y_std __gyro_bias_y_std)) _gyro_bias_y_std)
  (:gyro_bias_z_std
   (&optional __gyro_bias_z_std)
   (if __gyro_bias_z_std (setq _gyro_bias_z_std __gyro_bias_z_std)) _gyro_bias_z_std)
  (:accel_bias_x_std
   (&optional __accel_bias_x_std)
   (if __accel_bias_x_std (setq _accel_bias_x_std __accel_bias_x_std)) _accel_bias_x_std)
  (:accel_bias_y_std
   (&optional __accel_bias_y_std)
   (if __accel_bias_y_std (setq _accel_bias_y_std __accel_bias_y_std)) _accel_bias_y_std)
  (:accel_bias_z_std
   (&optional __accel_bias_z_std)
   (if __accel_bias_z_std (setq _accel_bias_z_std __accel_bias_z_std)) _accel_bias_z_std)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _delta_pos_x_std
    8
    ;; float64 _delta_pos_y_std
    8
    ;; float64 _delta_pos_z_std
    8
    ;; float64 _delta_vel_x_std
    8
    ;; float64 _delta_vel_y_std
    8
    ;; float64 _delta_vel_z_std
    8
    ;; float64 _delta_ori_x_std
    8
    ;; float64 _delta_ori_y_std
    8
    ;; float64 _delta_ori_z_std
    8
    ;; float64 _gyro_bias_x_std
    8
    ;; float64 _gyro_bias_y_std
    8
    ;; float64 _gyro_bias_z_std
    8
    ;; float64 _accel_bias_x_std
    8
    ;; float64 _accel_bias_y_std
    8
    ;; float64 _accel_bias_z_std
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _delta_pos_x_std
       (sys::poke _delta_pos_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_pos_y_std
       (sys::poke _delta_pos_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_pos_z_std
       (sys::poke _delta_pos_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_vel_x_std
       (sys::poke _delta_vel_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_vel_y_std
       (sys::poke _delta_vel_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_vel_z_std
       (sys::poke _delta_vel_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_ori_x_std
       (sys::poke _delta_ori_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_ori_y_std
       (sys::poke _delta_ori_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_ori_z_std
       (sys::poke _delta_ori_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_bias_x_std
       (sys::poke _gyro_bias_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_bias_y_std
       (sys::poke _gyro_bias_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_bias_z_std
       (sys::poke _gyro_bias_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_bias_x_std
       (sys::poke _accel_bias_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_bias_y_std
       (sys::poke _accel_bias_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_bias_z_std
       (sys::poke _accel_bias_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _delta_pos_x_std
     (setq _delta_pos_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_pos_y_std
     (setq _delta_pos_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_pos_z_std
     (setq _delta_pos_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_vel_x_std
     (setq _delta_vel_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_vel_y_std
     (setq _delta_vel_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_vel_z_std
     (setq _delta_vel_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_ori_x_std
     (setq _delta_ori_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_ori_y_std
     (setq _delta_ori_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_ori_z_std
     (setq _delta_ori_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_bias_x_std
     (setq _gyro_bias_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_bias_y_std
     (setq _gyro_bias_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_bias_z_std
     (setq _gyro_bias_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_bias_x_std
     (setq _accel_bias_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_bias_y_std
     (setq _accel_bias_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_bias_z_std
     (setq _accel_bias_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get lidar_localization::ESKFStd :md5sum-) "ab13091af10d5ae8e76adaf8e34014b3")
(setf (get lidar_localization::ESKFStd :datatype-) "lidar_localization/ESKFStd")
(setf (get lidar_localization::ESKFStd :definition-)
      "# time of ESKF estimation:
Header header

# a. position:
float64 delta_pos_x_std
float64 delta_pos_y_std
float64 delta_pos_z_std

# b. velocity:
float64 delta_vel_x_std
float64 delta_vel_y_std
float64 delta_vel_z_std

# c. orientation:
float64 delta_ori_x_std
float64 delta_ori_y_std
float64 delta_ori_z_std

# d. gyro. bias:
float64 gyro_bias_x_std
float64 gyro_bias_y_std
float64 gyro_bias_z_std

# e. accel. bias:
float64 accel_bias_x_std
float64 accel_bias_y_std
float64 accel_bias_z_std
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :lidar_localization/ESKFStd "ab13091af10d5ae8e76adaf8e34014b3")


