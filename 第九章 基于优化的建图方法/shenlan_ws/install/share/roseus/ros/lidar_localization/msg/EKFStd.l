;; Auto-generated. Do not edit!


(when (boundp 'lidar_localization::EKFStd)
  (if (not (find-package "LIDAR_LOCALIZATION"))
    (make-package "LIDAR_LOCALIZATION"))
  (shadow 'EKFStd (find-package "LIDAR_LOCALIZATION")))
(unless (find-package "LIDAR_LOCALIZATION::EKFSTD")
  (make-package "LIDAR_LOCALIZATION::EKFSTD"))

(in-package "ROS")
;;//! \htmlinclude EKFStd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_localization::EKFStd
  :super ros::object
  :slots (_header _pos_x_std _pos_y_std _pos_z_std _vel_x_std _vel_y_std _vel_z_std _ori_w_std _ori_x_std _ori_y_std _ori_z_std _gyro_bias_x_std _gyro_bias_y_std _gyro_bias_z_std _accel_bias_x_std _accel_bias_y_std _accel_bias_z_std ))

(defmethod lidar_localization::EKFStd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pos_x_std __pos_x_std) 0.0)
    ((:pos_y_std __pos_y_std) 0.0)
    ((:pos_z_std __pos_z_std) 0.0)
    ((:vel_x_std __vel_x_std) 0.0)
    ((:vel_y_std __vel_y_std) 0.0)
    ((:vel_z_std __vel_z_std) 0.0)
    ((:ori_w_std __ori_w_std) 0.0)
    ((:ori_x_std __ori_x_std) 0.0)
    ((:ori_y_std __ori_y_std) 0.0)
    ((:ori_z_std __ori_z_std) 0.0)
    ((:gyro_bias_x_std __gyro_bias_x_std) 0.0)
    ((:gyro_bias_y_std __gyro_bias_y_std) 0.0)
    ((:gyro_bias_z_std __gyro_bias_z_std) 0.0)
    ((:accel_bias_x_std __accel_bias_x_std) 0.0)
    ((:accel_bias_y_std __accel_bias_y_std) 0.0)
    ((:accel_bias_z_std __accel_bias_z_std) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pos_x_std (float __pos_x_std))
   (setq _pos_y_std (float __pos_y_std))
   (setq _pos_z_std (float __pos_z_std))
   (setq _vel_x_std (float __vel_x_std))
   (setq _vel_y_std (float __vel_y_std))
   (setq _vel_z_std (float __vel_z_std))
   (setq _ori_w_std (float __ori_w_std))
   (setq _ori_x_std (float __ori_x_std))
   (setq _ori_y_std (float __ori_y_std))
   (setq _ori_z_std (float __ori_z_std))
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
  (:pos_x_std
   (&optional __pos_x_std)
   (if __pos_x_std (setq _pos_x_std __pos_x_std)) _pos_x_std)
  (:pos_y_std
   (&optional __pos_y_std)
   (if __pos_y_std (setq _pos_y_std __pos_y_std)) _pos_y_std)
  (:pos_z_std
   (&optional __pos_z_std)
   (if __pos_z_std (setq _pos_z_std __pos_z_std)) _pos_z_std)
  (:vel_x_std
   (&optional __vel_x_std)
   (if __vel_x_std (setq _vel_x_std __vel_x_std)) _vel_x_std)
  (:vel_y_std
   (&optional __vel_y_std)
   (if __vel_y_std (setq _vel_y_std __vel_y_std)) _vel_y_std)
  (:vel_z_std
   (&optional __vel_z_std)
   (if __vel_z_std (setq _vel_z_std __vel_z_std)) _vel_z_std)
  (:ori_w_std
   (&optional __ori_w_std)
   (if __ori_w_std (setq _ori_w_std __ori_w_std)) _ori_w_std)
  (:ori_x_std
   (&optional __ori_x_std)
   (if __ori_x_std (setq _ori_x_std __ori_x_std)) _ori_x_std)
  (:ori_y_std
   (&optional __ori_y_std)
   (if __ori_y_std (setq _ori_y_std __ori_y_std)) _ori_y_std)
  (:ori_z_std
   (&optional __ori_z_std)
   (if __ori_z_std (setq _ori_z_std __ori_z_std)) _ori_z_std)
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
    ;; float64 _pos_x_std
    8
    ;; float64 _pos_y_std
    8
    ;; float64 _pos_z_std
    8
    ;; float64 _vel_x_std
    8
    ;; float64 _vel_y_std
    8
    ;; float64 _vel_z_std
    8
    ;; float64 _ori_w_std
    8
    ;; float64 _ori_x_std
    8
    ;; float64 _ori_y_std
    8
    ;; float64 _ori_z_std
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
     ;; float64 _pos_x_std
       (sys::poke _pos_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_y_std
       (sys::poke _pos_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_z_std
       (sys::poke _pos_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_x_std
       (sys::poke _vel_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_y_std
       (sys::poke _vel_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_z_std
       (sys::poke _vel_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ori_w_std
       (sys::poke _ori_w_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ori_x_std
       (sys::poke _ori_x_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ori_y_std
       (sys::poke _ori_y_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ori_z_std
       (sys::poke _ori_z_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64 _pos_x_std
     (setq _pos_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_y_std
     (setq _pos_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_z_std
     (setq _pos_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_x_std
     (setq _vel_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_y_std
     (setq _vel_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_z_std
     (setq _vel_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ori_w_std
     (setq _ori_w_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ori_x_std
     (setq _ori_x_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ori_y_std
     (setq _ori_y_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ori_z_std
     (setq _ori_z_std (sys::peek buf ptr- :double)) (incf ptr- 8)
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

(setf (get lidar_localization::EKFStd :md5sum-) "2e9c1dfcaa6649033a40105a26a0f620")
(setf (get lidar_localization::EKFStd :datatype-) "lidar_localization/EKFStd")
(setf (get lidar_localization::EKFStd :definition-)
      "# time of ESKF estimation:
Header header

# a. position:
float64 pos_x_std
float64 pos_y_std
float64 pos_z_std

# b. velocity:
float64 vel_x_std
float64 vel_y_std
float64 vel_z_std

# c. orientation:
float64 ori_w_std
float64 ori_x_std
float64 ori_y_std
float64 ori_z_std

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



(provide :lidar_localization/EKFStd "2e9c1dfcaa6649033a40105a26a0f620")


