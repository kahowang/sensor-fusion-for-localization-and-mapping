; Auto-generated. Do not edit!


(cl:in-package lidar_localization-msg)


;//! \htmlinclude ESKFStd.msg.html

(cl:defclass <ESKFStd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (delta_pos_x_std
    :reader delta_pos_x_std
    :initarg :delta_pos_x_std
    :type cl:float
    :initform 0.0)
   (delta_pos_y_std
    :reader delta_pos_y_std
    :initarg :delta_pos_y_std
    :type cl:float
    :initform 0.0)
   (delta_pos_z_std
    :reader delta_pos_z_std
    :initarg :delta_pos_z_std
    :type cl:float
    :initform 0.0)
   (delta_vel_x_std
    :reader delta_vel_x_std
    :initarg :delta_vel_x_std
    :type cl:float
    :initform 0.0)
   (delta_vel_y_std
    :reader delta_vel_y_std
    :initarg :delta_vel_y_std
    :type cl:float
    :initform 0.0)
   (delta_vel_z_std
    :reader delta_vel_z_std
    :initarg :delta_vel_z_std
    :type cl:float
    :initform 0.0)
   (delta_ori_x_std
    :reader delta_ori_x_std
    :initarg :delta_ori_x_std
    :type cl:float
    :initform 0.0)
   (delta_ori_y_std
    :reader delta_ori_y_std
    :initarg :delta_ori_y_std
    :type cl:float
    :initform 0.0)
   (delta_ori_z_std
    :reader delta_ori_z_std
    :initarg :delta_ori_z_std
    :type cl:float
    :initform 0.0)
   (gyro_bias_x_std
    :reader gyro_bias_x_std
    :initarg :gyro_bias_x_std
    :type cl:float
    :initform 0.0)
   (gyro_bias_y_std
    :reader gyro_bias_y_std
    :initarg :gyro_bias_y_std
    :type cl:float
    :initform 0.0)
   (gyro_bias_z_std
    :reader gyro_bias_z_std
    :initarg :gyro_bias_z_std
    :type cl:float
    :initform 0.0)
   (accel_bias_x_std
    :reader accel_bias_x_std
    :initarg :accel_bias_x_std
    :type cl:float
    :initform 0.0)
   (accel_bias_y_std
    :reader accel_bias_y_std
    :initarg :accel_bias_y_std
    :type cl:float
    :initform 0.0)
   (accel_bias_z_std
    :reader accel_bias_z_std
    :initarg :accel_bias_z_std
    :type cl:float
    :initform 0.0))
)

(cl:defclass ESKFStd (<ESKFStd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ESKFStd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ESKFStd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-msg:<ESKFStd> is deprecated: use lidar_localization-msg:ESKFStd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:header-val is deprecated.  Use lidar_localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'delta_pos_x_std-val :lambda-list '(m))
(cl:defmethod delta_pos_x_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_pos_x_std-val is deprecated.  Use lidar_localization-msg:delta_pos_x_std instead.")
  (delta_pos_x_std m))

(cl:ensure-generic-function 'delta_pos_y_std-val :lambda-list '(m))
(cl:defmethod delta_pos_y_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_pos_y_std-val is deprecated.  Use lidar_localization-msg:delta_pos_y_std instead.")
  (delta_pos_y_std m))

(cl:ensure-generic-function 'delta_pos_z_std-val :lambda-list '(m))
(cl:defmethod delta_pos_z_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_pos_z_std-val is deprecated.  Use lidar_localization-msg:delta_pos_z_std instead.")
  (delta_pos_z_std m))

(cl:ensure-generic-function 'delta_vel_x_std-val :lambda-list '(m))
(cl:defmethod delta_vel_x_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_vel_x_std-val is deprecated.  Use lidar_localization-msg:delta_vel_x_std instead.")
  (delta_vel_x_std m))

(cl:ensure-generic-function 'delta_vel_y_std-val :lambda-list '(m))
(cl:defmethod delta_vel_y_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_vel_y_std-val is deprecated.  Use lidar_localization-msg:delta_vel_y_std instead.")
  (delta_vel_y_std m))

(cl:ensure-generic-function 'delta_vel_z_std-val :lambda-list '(m))
(cl:defmethod delta_vel_z_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_vel_z_std-val is deprecated.  Use lidar_localization-msg:delta_vel_z_std instead.")
  (delta_vel_z_std m))

(cl:ensure-generic-function 'delta_ori_x_std-val :lambda-list '(m))
(cl:defmethod delta_ori_x_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_ori_x_std-val is deprecated.  Use lidar_localization-msg:delta_ori_x_std instead.")
  (delta_ori_x_std m))

(cl:ensure-generic-function 'delta_ori_y_std-val :lambda-list '(m))
(cl:defmethod delta_ori_y_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_ori_y_std-val is deprecated.  Use lidar_localization-msg:delta_ori_y_std instead.")
  (delta_ori_y_std m))

(cl:ensure-generic-function 'delta_ori_z_std-val :lambda-list '(m))
(cl:defmethod delta_ori_z_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:delta_ori_z_std-val is deprecated.  Use lidar_localization-msg:delta_ori_z_std instead.")
  (delta_ori_z_std m))

(cl:ensure-generic-function 'gyro_bias_x_std-val :lambda-list '(m))
(cl:defmethod gyro_bias_x_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:gyro_bias_x_std-val is deprecated.  Use lidar_localization-msg:gyro_bias_x_std instead.")
  (gyro_bias_x_std m))

(cl:ensure-generic-function 'gyro_bias_y_std-val :lambda-list '(m))
(cl:defmethod gyro_bias_y_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:gyro_bias_y_std-val is deprecated.  Use lidar_localization-msg:gyro_bias_y_std instead.")
  (gyro_bias_y_std m))

(cl:ensure-generic-function 'gyro_bias_z_std-val :lambda-list '(m))
(cl:defmethod gyro_bias_z_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:gyro_bias_z_std-val is deprecated.  Use lidar_localization-msg:gyro_bias_z_std instead.")
  (gyro_bias_z_std m))

(cl:ensure-generic-function 'accel_bias_x_std-val :lambda-list '(m))
(cl:defmethod accel_bias_x_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:accel_bias_x_std-val is deprecated.  Use lidar_localization-msg:accel_bias_x_std instead.")
  (accel_bias_x_std m))

(cl:ensure-generic-function 'accel_bias_y_std-val :lambda-list '(m))
(cl:defmethod accel_bias_y_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:accel_bias_y_std-val is deprecated.  Use lidar_localization-msg:accel_bias_y_std instead.")
  (accel_bias_y_std m))

(cl:ensure-generic-function 'accel_bias_z_std-val :lambda-list '(m))
(cl:defmethod accel_bias_z_std-val ((m <ESKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:accel_bias_z_std-val is deprecated.  Use lidar_localization-msg:accel_bias_z_std instead.")
  (accel_bias_z_std m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ESKFStd>) ostream)
  "Serializes a message object of type '<ESKFStd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_pos_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_pos_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_pos_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_vel_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_vel_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_vel_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_ori_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_ori_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_ori_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gyro_bias_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gyro_bias_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gyro_bias_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'accel_bias_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'accel_bias_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'accel_bias_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ESKFStd>) istream)
  "Deserializes a message object of type '<ESKFStd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_pos_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_pos_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_pos_z_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vel_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vel_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vel_z_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_ori_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_ori_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_ori_z_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyro_bias_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyro_bias_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyro_bias_z_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accel_bias_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accel_bias_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accel_bias_z_std) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ESKFStd>)))
  "Returns string type for a message object of type '<ESKFStd>"
  "lidar_localization/ESKFStd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ESKFStd)))
  "Returns string type for a message object of type 'ESKFStd"
  "lidar_localization/ESKFStd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ESKFStd>)))
  "Returns md5sum for a message object of type '<ESKFStd>"
  "ab13091af10d5ae8e76adaf8e34014b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ESKFStd)))
  "Returns md5sum for a message object of type 'ESKFStd"
  "ab13091af10d5ae8e76adaf8e34014b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ESKFStd>)))
  "Returns full string definition for message of type '<ESKFStd>"
  (cl:format cl:nil "# time of ESKF estimation:~%Header header~%~%# a. position:~%float64 delta_pos_x_std~%float64 delta_pos_y_std~%float64 delta_pos_z_std~%~%# b. velocity:~%float64 delta_vel_x_std~%float64 delta_vel_y_std~%float64 delta_vel_z_std~%~%# c. orientation:~%float64 delta_ori_x_std~%float64 delta_ori_y_std~%float64 delta_ori_z_std~%~%# d. gyro. bias:~%float64 gyro_bias_x_std~%float64 gyro_bias_y_std~%float64 gyro_bias_z_std~%~%# e. accel. bias:~%float64 accel_bias_x_std~%float64 accel_bias_y_std~%float64 accel_bias_z_std~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ESKFStd)))
  "Returns full string definition for message of type 'ESKFStd"
  (cl:format cl:nil "# time of ESKF estimation:~%Header header~%~%# a. position:~%float64 delta_pos_x_std~%float64 delta_pos_y_std~%float64 delta_pos_z_std~%~%# b. velocity:~%float64 delta_vel_x_std~%float64 delta_vel_y_std~%float64 delta_vel_z_std~%~%# c. orientation:~%float64 delta_ori_x_std~%float64 delta_ori_y_std~%float64 delta_ori_z_std~%~%# d. gyro. bias:~%float64 gyro_bias_x_std~%float64 gyro_bias_y_std~%float64 gyro_bias_z_std~%~%# e. accel. bias:~%float64 accel_bias_x_std~%float64 accel_bias_y_std~%float64 accel_bias_z_std~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ESKFStd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ESKFStd>))
  "Converts a ROS message object to a list"
  (cl:list 'ESKFStd
    (cl:cons ':header (header msg))
    (cl:cons ':delta_pos_x_std (delta_pos_x_std msg))
    (cl:cons ':delta_pos_y_std (delta_pos_y_std msg))
    (cl:cons ':delta_pos_z_std (delta_pos_z_std msg))
    (cl:cons ':delta_vel_x_std (delta_vel_x_std msg))
    (cl:cons ':delta_vel_y_std (delta_vel_y_std msg))
    (cl:cons ':delta_vel_z_std (delta_vel_z_std msg))
    (cl:cons ':delta_ori_x_std (delta_ori_x_std msg))
    (cl:cons ':delta_ori_y_std (delta_ori_y_std msg))
    (cl:cons ':delta_ori_z_std (delta_ori_z_std msg))
    (cl:cons ':gyro_bias_x_std (gyro_bias_x_std msg))
    (cl:cons ':gyro_bias_y_std (gyro_bias_y_std msg))
    (cl:cons ':gyro_bias_z_std (gyro_bias_z_std msg))
    (cl:cons ':accel_bias_x_std (accel_bias_x_std msg))
    (cl:cons ':accel_bias_y_std (accel_bias_y_std msg))
    (cl:cons ':accel_bias_z_std (accel_bias_z_std msg))
))
