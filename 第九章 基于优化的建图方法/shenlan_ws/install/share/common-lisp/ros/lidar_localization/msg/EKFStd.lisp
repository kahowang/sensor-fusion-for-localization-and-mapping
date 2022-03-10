; Auto-generated. Do not edit!


(cl:in-package lidar_localization-msg)


;//! \htmlinclude EKFStd.msg.html

(cl:defclass <EKFStd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pos_x_std
    :reader pos_x_std
    :initarg :pos_x_std
    :type cl:float
    :initform 0.0)
   (pos_y_std
    :reader pos_y_std
    :initarg :pos_y_std
    :type cl:float
    :initform 0.0)
   (pos_z_std
    :reader pos_z_std
    :initarg :pos_z_std
    :type cl:float
    :initform 0.0)
   (vel_x_std
    :reader vel_x_std
    :initarg :vel_x_std
    :type cl:float
    :initform 0.0)
   (vel_y_std
    :reader vel_y_std
    :initarg :vel_y_std
    :type cl:float
    :initform 0.0)
   (vel_z_std
    :reader vel_z_std
    :initarg :vel_z_std
    :type cl:float
    :initform 0.0)
   (ori_w_std
    :reader ori_w_std
    :initarg :ori_w_std
    :type cl:float
    :initform 0.0)
   (ori_x_std
    :reader ori_x_std
    :initarg :ori_x_std
    :type cl:float
    :initform 0.0)
   (ori_y_std
    :reader ori_y_std
    :initarg :ori_y_std
    :type cl:float
    :initform 0.0)
   (ori_z_std
    :reader ori_z_std
    :initarg :ori_z_std
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

(cl:defclass EKFStd (<EKFStd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EKFStd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EKFStd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-msg:<EKFStd> is deprecated: use lidar_localization-msg:EKFStd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:header-val is deprecated.  Use lidar_localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pos_x_std-val :lambda-list '(m))
(cl:defmethod pos_x_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:pos_x_std-val is deprecated.  Use lidar_localization-msg:pos_x_std instead.")
  (pos_x_std m))

(cl:ensure-generic-function 'pos_y_std-val :lambda-list '(m))
(cl:defmethod pos_y_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:pos_y_std-val is deprecated.  Use lidar_localization-msg:pos_y_std instead.")
  (pos_y_std m))

(cl:ensure-generic-function 'pos_z_std-val :lambda-list '(m))
(cl:defmethod pos_z_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:pos_z_std-val is deprecated.  Use lidar_localization-msg:pos_z_std instead.")
  (pos_z_std m))

(cl:ensure-generic-function 'vel_x_std-val :lambda-list '(m))
(cl:defmethod vel_x_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:vel_x_std-val is deprecated.  Use lidar_localization-msg:vel_x_std instead.")
  (vel_x_std m))

(cl:ensure-generic-function 'vel_y_std-val :lambda-list '(m))
(cl:defmethod vel_y_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:vel_y_std-val is deprecated.  Use lidar_localization-msg:vel_y_std instead.")
  (vel_y_std m))

(cl:ensure-generic-function 'vel_z_std-val :lambda-list '(m))
(cl:defmethod vel_z_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:vel_z_std-val is deprecated.  Use lidar_localization-msg:vel_z_std instead.")
  (vel_z_std m))

(cl:ensure-generic-function 'ori_w_std-val :lambda-list '(m))
(cl:defmethod ori_w_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:ori_w_std-val is deprecated.  Use lidar_localization-msg:ori_w_std instead.")
  (ori_w_std m))

(cl:ensure-generic-function 'ori_x_std-val :lambda-list '(m))
(cl:defmethod ori_x_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:ori_x_std-val is deprecated.  Use lidar_localization-msg:ori_x_std instead.")
  (ori_x_std m))

(cl:ensure-generic-function 'ori_y_std-val :lambda-list '(m))
(cl:defmethod ori_y_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:ori_y_std-val is deprecated.  Use lidar_localization-msg:ori_y_std instead.")
  (ori_y_std m))

(cl:ensure-generic-function 'ori_z_std-val :lambda-list '(m))
(cl:defmethod ori_z_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:ori_z_std-val is deprecated.  Use lidar_localization-msg:ori_z_std instead.")
  (ori_z_std m))

(cl:ensure-generic-function 'gyro_bias_x_std-val :lambda-list '(m))
(cl:defmethod gyro_bias_x_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:gyro_bias_x_std-val is deprecated.  Use lidar_localization-msg:gyro_bias_x_std instead.")
  (gyro_bias_x_std m))

(cl:ensure-generic-function 'gyro_bias_y_std-val :lambda-list '(m))
(cl:defmethod gyro_bias_y_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:gyro_bias_y_std-val is deprecated.  Use lidar_localization-msg:gyro_bias_y_std instead.")
  (gyro_bias_y_std m))

(cl:ensure-generic-function 'gyro_bias_z_std-val :lambda-list '(m))
(cl:defmethod gyro_bias_z_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:gyro_bias_z_std-val is deprecated.  Use lidar_localization-msg:gyro_bias_z_std instead.")
  (gyro_bias_z_std m))

(cl:ensure-generic-function 'accel_bias_x_std-val :lambda-list '(m))
(cl:defmethod accel_bias_x_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:accel_bias_x_std-val is deprecated.  Use lidar_localization-msg:accel_bias_x_std instead.")
  (accel_bias_x_std m))

(cl:ensure-generic-function 'accel_bias_y_std-val :lambda-list '(m))
(cl:defmethod accel_bias_y_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:accel_bias_y_std-val is deprecated.  Use lidar_localization-msg:accel_bias_y_std instead.")
  (accel_bias_y_std m))

(cl:ensure-generic-function 'accel_bias_z_std-val :lambda-list '(m))
(cl:defmethod accel_bias_z_std-val ((m <EKFStd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:accel_bias_z_std-val is deprecated.  Use lidar_localization-msg:accel_bias_z_std instead.")
  (accel_bias_z_std m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EKFStd>) ostream)
  "Serializes a message object of type '<EKFStd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vel_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vel_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vel_z_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_w_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_x_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_y_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_z_std))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EKFStd>) istream)
  "Deserializes a message object of type '<EKFStd>"
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
    (cl:setf (cl:slot-value msg 'pos_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_z_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_w_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_x_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_y_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_z_std) (roslisp-utils:decode-double-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EKFStd>)))
  "Returns string type for a message object of type '<EKFStd>"
  "lidar_localization/EKFStd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EKFStd)))
  "Returns string type for a message object of type 'EKFStd"
  "lidar_localization/EKFStd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EKFStd>)))
  "Returns md5sum for a message object of type '<EKFStd>"
  "2e9c1dfcaa6649033a40105a26a0f620")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EKFStd)))
  "Returns md5sum for a message object of type 'EKFStd"
  "2e9c1dfcaa6649033a40105a26a0f620")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EKFStd>)))
  "Returns full string definition for message of type '<EKFStd>"
  (cl:format cl:nil "# time of ESKF estimation:~%Header header~%~%# a. position:~%float64 pos_x_std~%float64 pos_y_std~%float64 pos_z_std~%~%# b. velocity:~%float64 vel_x_std~%float64 vel_y_std~%float64 vel_z_std~%~%# c. orientation:~%float64 ori_w_std~%float64 ori_x_std~%float64 ori_y_std~%float64 ori_z_std~%~%# d. gyro. bias:~%float64 gyro_bias_x_std~%float64 gyro_bias_y_std~%float64 gyro_bias_z_std~%~%# e. accel. bias:~%float64 accel_bias_x_std~%float64 accel_bias_y_std~%float64 accel_bias_z_std~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EKFStd)))
  "Returns full string definition for message of type 'EKFStd"
  (cl:format cl:nil "# time of ESKF estimation:~%Header header~%~%# a. position:~%float64 pos_x_std~%float64 pos_y_std~%float64 pos_z_std~%~%# b. velocity:~%float64 vel_x_std~%float64 vel_y_std~%float64 vel_z_std~%~%# c. orientation:~%float64 ori_w_std~%float64 ori_x_std~%float64 ori_y_std~%float64 ori_z_std~%~%# d. gyro. bias:~%float64 gyro_bias_x_std~%float64 gyro_bias_y_std~%float64 gyro_bias_z_std~%~%# e. accel. bias:~%float64 accel_bias_x_std~%float64 accel_bias_y_std~%float64 accel_bias_z_std~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EKFStd>))
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
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EKFStd>))
  "Converts a ROS message object to a list"
  (cl:list 'EKFStd
    (cl:cons ':header (header msg))
    (cl:cons ':pos_x_std (pos_x_std msg))
    (cl:cons ':pos_y_std (pos_y_std msg))
    (cl:cons ':pos_z_std (pos_z_std msg))
    (cl:cons ':vel_x_std (vel_x_std msg))
    (cl:cons ':vel_y_std (vel_y_std msg))
    (cl:cons ':vel_z_std (vel_z_std msg))
    (cl:cons ':ori_w_std (ori_w_std msg))
    (cl:cons ':ori_x_std (ori_x_std msg))
    (cl:cons ':ori_y_std (ori_y_std msg))
    (cl:cons ':ori_z_std (ori_z_std msg))
    (cl:cons ':gyro_bias_x_std (gyro_bias_x_std msg))
    (cl:cons ':gyro_bias_y_std (gyro_bias_y_std msg))
    (cl:cons ':gyro_bias_z_std (gyro_bias_z_std msg))
    (cl:cons ':accel_bias_x_std (accel_bias_x_std msg))
    (cl:cons ':accel_bias_y_std (accel_bias_y_std msg))
    (cl:cons ':accel_bias_z_std (accel_bias_z_std msg))
))
