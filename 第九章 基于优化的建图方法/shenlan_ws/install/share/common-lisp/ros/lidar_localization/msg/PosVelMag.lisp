; Auto-generated. Do not edit!


(cl:in-package lidar_localization-msg)


;//! \htmlinclude PosVelMag.msg.html

(cl:defclass <PosVelMag> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (child_frame_id
    :reader child_frame_id
    :initarg :child_frame_id
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (magnetic_field
    :reader magnetic_field
    :initarg :magnetic_field
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass PosVelMag (<PosVelMag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PosVelMag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PosVelMag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-msg:<PosVelMag> is deprecated: use lidar_localization-msg:PosVelMag instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PosVelMag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:header-val is deprecated.  Use lidar_localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'child_frame_id-val :lambda-list '(m))
(cl:defmethod child_frame_id-val ((m <PosVelMag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:child_frame_id-val is deprecated.  Use lidar_localization-msg:child_frame_id instead.")
  (child_frame_id m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <PosVelMag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:position-val is deprecated.  Use lidar_localization-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <PosVelMag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:velocity-val is deprecated.  Use lidar_localization-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'magnetic_field-val :lambda-list '(m))
(cl:defmethod magnetic_field-val ((m <PosVelMag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-msg:magnetic_field-val is deprecated.  Use lidar_localization-msg:magnetic_field instead.")
  (magnetic_field m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PosVelMag>) ostream)
  "Serializes a message object of type '<PosVelMag>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'child_frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'child_frame_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'magnetic_field) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PosVelMag>) istream)
  "Deserializes a message object of type '<PosVelMag>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'child_frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'child_frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'magnetic_field) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PosVelMag>)))
  "Returns string type for a message object of type '<PosVelMag>"
  "lidar_localization/PosVelMag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PosVelMag)))
  "Returns string type for a message object of type 'PosVelMag"
  "lidar_localization/PosVelMag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PosVelMag>)))
  "Returns md5sum for a message object of type '<PosVelMag>"
  "38da10ee57d20af73a575d4db66a11c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PosVelMag)))
  "Returns md5sum for a message object of type 'PosVelMag"
  "38da10ee57d20af73a575d4db66a11c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PosVelMag>)))
  "Returns full string definition for message of type '<PosVelMag>"
  (cl:format cl:nil "# timestamp of synced GNSS-odo measurement:~%Header header~%~%string child_frame_id~%~%# a. position:~%geometry_msgs/Point position~%~%# b. velocity:~%geometry_msgs/Vector3 velocity~%~%# c. magnetic field:~%geometry_msgs/Vector3 magnetic_field~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PosVelMag)))
  "Returns full string definition for message of type 'PosVelMag"
  (cl:format cl:nil "# timestamp of synced GNSS-odo measurement:~%Header header~%~%string child_frame_id~%~%# a. position:~%geometry_msgs/Point position~%~%# b. velocity:~%geometry_msgs/Vector3 velocity~%~%# c. magnetic field:~%geometry_msgs/Vector3 magnetic_field~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PosVelMag>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'child_frame_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'magnetic_field))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PosVelMag>))
  "Converts a ROS message object to a list"
  (cl:list 'PosVelMag
    (cl:cons ':header (header msg))
    (cl:cons ':child_frame_id (child_frame_id msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':magnetic_field (magnetic_field msg))
))
