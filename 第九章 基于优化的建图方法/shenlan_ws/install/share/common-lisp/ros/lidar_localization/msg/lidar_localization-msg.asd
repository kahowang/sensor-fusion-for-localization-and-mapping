
(cl:in-package :asdf)

(defsystem "lidar_localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EKFStd" :depends-on ("_package_EKFStd"))
    (:file "_package_EKFStd" :depends-on ("_package"))
    (:file "ESKFStd" :depends-on ("_package_ESKFStd"))
    (:file "_package_ESKFStd" :depends-on ("_package"))
    (:file "IMUGNSSMeasurement" :depends-on ("_package_IMUGNSSMeasurement"))
    (:file "_package_IMUGNSSMeasurement" :depends-on ("_package"))
    (:file "LidarMeasurement" :depends-on ("_package_LidarMeasurement"))
    (:file "_package_LidarMeasurement" :depends-on ("_package"))
    (:file "PosVel" :depends-on ("_package_PosVel"))
    (:file "_package_PosVel" :depends-on ("_package"))
    (:file "PosVelMag" :depends-on ("_package_PosVelMag"))
    (:file "_package_PosVelMag" :depends-on ("_package"))
  ))