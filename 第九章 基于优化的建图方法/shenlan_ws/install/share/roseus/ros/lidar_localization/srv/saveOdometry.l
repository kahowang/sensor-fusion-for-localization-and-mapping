;; Auto-generated. Do not edit!


(when (boundp 'lidar_localization::saveOdometry)
  (if (not (find-package "LIDAR_LOCALIZATION"))
    (make-package "LIDAR_LOCALIZATION"))
  (shadow 'saveOdometry (find-package "LIDAR_LOCALIZATION")))
(unless (find-package "LIDAR_LOCALIZATION::SAVEODOMETRY")
  (make-package "LIDAR_LOCALIZATION::SAVEODOMETRY"))
(unless (find-package "LIDAR_LOCALIZATION::SAVEODOMETRYREQUEST")
  (make-package "LIDAR_LOCALIZATION::SAVEODOMETRYREQUEST"))
(unless (find-package "LIDAR_LOCALIZATION::SAVEODOMETRYRESPONSE")
  (make-package "LIDAR_LOCALIZATION::SAVEODOMETRYRESPONSE"))

(in-package "ROS")





(defclass lidar_localization::saveOdometryRequest
  :super ros::object
  :slots ())

(defmethod lidar_localization::saveOdometryRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass lidar_localization::saveOdometryResponse
  :super ros::object
  :slots (_succeed ))

(defmethod lidar_localization::saveOdometryResponse
  (:init
   (&key
    ((:succeed __succeed) nil)
    )
   (send-super :init)
   (setq _succeed __succeed)
   self)
  (:succeed
   (&optional __succeed)
   (if __succeed (setq _succeed __succeed)) _succeed)
  (:serialization-length
   ()
   (+
    ;; bool _succeed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _succeed
       (if _succeed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _succeed
     (setq _succeed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass lidar_localization::saveOdometry
  :super ros::object
  :slots ())

(setf (get lidar_localization::saveOdometry :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveOdometry :datatype-) "lidar_localization/saveOdometry")
(setf (get lidar_localization::saveOdometry :request) lidar_localization::saveOdometryRequest)
(setf (get lidar_localization::saveOdometry :response) lidar_localization::saveOdometryResponse)

(defmethod lidar_localization::saveOdometryRequest
  (:response () (instance lidar_localization::saveOdometryResponse :init)))

(setf (get lidar_localization::saveOdometryRequest :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveOdometryRequest :datatype-) "lidar_localization/saveOdometryRequest")
(setf (get lidar_localization::saveOdometryRequest :definition-)
      "
---
bool succeed
")

(setf (get lidar_localization::saveOdometryResponse :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveOdometryResponse :datatype-) "lidar_localization/saveOdometryResponse")
(setf (get lidar_localization::saveOdometryResponse :definition-)
      "
---
bool succeed
")



(provide :lidar_localization/saveOdometry "8d9c3b918a0afafe09791ef8d7853918")


