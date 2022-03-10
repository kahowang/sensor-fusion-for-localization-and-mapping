;; Auto-generated. Do not edit!


(when (boundp 'lidar_localization::saveMap)
  (if (not (find-package "LIDAR_LOCALIZATION"))
    (make-package "LIDAR_LOCALIZATION"))
  (shadow 'saveMap (find-package "LIDAR_LOCALIZATION")))
(unless (find-package "LIDAR_LOCALIZATION::SAVEMAP")
  (make-package "LIDAR_LOCALIZATION::SAVEMAP"))
(unless (find-package "LIDAR_LOCALIZATION::SAVEMAPREQUEST")
  (make-package "LIDAR_LOCALIZATION::SAVEMAPREQUEST"))
(unless (find-package "LIDAR_LOCALIZATION::SAVEMAPRESPONSE")
  (make-package "LIDAR_LOCALIZATION::SAVEMAPRESPONSE"))

(in-package "ROS")





(defclass lidar_localization::saveMapRequest
  :super ros::object
  :slots ())

(defmethod lidar_localization::saveMapRequest
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

(defclass lidar_localization::saveMapResponse
  :super ros::object
  :slots (_succeed ))

(defmethod lidar_localization::saveMapResponse
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

(defclass lidar_localization::saveMap
  :super ros::object
  :slots ())

(setf (get lidar_localization::saveMap :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveMap :datatype-) "lidar_localization/saveMap")
(setf (get lidar_localization::saveMap :request) lidar_localization::saveMapRequest)
(setf (get lidar_localization::saveMap :response) lidar_localization::saveMapResponse)

(defmethod lidar_localization::saveMapRequest
  (:response () (instance lidar_localization::saveMapResponse :init)))

(setf (get lidar_localization::saveMapRequest :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveMapRequest :datatype-) "lidar_localization/saveMapRequest")
(setf (get lidar_localization::saveMapRequest :definition-)
      "
---
bool succeed
")

(setf (get lidar_localization::saveMapResponse :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveMapResponse :datatype-) "lidar_localization/saveMapResponse")
(setf (get lidar_localization::saveMapResponse :definition-)
      "
---
bool succeed
")



(provide :lidar_localization/saveMap "8d9c3b918a0afafe09791ef8d7853918")


