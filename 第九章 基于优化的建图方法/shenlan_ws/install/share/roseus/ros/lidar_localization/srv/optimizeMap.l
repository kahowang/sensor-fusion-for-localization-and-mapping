;; Auto-generated. Do not edit!


(when (boundp 'lidar_localization::optimizeMap)
  (if (not (find-package "LIDAR_LOCALIZATION"))
    (make-package "LIDAR_LOCALIZATION"))
  (shadow 'optimizeMap (find-package "LIDAR_LOCALIZATION")))
(unless (find-package "LIDAR_LOCALIZATION::OPTIMIZEMAP")
  (make-package "LIDAR_LOCALIZATION::OPTIMIZEMAP"))
(unless (find-package "LIDAR_LOCALIZATION::OPTIMIZEMAPREQUEST")
  (make-package "LIDAR_LOCALIZATION::OPTIMIZEMAPREQUEST"))
(unless (find-package "LIDAR_LOCALIZATION::OPTIMIZEMAPRESPONSE")
  (make-package "LIDAR_LOCALIZATION::OPTIMIZEMAPRESPONSE"))

(in-package "ROS")





(defclass lidar_localization::optimizeMapRequest
  :super ros::object
  :slots ())

(defmethod lidar_localization::optimizeMapRequest
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

(defclass lidar_localization::optimizeMapResponse
  :super ros::object
  :slots (_succeed ))

(defmethod lidar_localization::optimizeMapResponse
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

(defclass lidar_localization::optimizeMap
  :super ros::object
  :slots ())

(setf (get lidar_localization::optimizeMap :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::optimizeMap :datatype-) "lidar_localization/optimizeMap")
(setf (get lidar_localization::optimizeMap :request) lidar_localization::optimizeMapRequest)
(setf (get lidar_localization::optimizeMap :response) lidar_localization::optimizeMapResponse)

(defmethod lidar_localization::optimizeMapRequest
  (:response () (instance lidar_localization::optimizeMapResponse :init)))

(setf (get lidar_localization::optimizeMapRequest :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::optimizeMapRequest :datatype-) "lidar_localization/optimizeMapRequest")
(setf (get lidar_localization::optimizeMapRequest :definition-)
      "
---
bool succeed
")

(setf (get lidar_localization::optimizeMapResponse :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::optimizeMapResponse :datatype-) "lidar_localization/optimizeMapResponse")
(setf (get lidar_localization::optimizeMapResponse :definition-)
      "
---
bool succeed
")



(provide :lidar_localization/optimizeMap "8d9c3b918a0afafe09791ef8d7853918")


