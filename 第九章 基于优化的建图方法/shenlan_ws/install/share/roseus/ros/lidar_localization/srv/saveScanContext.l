;; Auto-generated. Do not edit!


(when (boundp 'lidar_localization::saveScanContext)
  (if (not (find-package "LIDAR_LOCALIZATION"))
    (make-package "LIDAR_LOCALIZATION"))
  (shadow 'saveScanContext (find-package "LIDAR_LOCALIZATION")))
(unless (find-package "LIDAR_LOCALIZATION::SAVESCANCONTEXT")
  (make-package "LIDAR_LOCALIZATION::SAVESCANCONTEXT"))
(unless (find-package "LIDAR_LOCALIZATION::SAVESCANCONTEXTREQUEST")
  (make-package "LIDAR_LOCALIZATION::SAVESCANCONTEXTREQUEST"))
(unless (find-package "LIDAR_LOCALIZATION::SAVESCANCONTEXTRESPONSE")
  (make-package "LIDAR_LOCALIZATION::SAVESCANCONTEXTRESPONSE"))

(in-package "ROS")





(defclass lidar_localization::saveScanContextRequest
  :super ros::object
  :slots ())

(defmethod lidar_localization::saveScanContextRequest
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

(defclass lidar_localization::saveScanContextResponse
  :super ros::object
  :slots (_succeed ))

(defmethod lidar_localization::saveScanContextResponse
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

(defclass lidar_localization::saveScanContext
  :super ros::object
  :slots ())

(setf (get lidar_localization::saveScanContext :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveScanContext :datatype-) "lidar_localization/saveScanContext")
(setf (get lidar_localization::saveScanContext :request) lidar_localization::saveScanContextRequest)
(setf (get lidar_localization::saveScanContext :response) lidar_localization::saveScanContextResponse)

(defmethod lidar_localization::saveScanContextRequest
  (:response () (instance lidar_localization::saveScanContextResponse :init)))

(setf (get lidar_localization::saveScanContextRequest :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveScanContextRequest :datatype-) "lidar_localization/saveScanContextRequest")
(setf (get lidar_localization::saveScanContextRequest :definition-)
      "
---
bool succeed
")

(setf (get lidar_localization::saveScanContextResponse :md5sum-) "8d9c3b918a0afafe09791ef8d7853918")
(setf (get lidar_localization::saveScanContextResponse :datatype-) "lidar_localization/saveScanContextResponse")
(setf (get lidar_localization::saveScanContextResponse :definition-)
      "
---
bool succeed
")



(provide :lidar_localization/saveScanContext "8d9c3b918a0afafe09791ef8d7853918")


