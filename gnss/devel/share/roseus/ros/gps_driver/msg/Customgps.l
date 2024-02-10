;; Auto-generated. Do not edit!


(when (boundp 'gps_driver::Customgps)
  (if (not (find-package "GPS_DRIVER"))
    (make-package "GPS_DRIVER"))
  (shadow 'Customgps (find-package "GPS_DRIVER")))
(unless (find-package "GPS_DRIVER::CUSTOMGPS")
  (make-package "GPS_DRIVER::CUSTOMGPS"))

(in-package "ROS")
;;//! \htmlinclude Customgps.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gps_driver::Customgps
  :super ros::object
  :slots (_header _latitude _longitude _altitude _utm_easting _utm_northing _zone _letter _hdop _gpgga_read ))

(defmethod gps_driver::Customgps
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:utm_easting __utm_easting) 0.0)
    ((:utm_northing __utm_northing) 0.0)
    ((:zone __zone) 0)
    ((:letter __letter) "")
    ((:hdop __hdop) 0.0)
    ((:gpgga_read __gpgga_read) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _utm_easting (float __utm_easting))
   (setq _utm_northing (float __utm_northing))
   (setq _zone (round __zone))
   (setq _letter (string __letter))
   (setq _hdop (float __hdop))
   (setq _gpgga_read (string __gpgga_read))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:utm_easting
   (&optional __utm_easting)
   (if __utm_easting (setq _utm_easting __utm_easting)) _utm_easting)
  (:utm_northing
   (&optional __utm_northing)
   (if __utm_northing (setq _utm_northing __utm_northing)) _utm_northing)
  (:zone
   (&optional __zone)
   (if __zone (setq _zone __zone)) _zone)
  (:letter
   (&optional __letter)
   (if __letter (setq _letter __letter)) _letter)
  (:hdop
   (&optional __hdop)
   (if __hdop (setq _hdop __hdop)) _hdop)
  (:gpgga_read
   (&optional __gpgga_read)
   (if __gpgga_read (setq _gpgga_read __gpgga_read)) _gpgga_read)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; float64 _utm_easting
    8
    ;; float64 _utm_northing
    8
    ;; uint8 _zone
    1
    ;; string _letter
    4 (length _letter)
    ;; float64 _hdop
    8
    ;; string _gpgga_read
    4 (length _gpgga_read)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utm_easting
       (sys::poke _utm_easting (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utm_northing
       (sys::poke _utm_northing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _zone
       (write-byte _zone s)
     ;; string _letter
       (write-long (length _letter) s) (princ _letter s)
     ;; float64 _hdop
       (sys::poke _hdop (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _gpgga_read
       (write-long (length _gpgga_read) s) (princ _gpgga_read s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utm_easting
     (setq _utm_easting (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utm_northing
     (setq _utm_northing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _zone
     (setq _zone (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _letter
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _letter (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _hdop
     (setq _hdop (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _gpgga_read
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gpgga_read (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get gps_driver::Customgps :md5sum-) "c13aa5d5b109c777f94aa4fa3948d681")
(setf (get gps_driver::Customgps :datatype-) "gps_driver/Customgps")
(setf (get gps_driver::Customgps :definition-)
      "Header header
float64 latitude
float64 longitude
float64 altitude
float64 utm_easting
float64 utm_northing
uint8 zone
string letter
float64 hdop
string gpgga_read 
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



(provide :gps_driver/Customgps "c13aa5d5b109c777f94aa4fa3948d681")


