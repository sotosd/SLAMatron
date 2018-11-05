# SLAMatron
Simultaneous Localization And Mapping  / An Autonomous Robotic Vehicle for Mapping .


A robot using MPU 6050 IMU, BT, 2 Ultrasonic Sensors and 2 uln2003 for controling the 28byj-48 steppers
The robot can be programmed as desplayed at the commented section in loop() in order to execute SLAM.
The robot follows the left sided wall, as long there is one. It steps for 10cm each time that drives forward (can be changed).
If any obstacle is found it can turn left or right (-90 and +90 degrees). 
It scans with the front sided USensor, which is attached on a servo motor, and stores it the distances measured. 
The measured distances correspond to the  angles [0,180] (stepping 10 degrees at a time => 19 values)
The IMU prevents drift and allows for precise direction when the robot turns without encoders. 
The measured distances, together with robot's heading and the robot steps, are being sent to base/station (pc)
where a python script processes them and produces the wanted map (Occupancy Grid Mapping).

Further description, and more information about this project will be added soon...
