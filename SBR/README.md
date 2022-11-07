### Self Balancing Robot 

1. TB6612FNG Motor Driver
2. Encoder
3. MPU -6050 (Gyroscope and Accelerometer)
4. HC-06 (Bluetooth module)
5. Ultrasonic sensor



### Motor Control
- Set the motor to be output 
- Start the motor at 0 and set the minimum speed to balance

### Orientation control
- With the help of the MPU-6050 to retrieve information from the gyroscope and accelerometer sensor
- Gyroscope sensor is a device that can measure and maintain the orientation and angular velocity of an object. These are more advanced than accelerometers. These can measure the tilt and lateral orientation of the object whereas accelerometer can only measure the linear motion.

- Calibration of the MPU-6050; Set the SBR balanced, Get the output of the x and y values for both the accelerometer and gyroscope readings fromthe registers using the wire library(12C Communication protocol)
- The angle can be calculated via the change rather than having to calibrate the gyroscope to zero

#### PID CONTROL
- 


## Bluetooth Control
- 

## Ultrasonic Sensor 
- 

### Implementing Balance by using MPU-6050 data against recorded rpm via the encoder
The Inertial Measurement Unit Sensor provides  time serries data
