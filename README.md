LSM9DS0
=======

LSM9DS0 9DOF sensor AHRS sketch

Like the original LSM9DS0_simple.ino sketch, it'll demo the following:
* How to create a LSM9DS0 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS0 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the calcAccel(), calcGyro() and calcMag()
  functions.
* How to get the temperature from the accelerometer and display on the Serial monitor in degrees Centigrade.

In addition, the sketch will demo:
* How to calculate and correct for the accelerometer and gyro biases
* How to check for data updates using interrupts
* How to display output at a rate different from the sensor data update and fusion filter update rates
* How to specify the accelerometer anti-aliasing (low-pass) filter rate
* How to use the data from the LSM9DS0 to fuse the sensor data into a quaternion representation of the sensor frame
  orientation relative to a fixed Earth frame providing absolute orientation information for subsequent use.
* An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
  Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.

A discussion of the use and limitations of this sensor and sensor fusion in general is found ![here.]( https://github.com/kriswiner/MPU-6050/wiki/Affordable-9-DoF-Sensor-Fusion)

I added a sketch specifically for the [LSM9DS0 Mini and Micro-Add-On shields.](https://www.tindie.com/products/onehorse/lsm9ds0-teensy-31-mini-shield/)  It uses a Teensy 3.1-specific i2c Wire library called i2c_t3.h, which allows easy access to Teensy-specific capabilities such as specification of which set of hardware i2c pins will be used, the bus speed (up to 1 MHz!) and also allows master and/or slave designation to handle multiplexing between i2c devices. See www.pjrc.com/teensy and  http://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3 for details.

A description of the modular Teensy Project which uses the LSM9DS0 motion sensor, among others, is described [here.](https://github.com/kriswiner/LSM9DS0/wiki/Modular-Teensy-Project)

For a discussion of the relative merits of modern board-mounted pressure sensors, see [here](https://github.com/kriswiner/MPU-9250/wiki/Small-pressure-sensors).

