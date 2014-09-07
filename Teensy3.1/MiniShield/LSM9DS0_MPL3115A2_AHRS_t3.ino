/*****************************************************************
LSM9DS0_AHRS.ino
SFE_LSM9DS0 Library AHRS Data Fusion Example Code
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 18, 2014
https://github.com/sparkfun/LSM9DS0_Breakout

Modified by Kris Winer, April 4, 2014

This sketch is for the Teesny 3.1 Mini add-on shield with the LSM9DS0 9-axis motion sensor coupled with the MPL3115A2
pressure sensor/altimeter. It uses the Teensy-3.1-only i2c_t3.h Wire library to allow access to auxiliary hardware 
i2c pins on the Teensy 3.1.

The LSM9DS0 is a versatile 9DOF sensor. It has a built-in accelerometer, gyroscope, and magnetometer. Very cool! Plus it
functions over either SPI or I2C. In this case, the Mini add-on shield has been hardwired for I2C so its SPI functions cannot be used.

The MPL3115A2 is a popular pressure sensor/altimeter capable of providing 20 cm (about a foot) of relative altitude accuracy.
This is not quite as good as the MS5637 which I will use in subsequent Mini and Micro Teensy add-on shileds and it has
a rather large footprint. Still, it is a very good choice for the MIni-sized board and works well here.

This Arduino sketch utilizes Jim Lindblom's SFE_LSM9DS0 library to generate the basic sensor data
for use in two sensor fusion algorithms becoming increasingly popular with DIY quadcopter and robotics engineers. 

Like the original LSM9SD0_simple.ino sketch, it'll demo the following:
* How to create a LSM9DS0 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS0 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGyro(), readAccel(), readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the calcAccel(), calcGyro() and calcMag()
  functions.
  
In addition, the sketch will demo:
* How to check for data updates using interrupts
* How to display output at a rate different from the sensor data update and fusion filter update rates
* How to specify the accelerometer anti-aliasing (low-pass) filter rate
* How to use the data from the LSM9DS0 to fuse the sensor data into a quaternion representation of the sensor frame
  orientation relative to a fixed Earth frame providing absolute orientation information for subsequent use.
* An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
  Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.

Hardware setup: This library supports communicating with the
LSM9DS0 and MPL3115A2 over both I2C and SPI but for this Mini shield we have hard-wired the board for I2C only. 

Note: The LSM9DS0 in the I2C mode uses the Teensy 3.1 i2c_t3.h Wire library. 
We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
  
	
This code is beerware. If you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please 
buy us a round!

Distributed as-is; no warranty is given.
*****************************************************************/

// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <i2c_t3.h>
//#include <Wire.h>
#include <SFE_LSM9DS0.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
// Standard 7-bit I2C slave address is 1100000 = 0x60, 8-bit read address is 0xC1, 8-bit write is 0xC0
#define MPL3115A2_ADDRESS 0x60  // SA0 is high, 0x1C if low

// Register defines courtesy A. Weiss and Nathan Seidle, SparkFun Electronics
#define MPL3115A2_STATUS     0x00
#define MPL3115A2_OUT_P_MSB  0x01
#define MPL3115A2_OUT_P_CSB  0x02
#define MPL3115A2_OUT_P_LSB  0x03
#define MPL3115A2_OUT_T_MSB  0x04
#define MPL3115A2_OUT_T_LSB  0x05
#define MPL3115A2_DR_STATUS  0x06
#define MPL3115A2_OUT_P_DELTA_MSB  0x07
#define MPL3115A2_OUT_P_DELTA_CSB  0x08
#define MPL3115A2_OUT_P_DELTA_LSB  0x09
#define MPL3115A2_OUT_T_DELTA_MSB  0x0A
#define MPL3115A2_OUT_T_DELTA_LSB  0x0B
#define MPL3115A2_WHO_AM_I   0x0C
#define MPL3115A2_F_STATUS   0x0D
#define MPL3115A2_F_DATA     0x0E
#define MPL3115A2_F_SETUP    0x0F
#define MPL3115A2_TIME_DLY   0x10
#define MPL3115A2_SYSMOD     0x11
#define MPL3115A2_INT_SOURCE 0x12
#define MPL3115A2_PT_DATA_CFG 0x13
#define MPL3115A2_BAR_IN_MSB 0x14 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_BAR_IN_LSB 0x15 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_P_TGT_MSB  0x16
#define MPL3115A2_P_TGT_LSB  0x17
#define MPL3115A2_T_TGT      0x18
#define MPL3115A2_P_WND_MSB  0x19
#define MPL3115A2_P_WND_LSB  0x1A
#define MPL3115A2_T_WND      0x1B
#define MPL3115A2_P_MIN_MSB  0x1C
#define MPL3115A2_P_MIN_CSB  0x1D
#define MPL3115A2_P_MIN_LSB  0x1E
#define MPL3115A2_T_MIN_MSB  0x1F
#define MPL3115A2_T_MIN_LSB  0x20
#define MPL3115A2_P_MAX_MSB  0x21
#define MPL3115A2_P_MAX_CSB  0x22
#define MPL3115A2_P_MAX_LSB  0x23
#define MPL3115A2_T_MAX_MSB  0x24
#define MPL3115A2_T_MAX_LSB  0x25
#define MPL3115A2_CTRL_REG1  0x26
#define MPL3115A2_CTRL_REG2  0x27
#define MPL3115A2_CTRL_REG3  0x28
#define MPL3115A2_CTRL_REG4  0x29
#define MPL3115A2_CTRL_REG5  0x2A
#define MPL3115A2_OFF_P      0x2B
#define MPL3115A2_OFF_T      0x2C
#define MPL3115A2_OFF_H      0x2D


enum SAMPLERATE {
  OS_6ms = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  OS_10ms,
  OS_18ms,
  OS_34ms,
  OS_66ms,
  OS_130ms, // 130 ms oversampling interval, 2^5 = 32 oversample ratio
  OS_258ms,
  OS_512ms
};

enum ST_VALUE {
  ATS_1s = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  ATS_2s,
  ATS_4s,
  ATS_8s,
  ATS_16s,
  ATS_32s,
  ATS_64s, // 2^6 = 64 s interval between up to 32 FIFO samples for half an hour of data logging
  ATS_128s,
  ATS_256s,
  ATS_512s,
  ATS_1024s,
  ATS_2048s,
  ATS_4096s,
  ATS_8192s,
  ATS_16384s,
  ATS_32768s  // 2^15 = 32768 s interval between up to 32 FIFO samples = 12 days of data logging!
};

uint8_t SAMPLERATE = OS_130ms;
uint8_t ST_VALUE = ATS_4s;
int AltimeterMode = 0; // use to choose between altimeter and barometer modes for FIFO data

// Define device outputs
float altitude = 0.;
float pressure = 0.;

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define SDO 1
#if SDO
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
#else 
#define LSM9DS0_XM  0x1E // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6A // Would be 0x6A if SDO_G is LOW
#endif

// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
const byte INT1XM = 13;  // INT1XM tells us when accel data is ready
const byte INT2XM = 14;  // INT2XM tells us when mag data is ready
const byte DRDYG  = 12;  // DRDYG  tells us when gyro data is ready
const byte SDOpin = 15;  // Selects either of two I2C addresses
const byte INT1P  =  9;  // INT1P tells us when pressure data is ready
const byte INT2P  =  8;  // INT2P tells us when the altimeter is asleep
const byte myLed  = 13;  // Microcontroller led

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (1.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t count = 0;  // used to control display output rate
uint32_t delt_t = 0; // used to control display output rate
float pitch, yaw, roll, heading;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, sumCount = 0;  // used to calculate integration interval
uint32_t Now = 0;                       // used to calculate integration interval

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature, Temperature;
 
void setup()
{
  delay(5000);
  Serial.begin(38400); // Start serial at 38400 bps
 
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);
  pinMode(INT1P,  INPUT);
  pinMode(INT2P,  INPUT);

  pinMode(SDOpin, OUTPUT);
  digitalWrite(SDOpin, SDO);
  pinMode(myLed,  OUTPUT);
  digitalWrite(myLed, HIGH);
  
  Serial.println("This is the LSM9DS0 and MPL3115A2...");

  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  display.setRotation(0); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);  display.print("LSM9DS0");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9 DOF sensor");
  display.setCursor(5, 30); display.print("data fusion");
  display.setCursor(20, 40); display.print("AHRS");
  display.display();
  delay(1000);
  
  // Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("M3115A2");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("20-bit");
  display.setCursor(0, 30); display.print("pressure");
  display.setCursor(0, 40); display.print("sensor");
  display.setCursor(20,50); display.print("altimeter");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
  display.display();
            
  // begin() returns a 16-bit value which includes both the gyro 
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  uint32_t status = dof.begin();
 
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  display.setCursor(0,0); display.print("I AM");
  display.setCursor(0,10); display.print(status, HEX);
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x49D4, HEX); 
  display.display();
  delay(1000); 
  
  // Read the WHO_AM_I register of the MPL3115A2, this is a good test of communication
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_WHO_AM_I);  // Read WHO_AM_I register
  Serial.print("MPL3115A2 WHO_AM_I returned: 0x");
  Serial.println(c, HEX);
  Serial.println("Should be 0xC4");
  Serial.println();
  display.setCursor(0, 0); display.print("MPL3115A2");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xC4, HEX); 
  display.display();
  delay(1000); 
  
    if (c == 0xC4) // WHO_AM_I should always be 0xC4
  {  
    
    MPL3115A2Reset();                // Start off by resetting all registers to the default
    initRealTimeMPL3115A2();         // initialize the accelerometer for realtime data acquisition if communication is OK
    MPL3115A2SampleRate(SAMPLERATE); // Set oversampling ratio
    Serial.print("Oversampling Ratio is "); Serial.println(1<<SAMPLERATE);  
    MPL3115A2enableEventflags();     // Set data ready enable
    Serial.println("MPL3115A2 event flags enabled...");

    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("OSR "); display.print(1<<SAMPLERATE);  
    display.setCursor(0,20); display.print("Event flags");
    display.setCursor(0,30); display.print("enabled...");
    display.display();
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to MPL3115A2: 0x");
    Serial.println(c, HEX);
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("Error! on 0x"); display.print(c, HEX);
    display.display();
    
    while(1) ; // Loop forever if communication doesn't happen
  }
  
 // Set data output ranges; choose lowest ranges for maximum resolution
 // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
    dof.setAccelScale(dof.A_SCALE_2G);
 // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
    dof.setGyroScale(dof.G_SCALE_245DPS);
 // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
    dof.setMagScale(dof.M_SCALE_2GS);
  
 // Set output data rates  
 // Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50, 
 //                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
    dof.setAccelODR(dof.A_ODR_200); // Set accelerometer update rate at 100 Hz
 // Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
 // Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
 // Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
 // to get a smooth ~150 Hz filter update rate
    dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise
 
 // Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
 //                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
    dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

 // Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
    dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms
    
 // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
 // all subsequent measurements.
    dof.calLSM9DS0(gbias, abias);
}

void loop()
{
  if(digitalRead(DRDYG)) {  // When new gyro data is ready
  dof.readGyro();           // Read raw gyro data
    gx = dof.calcGyro(dof.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
    gy = dof.calcGyro(dof.gy) - gbias[1];
    gz = dof.calcGyro(dof.gz) - gbias[2];
  }
  
  if(digitalRead(INT1XM)) {  // When new accelerometer data is ready
    dof.readAccel();         // Read raw accelerometer data
    ax = dof.calcAccel(dof.ax) - abias[0];   // Convert to g's, remove accelerometer biases
    ay = dof.calcAccel(dof.ay) - abias[1];
    az = dof.calcAccel(dof.az) - abias[2];
  }
  
  if(digitalRead(INT2XM)) {  // When new magnetometer data is ready
    dof.readMag();           // Read raw magnetometer data
    mx = dof.calcMag(dof.mx);     // Convert to Gauss and correct for calibration
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
  }

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now; 
    
    sum += deltat;
    sumCount++;
    
  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
//   MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
     MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
    
    MPL3115A2ActiveAltimeterMode(); 
    MPL3115A2readAltitude();  // Read the altitude

    MPL3115A2ActiveBarometerMode(); 
    MPL3115A2readPressure();  // Read the pressure
    
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = pressure/100; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;
    
    Serial.print("pressure is "); Serial.print(pressure, 2); Serial.println(" Pa");  // Print altitude in meters
    Serial.print("altitude is "); Serial.print(altitude, 2);  Serial.println(" m"); // Print altitude in meters   
    Serial.print("  temperature is "); Serial.print(temperature, 2);  Serial.println(" C"); // Print temperature in C
    
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(10, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("P   "); display.setCursor(30,10); display.print(pressure/1000., 2);    display.print(" kPa");
    display.setCursor(0,20); display.print("Alt "); display.setCursor(24,20); display.print(altitude, 1);    display.print("    m");
    display.setCursor(0,30); display.print("Alt "); display.setCursor(24,30); display.print(3.28084*altitude, 1);  display.print("   ft");
    display.setCursor(0,40); display.print("T   "); display.setCursor(30,40); display.print(temperature, 1); display.print("    C");
    display.display();
 
  // Print the heading and orientation for fun!
    printHeading(mx, my);
    printOrientation(ax, ay, az);

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination), 
  // looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;

    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)1000*mx); 
    Serial.print(" my = "); Serial.print( (int)1000*my); 
    Serial.print(" mz = "); Serial.print( (int)1000*mz); Serial.println(" mG");
    
    dof.readTemp();
    Temperature = 21.0 + (float) dof.temperature/8.; // slope is 8 LSB per degree C, just guessing at the intercept
    Serial.print("LSM9DS0 temperature = "); Serial.println(Temperature, 2);
    
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    
    Serial.print("filter rate = "); Serial.println((float) sumCount/sum, 1);

    display.clearDisplay();     
 
    display.setCursor(0, 0); display.print(" x   y   z  ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(1000*mx)); 
    display.setCursor(24, 24); display.print((int)(1000*my)); 
    display.setCursor(48, 24); display.print((int)(1000*mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
  
    // With ODR settings of 400 Hz, 380 Hz, and 25 Hz for the accelerometer, gyro, and magnetometer, respectively,
    // the filter is updating at a ~125 Hz rate using the Madgwick scheme and ~165 Hz using the Mahony scheme 
    // even though the display refreshes at only 2 Hz.
    // The filter update rate can be increased by reducing the rate of data reading. The optimal implementation is
    // one which balances the competing rates so they are about the same; that is, the filter updates the sensor orientation
    // at about the same rate the data is changing. Of course, other implementations are possible. One might consider
    // updating the filter at twice the average new data rate to allow for finite filter convergence times.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the sensor ODRs, especially the magnetometer ODR:
    // smaller ODRs for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of ~110 and ~135 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // With low ODR settings of 100 Hz, 95 Hz, and 6.25 Hz for the accelerometer, gyro, and magnetometer, respectively,
    // the filter is updating at a ~150 Hz rate using the Madgwick scheme and ~200 Hz using the Mahony scheme.
    // These filter update rates should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    display.setCursor(0, 40); display.print("rt: "); display.print((float) sumCount/sum, 2); display.print(" Hz"); 
    display.display();

    digitalWrite(myLed, !digitalRead(myLed)); // toggle led
    count = millis();
    sum = 0;
    sumCount = 0;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Some useful functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPL3115A2readAltitude() // Get altitude in meters and temperature in centigrade
{
  uint8_t rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers 

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
// while ((readByte(MPL3115A2_ADDRESS, STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
  
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Altutude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
  uint8_t msbA = rawData[0];
  uint8_t csbA = rawData[1];
  uint8_t lsbA = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4];
 
 // Calculate altitude, check for negative sign in altimeter data
 long foo = 0;
 if(msbA > 0x7F) {
   foo = ~((long)msbA << 16 | (long)csbA << 8 | (long)lsbA) + 1; // 2's complement the data
   altitude = (float) (foo >> 8) + (float) ((lsbA >> 4)/16.0); // Whole number plus fraction altitude in meters for negative altitude
   altitude *= -1.;
 }
 else {
   altitude = (float) ( (msbA << 8) | csbA) + (float) ((lsbA >> 4)/16.0);  // Whole number plus fraction altitude in meters
 }

// Calculate temperature, check for negative sign
if(msbT > 0x7F) {
 foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

void MPL3115A2readPressure()
{
  uint8_t  rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 //while ((readByte(MPL3115A2_ADDRESS, STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
 
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Pressure bytes
  uint8_t msbP = rawData[0];
  uint8_t csbP = rawData[1];
  uint8_t lsbP = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4]; 
 
  long pressure_whole =   ((long)msbP << 16 |  (long)csbP << 8 |  (long)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

// Calculate temperature, check for negative sign
long foo = 0;
if(msbT > 0x7F) { // Is the most significant bit a 1? Then its a negative number in two's complement form
 foo = ~((long) msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) ((foo >> 8) + ((lsbT >> 4)/16.0)); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

/*
=====================================================================================================
Define functions according to 
"Data Manipulation and Basic Settings of the MPL3115A2 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void MPL3115A2toggleOneShot()
{
    MPL3115A2Active();  // Set to active to start reading
    uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
    c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void MPL3115A2SampleRate(uint8_t samplerate)
{
  MPL3115A2Standby();  // Must be in standby to change registers

  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
  if(samplerate < 8) { // OSR between 1 and 7
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
  }
  
  MPL3115A2Active();  // Set to active to start reading
 }
 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 registers for FIFO mode
void initFIFOMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x40);  // enable FIFO
  
  //  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // Clear FIFO mode
// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
//  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 for realtime data collection 
void initRealTimeMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x80);  
  
  //  Configure INT 1 for data ready, all other interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // disable FIFO mode
  
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step
void MPL3115A2TimeStep(uint8_t ST_Value)
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2); // Read contents of register CTRL_REG2
 if (ST_Value <= 0xF) {
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
 }
 
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
 // Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
 // Bit 1 is event flag on new Pressure/Altitude data
 // Bit 0 is event flag on new Temperature data
void MPL3115A2enableEventflags()
{
  MPL3115A2Standby();  // Must be in standby to change registers
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void MPL3115A2ActiveAltimeterMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void MPL3115A2ActiveBarometerMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115A2.
// It must be in standby to change most register settings
void MPL3115A2Reset()
{
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, 0x04); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to standby mode.
// It must be in standby to change most register settings
void MPL3115A2Standby()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to active mode.
// Needs to be in this mode to output data
void MPL3115A2Active()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}


// Here's a fun function to calculate your heading, using Earth's
// magnetic field.
// It only works if the sensor is flat (z-axis normal to Earth).
// Additionally, you may need to add or subtract a declination
// angle to get the heading normalized to your location.
// See: http://www.ngdc.noaa.gov/geomag/declination.shtml
void printHeading(float hx, float hy)
{
  if (hy > 0)
  {
    heading = 90 - (atan(hx / hy) * (180 / PI));
  }
  else if (hy < 0)
  {
    heading = - (atan(hx / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hx < 0) heading = 180;
    else heading = 0;
  }
  
  Serial.print("Heading: ");
  Serial.println(heading, 2);
}

// Another fun function that does calculations based on the
// acclerometer data. This function will print your LSM9DS0's
// orientation -- it's roll and pitch angles.
void printOrientation(float x, float y, float z)
{
 // float pitch, roll;
  
  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
            void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }

// I2C write/read functions for the MPL3115A2 altimeter
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (size_t) 1);  // Read one uint8_t from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, (size_t)count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
