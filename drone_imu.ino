// Drone FBWA Demonstration
// Custom UAV frame + MPU6050 + Teensy 3.2
// 4 x DYS 1806 motors with 20A BL_Heli ESCs

// Complementary Filter + PID Control
// Adapted from tutorials/online resources
// https://playground.arduino.cc/Main/MPU-6050/

#include <Wire.h> //for i2c
#include <math.h> //math library

const int MPUAddress = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp; //creating variables for raw data values

// MPU6050 DEFINES
#define POWER 0x6B
#define ACCEL_XOUT_HIGH 0x3B
#define GYRO_FACTOR 131  //leads to angular velocity in degrees/sec
#define RAD2DEGREE 180/3.14159; //radians to degreees
#define ACCEL_FACTOR 16384
#define COMP_X_OFFSET 54.3
#define COMP_Y_OFFSET -44.4

// ACTUATION PARAMS
#define MOTOR_1 23
#define MOTOR_2 22
#define MOTOR_3 21
#define MOTOR_4 20
#define PWM_PERIOD 20000 // 20ms in microseconds = 20x10^3 microseconds
#define PWM_RESOLUTION 4096
#define PWM_FREQUENCY 50 // 50Hz 1 - 2 ms for PWM at UAV motors
int currPWM = 0;

int pins_motors[] = {MOTOR_1,MOTOR_2,MOTOR_3,MOTOR_4};

// Setting up PWM for Drone Actuators:
void setupMotors(int pins_motors[],int num_pins)
{
  for (int k = 0; k < num_pins; k++)
  {
    digitalWrite(pins_motors[k],OUTPUT);
    analogWrite(pins_motors[k],PWM_FREQUENCY);
    delay(5);
  }
}

// freq = 1 / period
// (PWM between 1000-2000 / 20ms) * 4096
// PWM requires is 1 - 2 ms (1000 - 2000 micro sec)
void send_motor_pwm()
{
  float sentPWM = (currPWM / PWM_PERIOD) * PWM_RESOLUTION; 
  for (int _pin_cnt = 0; _pin_cnt < _num_pins; _pin_cnt++) analogWrite(pins_motors[_pin_cnt], round(sentPWM));
}


// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally,
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};
 

char imuData[256];

//Used for complementary filter calculations
//needed to update gyro readings
unsigned long last_read_time;
float         last_x_angle;  
float         last_y_angle;
float         last_z_angle;  

//Function to update all read gyro angles and record the next time step for filter
void set_last_read_angle_data(unsigned long time, float x, float y, float z) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;
 
  Wire.beginTransmission(MPUAddress);
  n = Wire.write(start);
  if (n != 1)
    return (-10);
 
  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);
 
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPUAddress, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
 
  return (0);  // return : no error
}
 
 
// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;
 
  Wire.beginTransmission(MPUAddress);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);
 
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);
 
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
 
  return (0);         // return : no error
}
 
// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;
 
  error = MPU6050_write(reg, &data, 1);
 
  return (error);
}

// State Machine
#define STATE_IDLE (0x01)
#define STATE_ARM  (0x02)
#define STATE_STABILIZE (0x03)
int state = STATE_IDLE;

void setup(){

  // Setting up Serial
  Serial.begin(115200);

  // Setting up Motors/PWM
  setupMotors(pins_motors,4);

  // Setting up IMU
  Wire.begin();  
  Wire.beginTransmission(MPUAddress); //starts talking to IMU at address 0x68
  Wire.write(POWER);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
}

void loop(){

  accel_t_gyro_union accel_t_gyro;
  unsigned long t_now = millis();

  switch (state)
  {
    case (STATE_IDLE):
      break;

    case (STATE_ARM):
      break;

    case (STATE_STABILIZE):
      break;

    default:
      break;
  }  
  
  MPU6050_read (ACCEL_XOUT_HIGH, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
 
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  AcX = accel_t_gyro.value.x_accel;
  AcY = accel_t_gyro.value.x_accel;
  AcZ = accel_t_gyro.value.y_accel;
  GyX = accel_t_gyro.value.x_gyro/GYRO_FACTOR;
  GyY = accel_t_gyro.value.y_gyro/GYRO_FACTOR; 
  GyZ = accel_t_gyro.value.z_gyro/GYRO_FACTOR;

  //Accelerometer Y (-rho). turning around y axis results in vector on x axis
  float comp_accel_y = atan(-1.0*AcX/sqrt(pow(AcY,2) + pow(AcZ,2)))*RAD2DEGREE;
  
  //Accelerometer X (phi). turning around x axis results in vector on y axis
  float comp_accel_x = atan(AcY/sqrt(pow(AcX,2) + pow(AcZ,2)))*RAD2DEGREE;
  float theta = 0; //z-axis angle

  //Complementary Filter - removes noise and drift from IMU and Gyro
  //Filtered Angle = alpha * (angle + GyroData*dt) + (1-alpha)*Accelerometer Data
  
  double dt = (t_now - last_read_time)/1000.0; //timestep
  //Current angular position = 
  //Angular velocity (degree/s) * time between readings + previous angular postion
  float gyro_x = GyX * dt + last_x_angle;
  float gyro_y = GyY * dt + last_y_angle;
  float gyro_z = GyZ * dt + last_z_angle;

  float alpha = 0.96; //weighting, which measurement to use more, accel or gyro
  float cf_x = (alpha * gyro_x) + (1.0 - alpha)*comp_accel_x;
  float cf_y = (alpha * gyro_y) + (1.0 - alpha)*comp_accel_y;
  float cf_z = gyro_z;

  //update previous angles with current angles in loop
  set_last_read_angle_data(t_now, cf_x, cf_y, cf_z);

  // Motor Output  
  send_motor_pwm(currPWM);
  
  //Printing the filtered roll and pitch angles for debugging
  sprintf(imuData,"%0.3f,%0.3f\n",cf_x + COMP_X_OFFSET,cf_y + COMP_Y_OFFSET);  
  Serial.print(imuData);  
  
  delay(10);
 }
  

