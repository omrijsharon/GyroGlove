// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Joystick.h>
#include <EEPROM.h>
#include "PPMEncoder.h"


// Specific I2C addresses may be passed as a parameter here
MPU6050 mpu;        			// Default: AD0 low = 0x68

// MPU Control/Status
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
bool dmpReady = false;         	// Set true if DMP init was successful
uint8_t devStatus;              // Return status after device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer

// MPU Calibration variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int xAccelOffset, yAccelOffset, zAccelOffset, xGyroOffset, yGyroOffset, zGyroOffset;


// Orientation/Motion
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Quaternion q;                   // [w, x, y, z]       Quaternion Container
VectorInt16 aa;                 // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;             // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;            // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;           	// [x, y, z]            Gravity Vector
int16_t gyro[3];               	// [x, y, z]            Gyro Vector
float ypr[3];                   // [yaw, pitch, roll]   Yaw/Pitch/Roll & gravity vector
float euler[3];
float roll0, pitch0, yaw0;
float roll, pitch, yaw;

// Rotation matrix and vectors
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
float rotation_matrix[3][3];
float x_projection_vector[3];
float y_projection_vector[3];
float z_projection_vector[3];
float x_vector[3] = {1.0, 0.0, 0.0};
float y_vector[3] = {0.0, 1.0, 0.0};
float z_vector[3] = {0.0, 0.0, 1.0};
float yx_projection;
float yz_projection;
float xz_projection;

// Glitch Fixer
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int counter = 0;
int size = 4;
float roll_array[4];
float pitch_array[4];
float yaw_array[4];

// World Position
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
float position[3] = {0.0, 0.0, 0.0};
float velocity[3] = {0.0, 0.0, 0.0};
float acceleration[3] = {0.0, 0.0, 0.0};
float gravity_const = 9.81;
unsigned long t0;
unsigned long t1;
float dt;


// Joystick
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Joystick_ Joystick;
int RxAxis = 0;
int RyAxis = 0;
int XAxis = 0;
int YAxis = 0;

float yawStick;
float pitchStick;
float rollStick;

// Bend Sensor
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define BEND_Vin 16
int bendPin = A0;
float raw_throttle;
int throttle;
float throttle_avg;
int avg_size;
int min_throttle;
int mid_throttle;
int max_throttle;

// PPM
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define PPM_PIN 5


// LED
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define LED_PIN 13
bool blinkState = false;

// ================================================================
// === calculating the std of a sample in an array              ===
// ================================================================
float exclude_glitch_sample(float *array, int size) {
  float sum = 0;
  float mean = 0;
  int idx_max_std = 0;
  float std[size];

  // calculate the std of each sample without its effect on the mean
  for (int j = 0; j < size; j++) {
    // calculate mean without the jth element
    sum = 0;
    for (int i = 0; i < size; i++) {
      if (i != j) {
        sum += array[i];
      }
    }
    mean = sum / (size-1);
    std[j] = pow((array[j] - mean), 2);
    // Serial.print(std[j]); Serial.print(",");
  }
  // Serial.println("");
  // argmax of std
  for (int i = 1; i < size; i++) {
    if (std[i] > std[idx_max_std]) {
      idx_max_std = i;
    }
  }
  // Serial.print("max_std: "); Serial.println(std[idx_max_std]*16384);

  // calculate the mean without the sample with the highest std
  sum = 0;
  for (int i = 0; i < size; i++) {
    if (i != idx_max_std) {
      sum += array[i];
    }
  }
  mean = sum / (size-1);
  return mean;
}

// ================================================================
// === Calculating rotation matrix from quaternion               ===
// ================================================================

void rotation_matrix_from_quaternion(Quaternion *q, float rotation_matrix[3][3]) {
  rotation_matrix[0][0] = 1 - 2 * q->y * q->y - 2 * q->z * q->z;
  rotation_matrix[0][1] = 2 * q->x * q->y - 2 * q->z * q->w;
  rotation_matrix[0][2] = 2 * q->x * q->z + 2 * q->y * q->w;
  rotation_matrix[1][0] = 2 * q->x * q->y + 2 * q->z * q->w;
  rotation_matrix[1][1] = 1 - 2 * q->x * q->x - 2 * q->z * q->z;
  rotation_matrix[1][2] = 2 * q->y * q->z - 2 * q->x * q->w;
  rotation_matrix[2][0] = 2 * q->x * q->z - 2 * q->y * q->w;
  rotation_matrix[2][1] = 2 * q->y * q->z + 2 * q->x * q->w;
  rotation_matrix[2][2] = 1 - 2 * q->x * q->x - 2 * q->y * q->y;
}

void normalize_vector(float *vector){
  float norm = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  vector[0] = vector[0] / norm;
  vector[1] = vector[1] / norm;
  vector[2] = vector[2] / norm;
}

void matrix_multiply_vector(float rotation_matrix[3][3], float vector[3], float *projection_vector) {
  projection_vector[0] = rotation_matrix[0][0] * vector[0] + rotation_matrix[0][1] * vector[1] + rotation_matrix[0][2] * vector[2];
  projection_vector[1] = rotation_matrix[1][0] * vector[0] + rotation_matrix[1][1] * vector[1] + rotation_matrix[1][2] * vector[2];
  projection_vector[2] = rotation_matrix[2][0] * vector[0] + rotation_matrix[2][1] * vector[1] + rotation_matrix[2][2] * vector[2];
}

void inner_product(float *vector1, float *vector2, float *result) {
  *result = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
}

void convert_VectorFloat_to_float(VectorFloat *v, float *f) {
  f[0] = v->x;
  f[1] = v->y;
  f[2] = v->z;
}

// -------------------------------------------------------------------
// 			 GET YPR ANGLES
// -------------------------------------------------------------------
// This simply converts the values from the accel-gyro arrays into degrees.

float get_roll(){
	return (ypr[1] * 180.0/M_PI);
}

float get_pitch(){
	return (ypr[2] * 180.0/M_PI);
}

float get_yaw(){
	return (ypr[0] * 180.0/M_PI);
}

// -------------------------------------------------------------------
// 			 GET Euler ANGLES
// -------------------------------------------------------------------
// This simply converts the values from the accel-gyro arrays into degrees.

float get_psi(){
	return (euler[0] * 180.0/M_PI);
}

float get_phi(){
	return (euler[1] * 180.0/M_PI);
}

float get_theta(){
	return (euler[0] * 180.0/M_PI);
}


// -------------------------------------------------------------------
// 			 GET ACCEL_GYRO DATA
// -------------------------------------------------------------------

void accelgyroData(){

    // Reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    // Serial.println(mpuIntStatus);

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println("Warning - FIFO Overflowing!");

    // otherwise, check for DMP data ready interrupt (this should happen exactly once per loop: 100Hz)
    } else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length, should be less than 1-2ms, if any!
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get sensor data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        rotation_matrix_from_quaternion(&q, rotation_matrix);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetEuler(euler, &q);
        mpu.resetFIFO();

    }
}

void blink(int repeat, int delayTime){
  for (int i = 0; i < repeat; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_PIN, LOW);
    delay(delayTime);
  }
}

void setup() {

    EEPROM.get(0, xAccelOffset);
    EEPROM.get(2, yAccelOffset);
    EEPROM.get(4, zAccelOffset);
    EEPROM.get(6, xGyroOffset);
    EEPROM.get(8, yGyroOffset);
    EEPROM.get(10, zGyroOffset);
    EEPROM.get(12, min_throttle);
    EEPROM.get(14, mid_throttle);
    EEPROM.get(16, max_throttle);
    EEPROM.get(18, avg_size);

    Joystick.begin();
    Joystick.setRyAxisRange(0, 16384); // Throttle!!!
    Joystick.setRxAxisRange(-16384, 16384);
    Joystick.setXAxisRange(-16384, 16384);
    Joystick.setYAxisRange(-16384, 16384);
    // Joystick.setXAxisRange(-1, 1);
    // Joystick.setYAxisRange(-1, 1);
    // Joystick.setRxAxisRange(-1, 1);
    // Joystick.setRyAxisRange(-1, 1);
    pinMode(BEND_Vin, OUTPUT);
    digitalWrite(BEND_Vin, HIGH);
    pinMode(LED_PIN, OUTPUT);
    blink(2, 10);
    Wire.begin();
    ppmEncoder.begin(PPM_PIN, 8);
  
    // Initialize serial communication for debugging
    Serial.begin(115200);
    // while (!Serial); 
    // Initialize MPU6050
    mpu.initialize();

    Serial.println("Testing MPU connection:");
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    
    Serial.println(mpu.testConnection() ? "> MPU6050 connection successful" : "> MPU6050 connection failed");
    Serial.println("Initialising DMP");
    devStatus = mpu.dmpInitialize();
    delay(100);

    /* * * * * * * * * * * * * * * * * * * *
     * IMPORTANT!
     * Supply your own MPU6050 offsets here
     * * * * * * * * * * * * * * * * * * * */

    Serial.print("offsets:\t");
    Serial.print(xAccelOffset); 
    Serial.print("\t");
    Serial.print(yAccelOffset); 
    Serial.print("\t");
    Serial.print(zAccelOffset); 
    Serial.print("\t");
    Serial.print(xGyroOffset); 
    Serial.print("\t");
    Serial.print(yGyroOffset); 
    Serial.print("\t");
    Serial.print(zGyroOffset); 
    Serial.print("\t");
    Serial.print(min_throttle); 
    Serial.print("\t");
    Serial.print(max_throttle); 
    Serial.print("\t");
    Serial.println(avg_size); 
    Serial.println(""); 

    delay(50);

    mpu.setXAccelOffset(xAccelOffset);
    mpu.setYAccelOffset(yAccelOffset);
    mpu.setZAccelOffset(zAccelOffset);
    mpu.setXGyroOffset(xGyroOffset);
    mpu.setYGyroOffset(yGyroOffset);
    mpu.setZGyroOffset(zGyroOffset);

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        Serial.println("Enabling DMP");
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println("DMP Ready! Let's Proceed.");
        Serial.println("GyroGlove is now ready to be used!");
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

    } else {
		// In case of an error with the DMP
        if(devStatus == 1) Serial.println("> Initial Memory Load Failed");
        else if (devStatus == 2) Serial.println("> DMP Configuration Updates Failed");
    }
    t0 = micros();
    accelgyroData();
    t1 = micros();
    for (int i=0; i<avg_size; i++){
        throttle_avg += analogRead(bendPin)/avg_size;
    }
    acceleration[0] = gravity_const * aaWorld.x/16384.0;
    acceleration[1] = gravity_const * aaWorld.y/16384.0;
    acceleration[2] = gravity_const * aaWorld.z/16384.0;
    dt = (t1 - t0) / 1000000.0;
    
    for (int i=0; i<3; i++){
        velocity[i] += gravity_const * acceleration[i] * dt;
        position[i] += velocity[i] * dt;
    }
    t0 = t1; 
}

void loop() {
  	// Gather data from MPU6050
	accelgyroData();
  t1 = micros();
  // Calculating Axes from orientation
  matrix_multiply_vector(rotation_matrix, y_vector, y_projection_vector);
  matrix_multiply_vector(rotation_matrix, x_vector, x_projection_vector);
  inner_product(y_projection_vector, x_vector, &yx_projection);
  inner_product(y_projection_vector, z_vector, &yz_projection);
  inner_product(x_projection_vector, z_vector, &xz_projection);

  // Calculating throttle average of the last avg_size samples
  dt = (t1 - t0) / 1000000.0;
      for (int i=0; i<avg_size; i++){
          raw_throttle = analogRead(bendPin);
          throttle_avg = (throttle_avg * (avg_size-1) + raw_throttle) / avg_size;
      }
  throttle = constrain(16 * throttle_avg, min_throttle, max_throttle);
  if (throttle < mid_throttle) {
    throttle = map(throttle, min_throttle, mid_throttle, 0, 8192);
  }
  else {
    throttle = map(throttle, mid_throttle, max_throttle, 8192, 16384);
  }
  throttle = 16384 - throttle;

  // Calculating absolute world position and velocity
  acceleration[0] = gravity_const * aaWorld.x/16384.0;
  acceleration[1] = gravity_const * aaWorld.y/16384.0;
  acceleration[2] = gravity_const * aaWorld.z/16384.0;
  for (int i=0; i<3; i++){
      velocity[i] =  0.98 * velocity[i] + acceleration[i] * dt;
      position[i] += velocity[i] * dt;
  }
  t0 = t1;
  // roll = get_roll() - roll0;
  // pitch = get_pitch() - pitch0;
  // yaw = get_yaw() - yaw0;
  // roll = get_roll();
  // pitch = get_pitch();
  // yaw = get_yaw();

  // Serial.print("T: "); Serial.print(throttle); //Serial.print(",");
  /*
  Serial.print("quaternion: ");
  Serial.print("qw: "); Serial.print(16384*q.w); Serial.print(",");
  Serial.print("qx: "); Serial.print(16384*q.x); Serial.print(",");
  Serial.print("qy: "); Serial.print(16384*q.y); Serial.print(",");
  Serial.print("qz: "); Serial.print(16384*q.z);
  Serial.println("");
  */

  /*
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
  */

  /*
  Serial.print("Rotation matrix: ");
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      Serial.print(16384.0 * rotation_matrix[i][j]);
      Serial.print("\t");
    }
  }
  Serial.println("");
  */

  /*
  Serial.print("Position: ");
  for (int i=0; i<3; i++){
    Serial.print(position[i] * 16384);
    Serial.print("\t");
  }
  Serial.println("");
  Serial.print("Velocity: ");
  for (int i=0; i<3; i++){
    Serial.print(velocity[i] * 16384);
    Serial.print("\t");
  }
  Serial.println("");
  Serial.print("Acceleration: ");
  for (int i=0; i<3; i++){
    Serial.print(acceleration[i] * 16384);
    Serial.print("\t");
  }
  Serial.println("");
  */
  
  /*
  Serial.print("yaw: "); Serial.print(yx_projection); Serial.print(",");
  Serial.print("pitch: "); Serial.print(yz_projection); Serial.print(",");
  Serial.print("roll: "); Serial.print(xz_projection); Serial.print(",");
  //Serial.print("throttle : "); Serial.print(throttle);  Serial.println("");
  */
  
  //  Exclude glitched sample from measurement:
  yaw_array[counter % size] = yx_projection;
  pitch_array[counter % size] = yz_projection;
  roll_array[counter % size] = xz_projection;
  counter ++;
  yawStick = exclude_glitch_sample(yaw_array, size) * 16384;
  pitchStick = -1 * exclude_glitch_sample(pitch_array, size) * 16384;
  rollStick = exclude_glitch_sample(roll_array, size) * 16384;

  // Serial.print("yaw: "); Serial.print(yawStick); Serial.print(",");
  // Serial.print("pitch: "); Serial.print(pitchStick); Serial.print(",");
  // Serial.print("roll: "); Serial.print(rollStick); Serial.println("");
  if (counter==16384){
    counter = 0;
  }
  
  // yawStick = yx_projection * 16384;
  // pitchStick = -1.0 * yz_projection * 16384; // reversed pitch feels more natural.
  // rollStick = xz_projection * 16384;
  Joystick.setRxAxis(yawStick);
  Joystick.setXAxis(rollStick);
  Joystick.setYAxis(pitchStick);
  Joystick.setRyAxis(throttle);
  ppmEncoder.setChannel(3, map(yawStick, -16384, 16384, 1000, 2000));
  ppmEncoder.setChannel(1, map(rollStick, -16384, 16384, 1000, 2000));
  ppmEncoder.setChannel(2, map(pitchStick, -16384, 16384, 1000, 2000));
  ppmEncoder.setChannel(0, map(throttle, 0, 16384, 1000, 2000));

}
