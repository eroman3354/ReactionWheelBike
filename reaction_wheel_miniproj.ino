#include <Wire.h>
#include <EEPROM.h>

#define ENC_1 /*pin of gyro*/
#define ENC_2 /*pin of accelerometer*/

#define MPU6050 0x68 // Device I2C address
#define VBAT /*pin of battery*/
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

float Gyro_amount = 0.96;

float GyroX, AccY, AccZ;  // only need rotation about x axis and accel along y and z axes
float GyroAngle, AccAngle; // for angle conversion
float GyroErrorX, AccErrorY, AccErrorZ; // error from 0 (unless we calibrate and it isnt 0) 
float elapsedTime, currentTime, previousTime;
float error;

// proportionality of p vs i vs d on motor control
float Kp = 0.7;
float Ki = 0;
float Kd = 0.3;

bool calibrating = false;
bool calibrated = false;

int c = 0;                // used for fnc calculate_IMU_error (calibration)

struct OffsetsObj {
  int ID;
  int16_t AcY;
  int16_t AcZ;
  int16_t GyX;
};
OffsetsObj offsets;       // object of calibrated accel values to be saved permanently


void setup() {
  //Calibration so the robot knows what is 0 from the gyro
  Serial.begin(19200);
  Wire.begin();           // initialize communication
  Wire.beginTransmission(MPU6050); // MPU6050 = 0x68
  Wire.write(0x6B);       // talk to register 6B ???

  Wire.endTransmission(true);
  
  EEPROM.get(0, offsets);
  if (offsets.ID == 35) calibrated = true;     // if offset 35 exists, bike is calibrated
    else calibrated = false;

  delay(20);
}

void loop() {
  read_MPU_data();

  
}

void read_MPU_data(){
  // === Read MPU6050 data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_YOUT_H);                 // write to accell Y                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  
  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_ZOUT_H);                 // write to accell AcZ            
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  Wire.beginTransmission(MPU6050);
  Wire.write(GYRO_XOUT_H);                 // write to gyro X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true); 
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;    // get GyX from wire

  // convert Acc readings to degrees
  GyroAngle = GyroAngle + GyroX * elapsedTime;
  AccAngle = -atan2(AccY - offsets.AcY, -(AccZ - offsets.AcZ)) * 180 / PI; // may need to change logic depending on how offsets are calculated
  
  error = GyroAngle * Gyro_amount + AccAngle * (1 - Gyro_amount); // how gyro/accel are weighted into robot angle
}

void pid_controller(){
  // integral used for tuning values while running
  // derivative used to determine speed of reaction wheel
}

void save(){
  /*
  save the calibrated values in EEPROM to reuse after turning the MPU6050 off and on again
  */
  EEPROM.put(0, offsets);
  delay(100);
  EEPROM.get(0, offsets);
  if (offsets.ID == 35) calibrated = true;
  calibrating = false;
  Serial.println("calibrating off");
}

/*int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':                               // K1 is Kp
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      printValues();
      break;
    case 'i':                               // K2 is Ki
      if (cmd == '+')    K2 += 0.5;
      if (cmd == '-')    K2 -= 0.5;
      printValues();
      break;
    case 's':                               // K3 is Ks
      if (cmd == '+')    K3 += 0.2;
      if (cmd == '-')    K3 -= 0.2;
      printValues();
      break;  
    case 'a':                               // K4 is Ka
      if (cmd == '+')    K4 += 0.05;
      if (cmd == '-')    K4 -= 0.05;
      printValues();
      break;    
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        // calibrated
          offsets.ID = 35;
          offsets.AcZ = AcZ + 16384;
          offsets.AcY = AcY;
          Serial.print("AcY: "); Serial.print(offsets.AcY); Serial.print(" AcZ: "); Serial.println(offsets.AcZ); 
          save();     // saves tuning params to  EEPROM, switches callibrating to false
      }
      break;                
   }
   return 1;
}*/
