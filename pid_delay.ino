
float actual, currError, prevError, currTime, prevTime, dt, errIntegral, output, correction;
float kp=0.0005, kd=0.5, ki=0;
float reference=0;
bool is_good=true, is_start=false;
int cutoffPin=2;

#include <Wire.h>
#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

int potValue;  // value from the analog pin 
float maxOutput=55;
//float maxOutput=65;
float minOutput=0;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

float avgRoll=0;

void setup(){
  Serial.begin(19200);
  Serial.println("Setup");
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  Serial.println("I2c Good");
  pinMode(cutoffPin, INPUT);

  
  ESC.attach(9,1000,1100); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(0); 
  Serial.println("Waiting 2 seconds   to start");
  //MAKE SURE IMU IS FLAT AND STATIONARY
  Serial.println("Callibration start");
  calculate_IMU_error();
  Serial.println("Callibration over");


}

void get_accAngle(){ //updates accAngleX, accAngleY
    Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384 (LSB value), according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data 
  accAngleX = ((atan(AccY / AccZ)) * 180 / PI); //- 0.12; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = atan(-AccX/AccZ)*180/PI; //(atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.25; // AccErrorY ~(-1.58)
}

void get_gyro_reading(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; 
  GyroY -= GyroErrorY; 
  GyroZ -= GyroErrorZ; 

  //time elapsed
  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000; 

  // Integrating the gyro readings(angular velocity in deg/s) to get angular velocity
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
}

void calculate_IMU_error() { // update AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  
  //Steady state zero error
  int num_trials=500;
  while (c < num_trials) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  AccErrorX = AccErrorX / num_trials;
  AccErrorY = AccErrorY / num_trials;
  c = 0;

  while (c < num_trials) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  GyroErrorX = GyroErrorX / num_trials;
  GyroErrorY = GyroErrorY / num_trials;
  GyroErrorZ = GyroErrorZ / num_trials;

}

void loop(){
    // === Read acceleromter data === //
    get_accAngle();
  
    // === Read gyroscope data === //
    get_gyro_reading();

    // Complementary filter - combine acceleromter and gyro angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    avgRoll = 0.99*avgRoll+0.01*roll;
    
    prevTime=currTime;
    currTime=millis();
    dt = currTime-prevTime;
  
    actual=roll;
    prevError=currError;  
    currError = reference-actual;
  
    errIntegral += dt*currError;
  
    correction = kp*currError + kd*(currError-prevError)/dt + ki*errIntegral;
    output+=correction;
  
    //Output limiting
    output=min(output, maxOutput);
    output=max(output, minOutput);

  
    //check cutoff and imu input
    if(digitalRead(cutoffPin)&&is_good){
      Serial.println("Cutoff");
      Serial.println(roll);
      Serial.println(output);
      is_good=false;
      ESC.write(0);
    }
    if(roll!=roll&&is_good){ 
      is_good=false;
      ESC.write(0);
      Serial.println("Bad imu input");
    }

    if(roll==roll && is_good){
    ESC.write(output);
    }
    
    //NO output if IMU reading is not proper - Need to check if this works
    if(false){//if(roll==roll && is_good){
    Serial.print(avgRoll);
    Serial.print("/");
    Serial.print(roll);
    Serial.print("/");
    Serial.println(output);
    }

}
