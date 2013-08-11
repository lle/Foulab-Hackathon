//Source: http://forum.arduino.cc/index.php?topic=95022.15;wap2

#include <Wire.h>
#include <Servo.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int debug = 0;

//Gyroscope Stuff
int L3G4200D_Address = 105; //I2C address of the L3G4200D
int x;
int y;
int z;
int gyroThreshold = 300;

//Servo Stuff
Servo servoEars1;
Servo servoEars2;
int totalAngle;
int servoAngle;
int angleRemainder = 15;
int maxAngle = 140;

void setup(){
  //Determine if in debug mode
  int debug = 0;
  pinMode(12, INPUT);
  digitalWrite(12, HIGH);
  if(digitalRead(12)==LOW)
    debug = 1;  //Only start DEBUG MODE if pin12 is low. Thus, if you plugged pin12 to GND.
  
  //Setup Servo
  servoEars1.attach(9);
  servoEars2.attach(10);
  
  //Setup Gyro
  Wire.begin();
  
  //Setup Serial
  //Initialize Serial
  if(debug == 1)  //Serial.print slow down the servo motor update, only activate it in debug environment
    Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  
  for(int i=0; i<maxAngle; i++)
  {
    servoEars1.write(i);
    servoEars2.write(i);
    delay(1);
  }
  delay(300);
  for(int i=maxAngle; i>0; i--)
  {
    servoEars1.write(i);
    servoEars2.write(i);
    delay(1);
  }
  
  delay(250); //wait for the sensor to be ready 
}

void loop(){
  getGyroValues();  // This will update x, y, and z with new values
  Serial.print("\t X:");
  Serial.print(x);
  Serial.print("\t\t Y:");
  Serial.print(y);
  Serial.print("\t\t Z:");
  Serial.print(z);  

  //Calculate Total Angle  
  if(x>gyroThreshold || y>gyroThreshold || z>gyroThreshold)
  {
     servoAngle = servoAngle + (findMax(x,y,z) % angleRemainder); 
     Serial.print("\t\t +");
     Serial.print(x%angleRemainder);
  }
  else if(x<-gyroThreshold || y<-gyroThreshold || z<-gyroThreshold)
  {
     servoAngle = servoAngle + (findMin(x,y,z) % angleRemainder); ;
     Serial.print("\t\t ");
     Serial.print(x%angleRemainder);
  }
  else
  {
    Serial.print("\t 00");
  }
  
  //Angle Limiter
  if(servoAngle > maxAngle)
  {
    servoAngle = maxAngle;
  }
  else if(servoAngle < 0)
  {
    servoAngle = 0;
  }
  
  //Apply Angle
  servoEars1.write(servoAngle); 
  servoEars2.write(servoAngle); 
  
  Serial.print("\t\t S:");
  Serial.print(servoAngle);
  Serial.print("\n");
  
  delay(50); //Just here to slow down the serial to make it more readable
}

int findMax(int x, int y, int z)
{
  int maxValue = 0;
  if(x > y)
  { maxValue = x; }
  else
  { maxValue = y; }
  
  if(maxValue<z)
  { maxValue = z; }
  
  return maxValue;
}

int findMin(int x, int y, int z)
{
  int minValue = 0;
  if(x > y)
  { minValue = y; }
  else
  { minValue = x; }
  
  if(minValue>z)
  { minValue = z; }
  
  return minValue;
}

void getGyroValues(){
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);
  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);
  if(scale == 250)  writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  if(scale == 500)  writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  if(scale == 2000) writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 1); // read a byte
    while(!Wire.available()) {} // waiting
    v = Wire.read();
    return v;
}

