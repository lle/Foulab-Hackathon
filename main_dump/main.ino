//Source: http://forum.arduino.cc/index.php?topic=95022.15;wap2

#include <Wire.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;
//int totalAngle;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); //wait for the sensor to be ready 
}

void loop(){
  getGyroValues();  // This will update x, y, and z with new values
  Serial.print("\t X:");
  Serial.print(x);
  Serial.print("\t Y:");
  Serial.print(y);
  Serial.print("\t Z:");
  Serial.print(z);  
  /*
  totalAngle = x + y + z;
  Serial.print("\t T: ");
  Serial.print(totalAngle);
  Serial.print("\n");
  */
  
  delay(100); //Just here to slow down the serial to make it more readable
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

