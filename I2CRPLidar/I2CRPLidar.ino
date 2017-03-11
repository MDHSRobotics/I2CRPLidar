// ---------------------------------------------------------------------------
// Example I2C Arduino slave 
// ---------------------------------------------------------------------------

/*
 * RoboPeak RPLIDAR Arduino Example
 * This example shows the easy and common way to fetch data from an RPLIDAR
 * 
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3 
 */
 
/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
//#include <RPLidar.h>
#include <Wire.h>


// You need to create an driver instance 
//RPLidar lidar;

#define SLAVE_ADDRESS 0x41
#define STATUS_I2C_DATA_READY             0x41
#define STATUS_I2C_DATA_READY_LAST_SET    0xC1
#define STATUS_I2C_DATA_NOT_READY         0x00
#define LOOP_DELAY_DEFAULT          50       // Wait 50us, need to assess correct loop delay

/********* REGISTER DEFINITIONS **********/
// The lidar will stream a bunch of data points
// each data point consists are 10 bytes in size and consists of the following
#define SYNCH_SIZE 1            //SYNCH is a 1 byte flag that indicates the angle that is the beginning of a scan
#define QUAL_SIZE  1            //QUAL  is a 1 byte flag that indicates whether the lidar was able to make a measurement at the specified angle of the current point
#define ANGLE_SIZE 4            //ANGLE is a 4 byte field that indicates the angle of the current point
#define DIST_SIZE  4            //DIST  is a 4 byte field that indicates distance measurement at the specified angle of the current point
#define I2C_WORD_SIZE 4         //WORD is a general purpose 4 byte field

// the register is optimized for SMBus protocol which defines block reads as 32 byte blocks
// a register can hold up to 3 data points
#define STATUS_SIZE 1           //status is a 1 byte field that indicates the status of the I2C device, e.g. whether data is ready for the host to pick up
#define COUNT_SIZE  1           //count  is a 1 byte field that indicates the number of points sent in this register
                                //up to 3 points can be transmitted in each register read
#define RESERVED_SIZE 2         //2 spare bytes reserved for future use
#define POINT_SIZE  10          //point  is a 10 byte field that stores the values associated with a point
#define READ_REGISTER_SIZE  32  //the read register is a 32 byte register optimized for block read
                                //it incorporates the status and the data elements, count and 6 data points
#define POINT_LIMIT 3           //a limit of 6 points can be held in a buffer

//REGISTER ADDRESSES
//FRONT PART is the read register
#define READ_REGISTER      0x00     
#define STATUS_ADDRESS     READ_REGISTER                    //1 byte status positioned at the beginning of the READ_REGISTER
#define COUNT_ADDRESS      STATUS_ADDRESS + STATUS_SIZE     //1 byte count positioned after status
#define POINT1_ADDRESS     COUNT_ADDRESS  + COUNT_SIZE      //10 byte point positioned after count
#define POINT2_ADDRESS     POINT1_ADDRESS + POINT_SIZE      //10 byte point positioned after Point1
#define POINT3_ADDRESS     POINT2_ADDRESS + POINT_SIZE      //10 byte point positioned after Point2
//#define POINT4_ADDRESS     POINT3_ADDRESS + POINT_SIZE      //10 byte point positioned after Point3
//#define POINT5_ADDRESS     POINT4_ADDRESS + POINT_SIZE      //10 byte point positioned after Point4
//#define POINT6_ADDRESS     POINT5_ADDRESS + POINT_SIZE      //10 byte point positioned after Point5

//After the read register is where the write registers and the address are
#define MODE_ADDRESS        READ_REGISTER  + READ_REGISTER_SIZE //4 byte word to indicate allow host to configure device
#define CONFIG_ADDRESS      MODE_ADDRESS   + I2C_WORD_SIZE      //4 byte word to indicate allow host to configure device
#define ID_ADDRESS          CONFIG_ADDRESS + I2C_WORD_SIZE      //4 byte word to indicate the device address.  least significant bits are used for the address.

#define REGISTER_SIZE READ_REGISTER_SIZE + 3*I2C_WORD_SIZE
#define TEMP_REGISTER_SIZE REGISTER_SIZE - I2C_WORD_SIZE

#define MODE_PENDING 0
#define MODE_STREAMING 1
#define MODE_SIMULATION 2

/********* I/O PIN DEFINITIONS **********/
#define SLAVE_STATUS_LED 13
#define RPLIDAR_MOTOR 3

/********* COMMANDS *********************/

/********* Global  Variables  ***********/

unsigned char registerMap[REGISTER_SIZE];                        //the actual interface register
unsigned char registerMapTemp[REGISTER_SIZE - I2C_WORD_SIZE];    //register where data is written to before being published to the interface register
unsigned char deviceStatus = STATUS_I2C_DATA_NOT_READY;
bool newDataAvailable = false; 
bool configModeUpdated = false;
long previousMillis = 0;        // will store last timestamp
unsigned char currentPoint = 0;
unsigned int scanCount = 0;
bool isScanning = false;
unsigned char mode = MODE_PENDING;

//commands can be sent from the host to the slave
//a read command has a 1 byte argument that indicates where in the register we want to read from
//a write command has an address and a word, it'll either be a mode word or a config word, or a write to both
//so the max data payload size of the command is the size of 2 words + 1 address byte
#define COMMAND_SIZE 1+2*I2C_WORD_SIZE
byte receivedCommands[COMMAND_SIZE];    

// mode structure  4 bytes
// 0 - command , //2 byte mode setting
// 1 - data 1 ,  //2 byt mode value
unsigned int mode[I2C_WORD_SIZE / sizeof(unsigned int)];

// config structure
// 0 - command , //2 byte config setting
// 1 - data 1 ,  //2 byt config value
unsigned int configuration[I2C_WORD_SIZE / sizeof(unsigned int)];

void setup() {
  //Set up the Serial Monitor
  //The serial monitor is used to dosplay log messages and enable us to see what is happening inside the Arduini.  It is the main debugging tool
  //This can be disabled or commented out when debugging is no longer needed
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.

  Serial.println("Initializing subsystems ...");
  Serial.print("register size is ");
  Serial.println(REGISTER_SIZE);
  Serial.print("temp register size is ");
  Serial.println(TEMP_REGISTER_SIZE);
  Serial.print("ID ADDRESS is ");
  Serial.println(ID_ADDRESS);
  newDataAvailable = false;
  registerMapTemp[0] = STATUS_I2C_DATA_NOT_READY;
  registerMap[0] = STATUS_I2C_DATA_NOT_READY;
  setupI2C();
  setupPins();
  mode = MODE_PENDING;
  printMode();
  // bind the RPLIDAR driver to the arduino hardware serial
//  lidar.begin(Serial);
//  
//  // set pin modes
//  pinMode(RPLIDAR_MOTOR, OUTPUT);
//  pinMode(DISTANCE_LED, OUTPUT);
  float x = 41.41;
  Serial.print("x: ");
  Serial.println(x);

  unsigned char* p_x = floatToByteArray(x);

  
  for(int i=0;i<sizeof(float);i++){
    Serial.print("b[");
    Serial.print(i);
    Serial.print("]=");
    Serial.print((int)(*(p_x+i)));
    Serial.print(" ");
  }
    Serial.println(" ");
  float x1 = byteArrayToFloat(p_x);
  Serial.print("x: ");
  Serial.println(x1);
}

void printMode(){
  Serial.print("mode: ");
  switch(mode){
    case MODE_PENDING:
      Serial.print("pending...");
      break;
    case MODE_STREAMING:
      Serial.print("streaming...");
      break;
    case MODE_SIMULATION:
      Serial.print("simulation...");
      break;
  }
  Serial.println();
}

void  setupPins(){
  Serial.println("Setting up pins");
  pinMode(SLAVE_STATUS_LED,OUTPUT);
  digitalWrite(SLAVE_STATUS_LED,LOW);
}


void setupI2C(){
  Serial.println("setting up I2C");
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  randomSeed(analogRead(0));
}

char tmp[2];
void requestEvent(){
  Serial.println("request event");
  Serial.print("requested address: ");
  sprintf(tmp,"%02x",receivedCommands[0]);
  Serial.println(tmp);
  unsigned char regstatus;
  unsigned char count;
  unsigned char p1[POINT_SIZE];
  unsigned char p2[POINT_SIZE];
  unsigned char p3[POINT_SIZE];

  regstatus = registerMap[0];
  count = registerMap[1];
  for(unsigned char i=0;i<POINT_SIZE;i++){
    p1[i] = registerMap[i+2];
    p2[i] = registerMap[i+2+POINT_SIZE];
    p3[i] = registerMap[i+2+2*POINT_SIZE];
  }
  Serial.print("status: ");
  Serial.println(regstatus);
  Serial.print("count: ");
  Serial.println(count);
  Serial.print("p1: ");
  for(unsigned char i=0;i<POINT_SIZE;i++){
    sprintf(tmp,"%02x",p1[i]);
    Serial.print(tmp);
    Serial.print(" ");
  }
  Serial.print("\t");
  printPoint(p1);
  Serial.print("p2: ");
  for(unsigned char i=0;i<POINT_SIZE;i++){
    sprintf(tmp,"%02x",p2[i]);
    Serial.print(tmp);
    Serial.print(" ");
  }
  Serial.print("\t");
  printPoint(p2);
  Serial.print("p3: ");
  for(unsigned char i=0;i<POINT_SIZE;i++){
    sprintf(tmp,"%02x",p3[i]);
    Serial.print(tmp);
    Serial.print(" ");
  }
  Serial.print("\t");
  printPoint(p3);
  //now test parse the points...
  
  if(receivedCommands[0]==0){
    for(unsigned char i= 0; i< READ_REGISTER_SIZE;i++){
      sprintf(tmp,"%02x",registerMap[i]);
      Serial.print(tmp);
      if(((i+1)%8)==0) Serial.print("\n");
      else Serial.print(" ");
    }
    Wire.write(registerMap, READ_REGISTER_SIZE);  //Set the buffer up to send all 64 bytes of data of the read register
    newDataAvailable = false;
    registerMap[0] = STATUS_I2C_DATA_NOT_READY;
  }
  if(receivedCommands[0]==MODE_ADDRESS){
    
  }
}

void receiveEvent(int bytesReceived){
  Serial.println("receive event");
  for (uint8_t a = 0; a < bytesReceived; a++){
    if ( a < COMMAND_SIZE){
         receivedCommands[a] = Wire.read();
    }
    else{
         Wire.read();  // if we receive more data then allowed just throw it away
    }
  }
  if(bytesReceived == 1 && (receivedCommands[0] < REGISTER_SIZE))
  {
      //request is a read from the specified address
      return; 
  }
  if(bytesReceived == 1 && (receivedCommands[0] >= REGISTER_SIZE))
  {
      //request is a read to an invalid address, reset the address to 0
      receivedCommands[0] = 0x00;
      return;
  }

  byte address = receivedCommands[0];
  Serial.print("received more than 1 byte"); //meaning that the master is writing to the slave.
  Serial.print(", address writing to is: 0x");
  Serial.print(address,HEX);
  Serial.println();
  if(address < CONFIG_ADDRESS || address >= ID_ADDRESS){
//  we can only write to either the config or mode registers
//  Serial.println('address is not a valid write address');
    return;
  }

  if(address == CONFIG_ADDRESS){
    //we are receiving a config word to write into the config register
    memcpy(configuration, receivedCommands+1, I2C_WORD_SIZE);
  }
  else if(address == MODE_ADDRESS){
    //we are either receiving a mode word or a mode word and a config word to write into the register
    memcpy(mode, receivedCommands+1, bytesReceived-1);
  }
  else{
//  we can only write to either the config or mode registers
//  Serial.println('address is not a valid write address');
    return;
  }
  configModeUpdated = true;  //we had a valid request to either update configuration or mode
}


#define PERIOD 1000 //period defined as 1s, aka 1000ms

void loop() {
  //Serial.println("looping...");

  if(configModeUpdated){
    configModeUpdate();
  }
  unsigned long currentMillis = millis();
  if(currentMillis-previousMillis>PERIOD){
    scan(); //start a simulated scan on a periodic basis
    previousMillis = currentMillis;
  }
  if(isScanning){
    readPoint();
  }


//    Serial.print("reg status:  ");
//    Serial.print(registerMap[0]);
//    Serial.print("\t");
//    Serial.println(registerMapTemp[0]);
    if((registerMapTemp[0]==STATUS_I2C_DATA_READY || registerMapTemp[0]==STATUS_I2C_DATA_READY_LAST_SET) && registerMap[0]==STATUS_I2C_DATA_NOT_READY){
      Serial.println("swapping registers ...");
      //temp buffer ready to be swapped with register
      memcpy(registerMap,registerMapTemp,TEMP_REGISTER_SIZE);
      newDataAvailable = true;
      registerMapTemp[0] = STATUS_I2C_DATA_NOT_READY;
      currentPoint = 0;
      Serial.println("data set ready");
    }

//  delayMicroseconds(LOOP_DELAY_DEFAULT);                      
//  if (IS_OK(lidar.waitPoint())) {
//    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
//    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
//    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
//    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
//    
//    //perform data processing here... 
//    if(quality > 0 && angle >=0 && angle <= 30){
//          digitalWrite(STATUS_LED,HIGH);
//          if(distance > 300){
//            digitalWrite(DISTANCE_LED,HIGH);
//          }
//        else{
//          digitalWrite(DISTANCE_LED,LOW);
//        }
//    }
//    
//  } else {
//    digitalWrite(STATUS_LED,LOW);
//    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
//    
//    // try to detect RPLIDAR... 
//    rplidar_response_device_info_t info;
//    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
//       // detected...
//      lidar.startScan();
//       // start motor rotating at max allowed speed
//       analogWrite(RPLIDAR_MOTOR, 255);
//       delay(LOOP_DELAY_DEFAULT);                      // Wait between pings. 29ms should be the shortest delay between pings.
//    }
//  }
}



void configModeUpdate(){
    configModeUpdated = false;
    switch(mode[0]){
      case LED_COMMAND:
        switch(mode[1]){
          case 0x4800:
          digitalWrite(SLAVE_STATUS_LED,HIGH);
          break;
          case 0x4C00:
          digitalWrite(SLAVE_STATUS_LED,LOW);
          break;
        }
        break;
    }

//    printMode();
}


void printMode(){
  char tmp[85];
  sprintf(tmp,"mode: [%.4X %.4X %.4X]",mode[0],mode[1],mode[2]);
  Serial.print(tmp);
}

void printPoint(unsigned char* point){
  float angle;
  float distance;

  angle = byteArrayToFloat(point+2);
  distance = byteArrayToFloat(point+2+sizeof(float));

  unsigned char s = *(point);
  unsigned char q = *(point+1);
  Serial.print("S:");
  Serial.print((int)s);
  Serial.print(" A:");
  Serial.print(angle);
  Serial.print(" D:");
  Serial.print(distance);
  Serial.print("Q:");
  Serial.print((int)q);
  Serial.println();
}

//Create an observable angular test pattern with motion
#define MEAN_DISTANCE 1500 //average distance in a scan
#define CYCLES_PER_SCAN 8  //8 cycles in each scan
#define PATTERN_PERIOD 16  //takes 16 scans to complete pattern
#define PATTERN_HEIGHT 500 
#define PROBABILITY_BAD_POINT 33

float calculateDistance(float angle,unsigned int count){
  //distance will be a factor of angle and scan count, where the pattern will be changed by the angle and the count.  the pattern will repeat every PATTERN_PERIODS
  float distance = MEAN_DISTANCE;
  count = count %16;
  float height = PATTERN_HEIGHT * cos(angle / 180.0f * M_PI ) * cos(count /8 * M_PI);
  distance += height;
  return distance;
}

bool synchObtained = false;

void scan(){
    //for now mock a scan
    isScanning = true;
    synchObtained = false;
    Serial.println("scan ...");
}

void readPoint(){
  //in order to allow multiple things to occur concurrently, we need to get one point at a time and let the MC do other things in between points
  //assume we get 360 points per scan

  //we're getting them here 1 point at a time

  if(!nextPoint()){
    isScanning = false;
    scanCount++;
    scanCount = scanCount % PATTERN_PERIOD;
  }  
}

unsigned int pointCount = 0;
#define MAX_ANGLE 360
#define START_ANGLE 0.5


byte point[POINT_SIZE];  //buffer to hold point value

bool nextPoint(){
  //get the next point
  //returning false indicates the end of the scan

  float angle = (float)pointCount + START_ANGLE;
  if(angle>=MAX_ANGLE){
    registerMapTemp[0] = STATUS_I2C_DATA_READY_LAST_SET;
    pointCount = 0;
    return false;
  }
  byte qual = (random(100)<=PROBABILITY_BAD_POINT?0x00:0x47);
  bool synch = false;
  float distance;
  distance = 0;
  if(qual!=0){
    if(!synchObtained){
      synch = true;
      synchObtained = true;      
    }
    distance = calculateDistance(angle,scanCount);
  }
  writePoint(angle,distance,synch, qual, point);
  //if I can put it in the temp register, go ahead and do it
  //put it in the temp register at the right spot
  if(currentPoint < POINT_LIMIT){
    Serial.print(currentPoint);
    Serial.print(" - ");
    printPoint(point);
    memcpy(registerMapTemp+2 + currentPoint*POINT_SIZE,point,POINT_SIZE);
    currentPoint++;
    registerMapTemp[1] = currentPoint;
    if(currentPoint == POINT_LIMIT){
       registerMapTemp[0] = STATUS_I2C_DATA_READY;
    }  
  }

  pointCount++;
  return true;
}

void writePoint(float angle,float distance,bool synch, byte qual, byte* point){
  unsigned char offset=0;
  *(point+offset) = (byte)synch;
  offset++;
  
  *(point +offset) = qual;
  offset++;
  unsigned char * angleBytes = floatToByteArray(angle);
  unsigned char * distanceBytes = floatToByteArray(distance);
  memcpy(point+offset, angleBytes, sizeof(float));  
  offset = offset + sizeof(float);
  memcpy(point+offset, distanceBytes, sizeof(float));  
  char tmp[16];
}

unsigned char * floatToByteArray(float f) {
  return reinterpret_cast<unsigned char *>(&f);
}

float byteArrayToFloat(byte* p_bytes) {
  return *(reinterpret_cast<float *>(p_bytes));
}

