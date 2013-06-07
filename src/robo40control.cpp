#include "Arduino.h"
#include "Wire.h"

//#include "digitalWriteFast.h"  //no need as we use analogRead (unfortunately..)

#include "robo40control.h"

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

aJsonStream serial_stream(&Serial);
//aJsonStream *jsonStream;
//Stream *serial_stream;

int HMC6352Address = 0x42;
// This is calculated in the setup() function
int slaveAddress;
//int ledPin = 13;
//boolean ledState = false;
byte headingData[2];
int headingValue;
int headingHistory[MAX_HEADING_HISTORY];
long headingAvg = -1L;
int headingMax = -1;
int headingMin = 10000;

bool ag_connected = false;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;
int agValue[6];
int agHistory[6][MAX_AG_HISTORY];
long agAvg[6];
int agMax[6];
int agMin[6];

volatile int _LeftEncoderTicks = 0;
volatile int _RightEncoderTicks = 0;

int CSense1 = 0;
int CSense2 = 0;
int ReportC1 = 0;
int ReportC2 = 0;

int curSpeedMotor1 = 0;
int curSpeedMotor2 = 0;
int curSpeedMotor3 = 0;

int notagain = 0;

int curleftSpeed = 0;
int currightSpeed = 0;


// --------------------------------------------------------------------
void setup() {
  //connect to computer, uses pin ? and ?
  Serial.begin(115200);
  
  // connection to compass, uses pins 20 and 21
  Wire.begin();
  
  /*//connect to bluetooth, uses pin ? and ?
  Serial1.begin(57600);
  //  Serial1.flush();	
  //  Serial1.setTimeout(10);*/
  

  //setup compass
  // Shift the device's documented slave address (0x42) 1 bit right
  // This compensates for how the TWI library only wants the
  // 7 most significant bits (with the high bit padded with 0)
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
  resetCompass();

  //setup accelero/gyro
  // initialize device
  // Serial.println("Initializing Accelero/Gyro MPU6050...");
  accelgyro.initialize();
  // verify connection
  // Serial.println("Testing connection...");
  ag_connected = accelgyro.testConnection();
  // Serial.println(ag_connected ? "MPU6050 connection successful" : "MPU6050 connection failed");
  resetAG();

  // createJson();

  //setup pulse counter callbacks
  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterrupt, HandleLeftMotorInterruptA, RISING);
  // Right encoder
  pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_RightEncoderInterrupt, HandleRightMotorInterruptA, RISING);

  //setup motors
  pinMode(M1, OUTPUT);  
  pinMode(M2, OUTPUT);
    
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(E3, OUTPUT);
  pinMode(E4, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

}


void loop()
{

  receiveCommands();

  readCompass();
  readAG();

  sendData();

} 

void receiveCommands()
{
	aJsonObject* item;
	if (serial_stream.available()) {
		item = aJson.parse(&serial_stream);
    } else {
        return;
    }
    
    aJson.print(item, &serial_stream);
    Serial.println(" ");

	switch(getType(item)) {
		case DRIVE_COMMAND:
			handleDriveCommand(item);
			break;
		case MOTOR_COMMAND:
			handleMotorCommand(item);
			break;
		case CONTROL_COMMAND:
			handleControlCommand(item);
			break;
		case DISCONNECT:
			handleDisconnect(item);
			break;
	}
	aJson.deleteItem(item);
}

void handleControlCommand(aJsonObject* json) {
	// is not necessary for now
}

void handleDisconnect(aJsonObject* json) {
	// is sent when the phone disconnects from the robot
	// best to turn off all motors here
}

void handleMotorCommand(aJsonObject* json) {
  Serial.println("1");
	int id = -1, direction = -1, value = -1;
	decodeMotorCommand(json, &id, &direction, &value);
  secdrive(value, id);
  Serial.println("2");
	// TODO: add here the function call to drive the brush motor
	//  id is in case we want to control other mothers, currently a value of 1 is sent
	//  direction, either 0 or 1, where 1 is "forward"
	//  speed, a value between 0 and 255
}

void handleDriveCommand(aJsonObject* json) {
	int leftSpeed = 0, rightSpeed = 0;
	decodeDriveCommand(json, &leftSpeed, &rightSpeed);
	drive(leftSpeed, rightSpeed);
}

void readCompass() {
    // Send a "A" command to the HMC6352
    // This requests the current heading data
    Wire.beginTransmission(slaveAddress);
    Wire.write("A");              // The "Get Data" command
    Wire.endTransmission();
    delay(2);                   // The HMC6352 needs at least a 70us (microsecond) delay after this command.
    // Read the 2 heading bytes, MSB first. The resulting 16bit word is the compass heading in 10th's of a degree
    // For example: a heading of 1345 would be 134.5 degrees
    Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
    int i = 0;
    while(Wire.available() && i < 2)
    { 
      headingData[i] = Wire.read();
      i++;
    }
    headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together

    PushFront(headingValue, headingHistory, MAX_HEADING_HISTORY);

    //calculate new min / max values
    if (headingValue > headingMax) {
      headingMax = headingValue;
    }

    if (headingValue < headingMin) {
      headingMin = headingValue;
    }

  //calculate new rolling average
  long nHeadingAvg = 0L;
  int elements = 0;    
  for (int i = 0; i < MAX_HEADING_HISTORY; i++)
  {
    if (headingHistory[i] == -1.0) continue;
    elements++;
    nHeadingAvg += headingHistory[i];
  }

  headingAvg = (long) nHeadingAvg / elements;

}

void readAG() {

    if (!ag_connected) return;

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&agValue[AX], &agValue[AY], &agValue[AZ], &agValue[GX], &agValue[GY], &agValue[GZ]);

    for (int j = 0; j < 6; ++j)
    {
      // agValue[j] = agValue[j] / (32767 / 2) * 9.81;

      PushFront(agValue[j], agHistory[j], MAX_AG_HISTORY);

      //calculate the new min / max values
      if (agValue[j] > agMax[j]) {
        agMax[j] = agValue[j];
      }

      if (agValue[j] < agMin[j]) {
        agMin[j] = agValue[j];
      }

      //calculate the new rolling average
      int elements = 0;
      long average = 0L;
      for (int i = 0; i < MAX_AG_HISTORY; i++)
      {
        if (agHistory[j][i] == -1.0) continue;
        elements++;
        average += agHistory[j][i];
      }
      agAvg[j] = (long) average /  elements;
    }
}

//encoder interrupts

// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _LeftEncoderBSet = analogRead(c_LeftEncoderPinB) < 512 ? false : true;   // read the input pin
 
  // and adjust counter + if A leads B
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #endif
}
 
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _RightEncoderBSet = analogRead(c_RightEncoderPinB) < 512 ? false : true;   // read the input pin  //digitalReadFast
 
  // and adjust counter + if A leads B
  #ifdef RightEncoderIsReversed
    _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
  #else
    _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
  #endif
}

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

aJsonObject *json, *compass, *accelero, *gyro, *odo, *motor; //why is this global?
void sendData() {
  json = aJson.createObject();

  compass = aJson.createObject();
  aJson.addNumberToObject(compass, "heading", formatCompassValue(headingValue));
  aJson.addItemToObject(json, "compass", compass);

  accelero = aJson.createObject();
  aJson.addNumberToObject(accelero, "x", formatAcceleroValue(agValue[AX]));
  aJson.addNumberToObject(accelero, "y", formatAcceleroValue(agValue[AY]));
  aJson.addNumberToObject(accelero, "z", formatAcceleroValue(agValue[AZ]));
  aJson.addItemToObject(json, "accelero", accelero);

  gyro = aJson.createObject();
  aJson.addNumberToObject(gyro, "x", formatGyroValue(agValue[GX]));
  aJson.addNumberToObject(gyro, "y", formatGyroValue(agValue[GY]));
  aJson.addNumberToObject(gyro, "z", formatGyroValue(agValue[GZ]));
  aJson.addItemToObject(json, "gyro", gyro);

  odo = aJson.createObject();
  aJson.addNumberToObject(odo, "rightencoder", _RightEncoderTicks);
  aJson.addNumberToObject(odo, "leftencoder", _LeftEncoderTicks);
  aJson.addItemToObject(json, "odo", odo);

  motor = aJson.createObject();
  aJson.addNumberToObject(odo, "motor1", CSense1);
  aJson.addNumberToObject(odo, "motor2", CSense2);
  aJson.addNumberToObject(odo, "motor1peak", ReportC1);
  aJson.addNumberToObject(odo, "motor2peak", ReportC2);
  aJson.addItemToObject(json, "motor", motor);

  aJson.print(json, &serial_stream);
  Serial.println();
  aJson.deleteItem(json);
}

// actuator functions

void drive(int leftSpeed, int rightSpeed)
{
        int count = 0;
        int incidentcount = 0;
        
        int motor1pin = E1;
        int motor2pin = E3;

	int left = capSpeed(leftSpeed);  //can we not take these out?
	int right = capSpeed(rightSpeed);

        CSense1 = 0;
        CSense2 = 0;
        ReportC1 = 0;
        ReportC2 = 0;       

        digitalWrite(M1,HIGH);  
        digitalWrite(M2,HIGH);

    Serial.println("in Drive");
      analogWrite(E1, 0);   //PWM Speed Control
      analogWrite(E2, 0);   //PWM Speed Control
      analogWrite(E3, 0);   //PWM Speed Control
      analogWrite(E4, 0);   //PWM Speed Control
    Serial.println("speed set to zero");

	//setDirection(&left, &right);
    if (leftSpeed > 0 && rightSpeed > 0)
    { // forward direction
      motor1pin = E1;
      motor2pin = E3;    
    } 
    else if (leftSpeed < 0 && rightSpeed < 0)
    {  // backward
      motor1pin = E2;
      motor2pin = E4;    
    }
    else if (leftSpeed < 0 && rightSpeed > 0)
    { // turning right
      motor1pin = E2;
      motor2pin = E3;    
    }
    else  // only option left: if (leftSpeed > 0 && rightSpeed < 0)
    { // turning left
      motor1pin = E1;
      motor2pin = E4;    
    }

    while ((incidentcount < MAXINCIDENTCOUNT) && (curleftSpeed != abs(capSpeed(leftSpeed))) && (currightSpeed != abs(capSpeed(rightSpeed)))) {
      CSense1 = analogRead(A0);
      CSense2 = analogRead(A2);
      ReportC1 = (CSense1 > ReportC1 ? CSense1 : ReportC1); // track maximum current sensed
      ReportC2 = (CSense2 > ReportC2 ? CSense2 : ReportC2);
      //Serial.print("CSense1 :");Serial.println(CSense1);
      //Serial.print("CSense2 :");Serial.println(CSense2);
      if ( (CSense1 < CURRENTLIMIT) &&  (CSense2 < CURRENTLIMIT) ) { // check for current
        count++;  // if ok increase speed
      }
      else {
        count = count--; // if not ok decrease
        incidentcount++;
        if (incidentcount > MAXINCIDENTCOUNT) {
          count = 0; // reset speed
        }
      }
      // Serial.print("Count :");Serial.println(count);

      if (count > abs(capSpeed(leftSpeed)))
        left = abs(capSpeed(leftSpeed));
      else
        left = count;

      if (count > abs(capSpeed(rightSpeed)))
        right = abs(capSpeed(rightSpeed));
      else
        right = count;
        
      analogWrite(motor1pin, left);   //PWM Speed Control
      analogWrite(motor2pin, right);   //PWM Speed Control

      curleftSpeed = left;
      currightSpeed = right;
      //Serial.print("curleftSpeed :");Serial.println(curleftSpeed);
      //Serial.print("currightSpeed :");Serial.println(currightSpeed);
      delay(10);
    }

  Serial.println(left);
  Serial.println(right);
  Serial.println(ReportC1);
  Serial.println(ReportC2);

}

//driver function for one motor
void secdrive(int value,int motor)
{
    int count = 0;

    int motorPin;
    int motorPin1;
    int motorPin2;
    int enablePin;

    int curSpeed = 0;

    if (motor == 1) {
      motorPin1 = E5;
      motorPin2 = E6;
      enablePin = M3;
      curSpeed = curSpeedMotor1;
    } else if (motor == 2) {
      motorPin1 = E7;
      motorPin2 = E8;
      enablePin = M4;
      curSpeed = curSpeedMotor2;
    } else if (motor == 3) {
      motorPin1 = E9;
      motorPin2 = E10;
      enablePin = M5;
      curSpeed = curSpeedMotor3;
    }
    
    digitalWrite(enablePin,HIGH); // enable selected motor 

    //Serial.println("in Drive");

    //analogWrite(motorPin1, 0);   //PWM Speed Control set to zero
    //analogWrite(motorPin2, 0);   //PWM Speed Control

    //Serial.println("speed set to zero");

	//setDirection(&left, &right);
    if (curSpeed > 0)
    { // forward direction
      motorPin = motorPin1;
    } 
    else
    {  // backward
      motorPin = motorPin2;
    }

    while ((curSpeed != value)) {

      if (curSpeed < value)
        curSpeed++;  //  increase speed
      else
        curSpeed--;  //  decrease spedd
        // Serial.print("Count :");Serial.println(count);
      if (curSpeed == 0) {
        if (motorPin == motorPin1)
          motorPin = motorPin2;
        else
          motorPin = motorPin1;
      }
      
      // curSpeed = count;
        
      analogWrite(motorPin, curSpeed);   //PWM Speed Control
      delay(5);
    }

    // report motor speed to global
    if (motor == 1) {
      curSpeedMotor1 = curSpeed;
    } else if (motor == 2) {
      curSpeedMotor2 = curSpeed;
    } else if (motor == 3) {
      curSpeedMotor3 = curSpeed;
    }
}

int capSpeed(int value)
{
	return max(min(value,249),-249);   
}


///helper functions

void PushFront(int val, int* valueList, int len)
{
  for (int i = len-2; i >= 0; i--) 
  {
    valueList[i+1] = valueList[i];
  }
  valueList[0] = val;
}

float formatAcceleroValue(int value) {
  return float(value / 32767.0 * ACCELEROMETER_RANGE);
}

float formatGyroValue(int value) {
  return float(value / 32767.0 * GYROSCOPE_RANGE);
}

float formatCompassValue(int value) {
  return float(value / 10.0);
}

void resetAG() {
  for (int i = 0; i < 6; ++i)
  {
    agValue[i] = 0;
    for (int j = 0; j < MAX_AG_HISTORY; ++j)
    {
      agHistory[i][j] = -1;
    }
    agAvg[i] = -1L;
    agMax[i] = -1000000;
    agMin[i] = 1000000;
  }
}

void resetCompass() {
  for (int i = 0; i < MAX_HEADING_HISTORY; i++) {
    headingHistory[i] = -1;
  }

  headingAvg = -1L;
  headingMax = -1000000;
  headingMin = 1000000;
}

int getType(aJsonObject* json) {
	aJsonObject* header;
	aJsonObject* type;
	header = aJson.getObjectItem(json, "header");
	type = aJson.getObjectItem(header, "type");
	return type->valueint;
}

void decodeMotorCommand(aJsonObject* json, int* motor_id, int* direction, int* speed) {
	aJsonObject* data = aJson.getObjectItem(json, "data");

	aJsonObject* motor_id_j = aJson.getObjectItem(data, "motor_id");
	*motor_id = motor_id_j->valueint;

	aJsonObject* direction_j = aJson.getObjectItem(data, "direction");
	*direction = direction_j->valueint;

	aJsonObject* speed_j = aJson.getObjectItem(data, "speed");
	*speed = speed_j->valueint;
}

void decodeDriveCommand(aJsonObject* json, int* left, int* right) {
	aJsonObject* data = aJson.getObjectItem(json, "data");

	aJsonObject* left_j = aJson.getObjectItem(data, "left");
	*left = left_j->valueint;

	aJsonObject* right_j = aJson.getObjectItem(data, "right");
	*right = right_j->valueint;
}

