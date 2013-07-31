#include "Arduino.h"
#include "Wire.h"

//#include "digitalWriteFast.h"  //no need as we use analogRead (unfortunately..)

#include "robo40control.h"
#include "TimerOne.h"
#include "log.h"

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

aJsonStream serial_stream(&Serial);

int HMC6352Address = 0x42;

// This is calculated in the setup() function
int slaveAddress;
byte headingData[2];
int headingValue;
int headingHistory[MAX_HEADING_HISTORY];
long headingAvg = -1L;
int headingMax = -1;
int headingMin = 10000;

bool ag_connected = false;
int agValue[6];
int agHistory[6][MAX_AG_HISTORY];
long agAvg[6];
int agMax[6];
int agMin[6];

volatile int _LeftEncoderTicks = 0;
volatile int _RightEncoderTicks = 0;

int currentSenseLeft = 0;
int currentSenseRight = 0;
int reportCSLeft = 0;
int reportCSRight = 0;

bool motor_access[4] = {false, false, false, false};
int curSpeedMotor[4] = {0, 0, 0, 0};
int currentSenseMotor[4] = {0, 0, 0, 0};
int reportCSMotor[4] = {0, 0, 0, 0};

int notagain = 0;

bool drive_access = false;
int curLeftSpeed = 0;
int curRightSpeed = 0;
int lastDirectionLeft = 0;
int lastDirectionRight = 0;

int val = 0;
int id = 1;
int speed = 0;
int delay_time = 0;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

// --------------------------------------------------------------------
void setup() {

    //first setup pins, to minimize time that the vacuum pump is on
	for (int i = 0; i < 4; ++i) {
		pinMode(PWM_MOTORS[i], OUTPUT);
		setMotor(i, 0);
	}

	//setup pulse counter callbacks
	// Quadrature encoders
	// Left encoder
	pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
	pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
	attachInterrupt(c_LeftEncoderInterrupt, HandleLeftMotorInterruptA, RISING);
	// Right encoder
	pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
	pinMode(c_RightEncoderPinB, INPUT);      // sets pin B as input
	attachInterrupt(c_RightEncoderInterrupt, HandleRightMotorInterruptA, RISING);

	pinMode(DIRECTION_LEFT_FW, OUTPUT);
	pinMode(DIRECTION_LEFT_BW, OUTPUT);
	pinMode(DIRECTION_RIGHT_FW, OUTPUT);
	pinMode(DIRECTION_RIGHT_BW, OUTPUT);

	pinMode(DIRECTION_A, OUTPUT);
	pinMode(DIRECTION_B, OUTPUT);
	pinMode(DIRECTION_C, OUTPUT);
	pinMode(DIRECTION_D, OUTPUT);

	pinMode(PWM_LEFT, OUTPUT);
	pinMode(PWM_RIGHT, OUTPUT);

	pinMode(CURRENT_SENSE_LEFT, INPUT);
	pinMode(CURRENT_SENSE_RIGHT, INPUT);

	pinMode(CURRENT_SENSE_A, INPUT);
	pinMode(CURRENT_SENSE_B, INPUT);
	pinMode(CURRENT_SENSE_C, INPUT);
	pinMode(CURRENT_SENSE_D, INPUT);

	pinMode(FLASH_LIGHT, OUTPUT);

    pinMode(BUMPER1, INPUT);
    pinMode(BUMPER2, INPUT);

	//connect to computer, uses pin 0 and 1
	Serial.begin(115200);
	initLogging(&Serial);

    // timer for current sensing
    // TODO: put this down to 100 ms or so, its now on 1500 ms for easier debugging
	Timer1.initialize(1500000); // initialize to 1.5s
	Timer1.attachInterrupt(timerCallback); // attach callback function

    // timer for pump; TODO: see how to do this properly, also use a different timer of course
	//Timer1.initialize(1000);
	//Timer1.attachInterrupt(timerCB);
	
	// connection to compass, uses pins 20 and 21
	Wire.begin();
	
	//connect to bluetooth, uses pin 16 and 17
	// Serial2.begin(115200);
	//  Serial2.flush();    
	//  Serial2.setTimeout(10);

	//setup compass
	// Shift the device's documented slave address (0x42) 1 bit right
	// This compensates for how the TWI library only wants the
	// 7 most significant bits (with the high bit padded with 0)
	slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
	resetCompass();

	//setup accelero/gyro
	// initialize device
	accelgyro.initialize();
	// verify connection
	ag_connected = accelgyro.testConnection();
	LOGd(3, ag_connected ? "MPU6050 connection successful" : "MPU6050 connection failed");
	resetAG();

	// createJson();

	LOGd(1, "ready");
	print();
}

int oncount = 5;
int interval = 100;

int count = 0;

boolean on = false;
boolean run = false;
boolean lastOn = false;
boolean log_sense = true;

void print() {
	LOGd(3, "motor: %d, val: %d, speed: %d, delay: %d, interval: %d, on interval: %d",
		id, val, speed, delay_time, interval, oncount);
}

void reset() {
	oncount = 5;
	interval = 100;
	count = 0;
	run = false;
	on = false;
	lastOn = false;
	log_sense = true;
}

void timerCB() {
	if (run) {
		count++;
		if (count <= oncount) {
			on = true;
		} else {
			on = false;
			if (count == interval) {
				count = 0;
				log_sense = true;
			}
		}

		if (lastOn != on) {
			LOGi(1, "on %d at %d", on, millis());
			digitalWrite(PWM_MOTORS[1], on ? HIGH : LOW);
		}
		lastOn = on;

		if (on && log_sense) {
			senseMotor(1);

			LOGd(3, "sense %d", 
				currentSenseMotor[1]);

			log_sense = false;
		} else {
			log_sense = true;
		}
	} else {
		digitalWrite(PWM_MOTORS[1], LOW);
	}
}

void timerCallback() {
    // check bumper
    if (digitalRead(BUMPER1))
    {
         LOGd(1, "bumper1 true!");
    }
    else 
    {
        LOGd(1, "bumper1 false!"); 
    }

    if (digitalRead(BUMPER2))
    {
         LOGd(1, "bumper2 true!");
    }
    else 
    {
        LOGd(1, "bumper2 false!"); 
    }

	// check motors
	for (int i = 0; i < 4; ++i) {

		if (!(motor_access[i]) && (curSpeedMotor[i] != 0)) {
			senseMotor(i);

			LOGd(3, "motor %d, sense %d", 
				i+1, currentSenseMotor[i]);
					// Serial.print("motor ");
					// Serial.print(i+1);
					// Serial.print(", sense ");
					// Serial.println(currentSenseMotor[i]);

					// if current is over the limit, stop motor
			if (currentSenseMotor[i] > CURRENT_LIMIT_MOTORS[i]) {

				int sense;
				int count;

							// check if it was only an outlier
				for (int j = 0; j < OUTLIER_CHECKS; ++j) {
					sense = analogRead(CURRENT_SENSE_MOTORS[i]);
					if (sense > CURRENT_LIMIT_MOTORS[i]) {
						count++;
					}
					delay(5);
				}

				if (count > OUTLIER_THRESHOLD) {
					setMotor(i, 0);
									// digitalWrite(PWM_MOTORS[i], 0);

					LOGd(0, "INITIATING EMERGENCY SHUTDOWN FOR MOTOR %d",
						i+1);
									// Serial.print("INITIATING EMERGENCY SHUTDOWN FOR MOTOR ");
									// Serial.println(i+1);
				}

			}
		}
	}

	// check left / right wheel (only if speed is set)
	if (!drive_access && ((curLeftSpeed != 0) || (curRightSpeed != 0))) {
		senseLeftRight();

		LOGd(3, "senseLeft %d, senseRight %d",
			currentSenseLeft, currentSenseRight);
			// Serial.print("senseLeft ");
			// Serial.print(currentSenseLeft);
			// Serial.print(", senseRight ");
			// Serial.println(currentSenseRight);

			// if one of the currents is over the limit, stop both motors
		if ((currentSenseLeft > CURRENT_LIMIT_DRIVE) || (currentSenseRight > CURRENT_LIMIT_DRIVE)) {

			int senseLeft, senseRight;
			int countLeft, countRight;

					// check if it was only an outlier
			for (int j = 0; j < OUTLIER_CHECKS; ++j) {
				senseLeft = analogRead(CURRENT_SENSE_LEFT);
				senseRight = analogRead(CURRENT_SENSE_RIGHT);

				if (senseLeft > CURRENT_LIMIT_DRIVE) {
					countLeft++;
				}
				if (senseRight > CURRENT_LIMIT_DRIVE) {
					countRight++;
				}
				delay(5);
			}

			if ((countLeft > OUTLIER_THRESHOLD) || (countRight > OUTLIER_THRESHOLD)) {
				curLeftSpeed = 0;
				curRightSpeed = 0;

				digitalWrite(PWM_LEFT, 0);
				digitalWrite(PWM_RIGHT, 0);

				LOGd(0, "INITIATING EMERGENCY SHUTDOWN FOR DRIVES");
							// Serial.println("INITIATING EMERGENCY SHUTDOWN FOR DRIVES");
			}
		}
	}

}

void senseLeftRight() {
	currentSenseLeft = analogRead(CURRENT_SENSE_LEFT);
	currentSenseRight = analogRead(CURRENT_SENSE_RIGHT);
	reportCSLeft = (currentSenseLeft > reportCSLeft ? currentSenseLeft : reportCSLeft); // track maximum current sensed
	reportCSRight = (currentSenseRight > reportCSRight ? currentSenseRight : reportCSRight);  
}

void senseMotor(int motor) {
	currentSenseMotor[motor] = analogRead(CURRENT_SENSE_MOTORS[motor]);
	reportCSMotor[motor] = max(reportCSMotor[motor], currentSenseMotor[motor]);
}

void loop()
{

	receiveCommands();

	readCompass();
	readAG();

	// LOGi(1, "");

	// delay(1000);

} 

void handleInput(int incoming) {
	switch(incoming) {
		case 'g': sendData(); LOGi(1, ""); break;
		case 'h': run = !run; LOGi(1, "run %d", run); break;
		case 'r': reset(); break;
		case '1':
    	oncount--;
		if (oncount < 1) {
			oncount = 1;
		}
		LOGi(1, "on interval: %d", oncount);
					// for (int i = 0; i < 100; ++i) {
					//   secdrive(i, 4);
					//   delay(1000);
					// }
					// val = 130;1
					// secdrive(130, id);
		// drive(speed, speed);
		break;
		case '2':
		oncount++;
		LOGi(1, "on interval: %d", oncount);
					// val = 70;
					// secdrive(70, id);
		// drive(-speed, -speed);
		break;
		case '3':
		interval--;
		if (interval < 1) {
			interval = 1;
		}
		LOGi(1, "interval: %d", interval);
					// val = 0;
					// secdrive(0, id);
		// drive(0, 0);
		break;
		case '4':
		interval++;
		LOGi(1, "interval: %d", interval);		
		// drive(-speed, speed); // right
		// val = 255;
		// secdrive(255, id);
		// digitalWrite(DIRECTION_D, HIGH);
		break;
		case 'q':
		drive(speed, -speed);
		// val = 255;
		// secdrive(255, id);
		// digitalWrite(DIRECTION_D, HIGH);
		break;
		case 'f': LOGd(2, "flashing"); flashLight(speed); break;
		case '5': stop(id); break;
		case '6': secdrive(val, id); break;
		case '7': secdrive(0, id); break;
		case '8': secdrive(200, id); break;
		case '9': secdrive(50, id); break;
		case '0': secdrive(-50, id); break;
		case 'z': speed += 10; LOGd(2, "speed: %d", speed);	break;
		case 'x': speed -= 10; LOGd(2, "speed: %d", speed);	break;
		case 'c': delay_time += 10; LOGd(2, "delay: %d", delay_time); break;
		case 'v': delay_time -= 10; LOGd(2, "delay: %d", delay_time); break;   
		case 'u': val += 10; LOGd(2, "val: %d", val); break;
		case 'd': val -= 10; LOGd(2, "val: %d", val); break;
		case 'i': val += 1; LOGd(2, "val: %d", val); break;
		case 'k': val -= 1;	LOGd(2, "val: %d", val); break;
		case 'w': id += 1; LOGd(2, "motor: %d", id); break;
		case 's': id -= 1; LOGd(2, "motor: %d", id); break;
		case 'p': print(); break;
        case 't':
        	digitalWrite(DIRECTION_RIGHT_FW, HIGH);
			digitalWrite(DIRECTION_RIGHT_BW, LOW);
		    analogWrite(PWM_RIGHT, 200);   //PWM Speed Control
            break;
		default: LOGd(1, "incoming: %c (%d)", incoming, incoming); break;
	}
}

void handleInput() {

}

void receiveCommands()
{

	// aJsonObject* item;
	// if (serial_stream.available()) {
	// 	item = aJson.parse(&serial_stream);

	// 	if (item == NULL) {
	// 		LOGd(0, "not a json object!");
	// 		// handleInput();
	// 		serial_stream.flush();
	// 		return;
	// 	}
	// } else {
	// 	return;
	// }

	// LOGd(3, "cmdReceived");

	// // aJson.print(item, &serial_stream);
	// // Serial.println(" ");

	// switch(getType(item)) {
	// 	case DRIVE_COMMAND:
	// 		handleDriveCommand(item);
	// 		break;
	// 	case MOTOR_COMMAND:
	// 		handleMotorCommand(item);
	// 		break;
	// 	case CONTROL_COMMAND:
	// 		handleControlCommand(item);
	// 		break;
	// 	case DISCONNECT:
	// 		handleDisconnect(item);
	// 		break;
	// 	case SENSOR_REQUEST:
	// 		handleSensorRequest(item);
	// 	default:
	// }
	// aJson.deleteItem(item);

	if (Serial.available()) {
	 int incoming = Serial.read();
	 handleInput(incoming);
	}

	// if (Serial2.available()) {
	//  int incoming = Serial2.read();
	//  LOGi(1, "serial2: %d", incoming);
	//  handleInput(incoming);
	// }
}

void handleControlCommand(aJsonObject* json) {
// is not necessary for now
}

void handleDisconnect(aJsonObject* json) {
// is sent when the phone disconnects from the robot, best to turn off all motors here
	drive(0, 0);
    for (int i = 0; i < 4; ++i) {
		setMotor(i, 0);
	}
    flashLight(0);
}

void handleSensorRequest(aJsonObject* json) {
	LOGd(3, "handleSensorRequest");
	sendData();
}

void handleMotorCommand(aJsonObject* json) {
	LOGd(3, "handleMotorCommand");
//Serial.println("1");
	int id = -1, direction = -1, value = -1;
	decodeMotorCommand(json, &id, &direction, &value);
	secdrive(value, id);
//Serial.println("2");
// TODO: add here the function call to drive the brush motor
//  id is in case we want to control other mothers, currently a value of 1 is sent
//  direction, either 0 or 1, where 1 is "forward"
//  speed, a value between 0 and 255
}

void handleDriveCommand(aJsonObject* json) {
	LOGd(3, "hanldeDriveCommand");
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

// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA()
{
    if (curLeftSpeed > 0) 
    {
        lastDirectionLeft = 1;
    } 
    else if (curLeftSpeed < 0) 
    {
        lastDirectionLeft = -1;
    }

    if (lastDirectionLeft == 0) return; //early exit because we dont know in which direction we go

    #ifdef LeftEncoderIsReversed
        _LeftEncoderTicks -= lastDirectionLeft;
	#else
	    _LeftEncoderTicks += lastDirectionLeft;
	#endif

    //for working 2channel encoder
	// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
	//_LeftEncoderBSet = analogRead(c_LeftEncoderPinB) < 512 ? false : true;   // read the input pin

	// and adjust counter + if A leads B
	//#ifdef LeftEncoderIsReversed
	//  _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
	//#else
	//  _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
	//#endif

	LOGi(1, "interruptA %d", _LeftEncoderTicks);
}
	
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{

    if (curRightSpeed > 0) 
    {
        lastDirectionRight = 1;
    } 
    else if (curRightSpeed < 0) 
    {
        lastDirectionRight = -1;
    }

    if (lastDirectionRight == 0) return; //early exit because we dont know in which direction we go

    #ifdef RightEncoderIsReversed
        _RightEncoderTicks -= lastDirectionRight;
	#else
	    _RightEncoderTicks += lastDirectionRight;
	#endif

    //for working 2channel encoder
	// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
    //	_RightEncoderBSet = analogRead(c_RightEncoderPinB) < 512 ? false : true;   // read the input pin  //digitalReadFast

	// and adjust counter + if A leads B
	//#ifdef RightEncoderIsReversed
	//  _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
	//#else
	//  _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
	//#endif

	LOGi(1, "interruptB %d", _RightEncoderTicks);
}

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

int msgCounter;
void sendData() {

	aJsonObject *json, *header, *data, *group, *sub, *item;

	json = aJson.createObject();

	header = aJson.createObject();
	aJson.addNumberToObject(header, "id", HEADER);
	aJson.addNumberToObject(header, "timestamp", msgCounter++);
	aJson.addNumberToObject(header, "type", SENSOR_DATA);
	aJson.addItemToObject(json, "header", header);

	data = aJson.createObject();

	// COMPASS
	group = aJson.createObject();
	aJson.addNumberToObject(group, "heading", formatCompassValue(headingValue));
	aJson.addItemToObject(data, "compass", group);

	// ACCELERO
	group = aJson.createObject();
	aJson.addNumberToObject(group, "x", formatAcceleroValue(agValue[AX]));
	aJson.addNumberToObject(group, "y", formatAcceleroValue(agValue[AY]));
	aJson.addNumberToObject(group, "z", formatAcceleroValue(agValue[AZ]));
	aJson.addItemToObject(data, "accelero", group);

	// GYRO
	group = aJson.createObject();
	aJson.addNumberToObject(group, "x", formatGyroValue(agValue[GX]));
	aJson.addNumberToObject(group, "y", formatGyroValue(agValue[GY]));
	aJson.addNumberToObject(group, "z", formatGyroValue(agValue[GZ]));
	aJson.addItemToObject(data, "gyro", group);

	// ENCODER
	group = aJson.createObject();
	aJson.addNumberToObject(group, "rightEncoder", _RightEncoderTicks);
	aJson.addNumberToObject(group, "leftEncoder", _LeftEncoderTicks);
	aJson.addItemToObject(data, "odom", group);

	// WHEELS
	group = aJson.createObject();

	// .. LEFT
	sub = aJson.createObject();
	item = aJson.createObject();
	aJson.addNumberToObject(item, "present", currentSenseLeft);
	aJson.addNumberToObject(item, "peak", reportCSLeft);
	aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curLeftSpeed);
	aJson.addItemToObject(group, "left", sub);

	// .. RIGHT
	sub = aJson.createObject();
	item = aJson.createObject();
	aJson.addNumberToObject(item, "present", currentSenseRight);
	aJson.addNumberToObject(item, "peak", reportCSRight);
	aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curRightSpeed);
	aJson.addItemToObject(group, "right", sub);
	
	aJson.addItemToObject(data, "wheels", group);

	// MOTORS
	group = aJson.createObject();

	// .. ELEVATOR
	sub = aJson.createObject();
	item = aJson.createObject();
	aJson.addNumberToObject(item, "present", currentSenseMotor[ELEVATOR]);
	aJson.addNumberToObject(item, "peak", reportCSMotor[ELEVATOR]);
	aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curSpeedMotor[ELEVATOR]);
	aJson.addItemToObject(group, "elevator", sub);

	// .. PUMP
	sub = aJson.createObject();
	item = aJson.createObject();
	aJson.addNumberToObject(item, "present", currentSenseMotor[PUMP]);
	aJson.addNumberToObject(item, "peak", reportCSMotor[PUMP]);
	aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curSpeedMotor[PUMP]);
	aJson.addItemToObject(group, "pump", sub);

	// .. VACUUM
	sub = aJson.createObject();
	item = aJson.createObject();
	aJson.addNumberToObject(item, "present", currentSenseMotor[VACUUM]);
	aJson.addNumberToObject(item, "peak", reportCSMotor[VACUUM]);
	aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curSpeedMotor[VACUUM]);
	aJson.addItemToObject(group, "vacuum", sub);

	// .. BRUSH
	sub = aJson.createObject();
	item = aJson.createObject();
	aJson.addNumberToObject(item, "present", currentSenseMotor[BRUSH]);
	aJson.addNumberToObject(item, "peak", reportCSMotor[BRUSH]);
	aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curSpeedMotor[BRUSH]);
	aJson.addItemToObject(group, "brush", sub);

	aJson.addItemToObject(data, "motors", group);   

	aJson.addItemToObject(json, "data", data);

	aJson.print(json, &serial_stream);
	Serial.println("");
	aJson.deleteItem(json);
}

void stop(int motor) {

	if ((motor < 1) || (motor > 4))
		return;

	int motor_id = motor -1;

	// curSpeedMotor[motor_id] = 0;
	// analogWrite(PWM_MOTORS[motor_id], 0);   //PWM Speed Control
	setMotor(motor_id, 0);
}

void flashLight(int speed) {
	analogWrite(FLASH_LIGHT, speed);
}

// actuator functions

// actuator functions
void drive(int leftSpeed, int rightSpeed)
{
	int count = 0;
	int incidentcount = 0;

	// int curLeftSpeed = 0;
	// int curRightSpeed = 

	leftSpeed = capSpeed(leftSpeed);
	rightSpeed = capSpeed(rightSpeed);

	LOGd(2, "drive(%d, %d)", 
		leftSpeed, rightSpeed);
	
	drive_access = true;

	while ((incidentcount < MAXINCIDENTCOUNT) && 
		   ((curLeftSpeed != capSpeed(leftSpeed)) || 
		   	(curRightSpeed != capSpeed(rightSpeed)))) {
		senseLeftRight();

		//Serial.print("currentSenseLeft :");Serial.println(currentSenseLeft);
		//Serial.print("currentSenseRight :");Serial.println(currentSenseRight);
		if ( (currentSenseLeft < CURRENT_LIMIT_DRIVE) ) { // check for current
			if (leftSpeed < curLeftSpeed) {
				curLeftSpeed--;
			} else if (leftSpeed > curLeftSpeed) {
				curLeftSpeed++;
			}
		}
		else {
			if (leftSpeed < curLeftSpeed) {
				curLeftSpeed++;
			} else if (leftSpeed > curLeftSpeed) {
				curLeftSpeed--;
			}
			incidentcount++;
			if (incidentcount > MAXINCIDENTCOUNT) {
				curRightSpeed = 0; // reset speed
				curLeftSpeed = 0;
			}
		}

		if ( (currentSenseRight < CURRENT_LIMIT_DRIVE) ) { // check for current
			if (rightSpeed < curRightSpeed) {
				curRightSpeed--;
			} else if (rightSpeed > curRightSpeed) {
				curRightSpeed++;
			}
		}
		else {
			if (rightSpeed < curRightSpeed) {
				curRightSpeed++;
			} else if (rightSpeed > curRightSpeed) {
				curRightSpeed--;
			}
			incidentcount++;
			if (incidentcount > MAXINCIDENTCOUNT) {
				curRightSpeed = 0; // reset speed
				curLeftSpeed = 0;
			}
		}
		// Serial.print("Count :");Serial.println(count);
		
		curLeftSpeed = capSpeed(curLeftSpeed);
		curRightSpeed = capSpeed(curRightSpeed);

		if (curLeftSpeed > 0) {
			digitalWrite(DIRECTION_LEFT_FW, HIGH);
			digitalWrite(DIRECTION_LEFT_BW, LOW);
		} else if (curLeftSpeed < 0) {
			digitalWrite(DIRECTION_LEFT_FW, LOW);
			digitalWrite(DIRECTION_LEFT_BW, HIGH);
		} else {
			digitalWrite(DIRECTION_LEFT_FW, LOW);
			digitalWrite(DIRECTION_LEFT_BW, LOW);
		}

		if (curRightSpeed > 0) {
			digitalWrite(DIRECTION_RIGHT_FW, HIGH);
			digitalWrite(DIRECTION_RIGHT_BW, LOW);
		} else if (curRightSpeed < 0) {
			digitalWrite(DIRECTION_RIGHT_FW, LOW);
			digitalWrite(DIRECTION_RIGHT_BW, HIGH);
		} else {
			digitalWrite(DIRECTION_RIGHT_FW, LOW);
			digitalWrite(DIRECTION_RIGHT_BW, LOW);
		}

		analogWrite(PWM_LEFT, abs(curLeftSpeed));   //PWM Speed Control
		analogWrite(PWM_RIGHT, abs(curRightSpeed));   //PWM Speed Control

		LOGd(3, "left: %d, right: %d, senseLeft: %d, senseRight: %d",
			curLeftSpeed, curRightSpeed, currentSenseLeft, currentSenseRight);

		delay(delay_time);

	}

	drive_access = false;

}

void setMotor(int id, int value) {
	curSpeedMotor[id] = value;
	if (MOTOR_INVERTED[id]) {
		value = 255 - abs(value);   
	}
	analogWrite(PWM_MOTORS[id], abs(value));

}

//driver function for one motor
void secdrive(int value,int motor)
{
	LOGd(2, "secdrive(%d, %d)",
		motor, value);

	int count = 0;
	int incidentcount = 0;

	int motorPin;
	int motorPin1;
	int motorPin2;
	int enablePin;

	int currentSensePin;
	int pwmPin;
	int directionPin;

	int *curSpeed = NULL;
	int *sense = NULL;
	int *report = NULL;

	if ((motor < 1) || (motor > 4))
		return;

	int motor_id = motor -1;

	pwmPin = PWM_MOTORS[motor_id];
	directionPin = DIRECTION_MOTORS[motor_id];
	currentSensePin = CURRENT_SENSE_MOTORS[motor_id];

	curSpeed = &(curSpeedMotor[motor_id]);
	sense = &(currentSenseMotor[motor_id]);
	report = &(reportCSMotor[motor_id]);

	motor_access[motor_id] = true;

	while ((incidentcount < MAXINCIDENTCOUNT) && (*curSpeed != capSpeed(value))) {
		senseMotor(motor_id);

		if ( (*sense < CURRENT_LIMIT_MOTORS[motor_id]) ) { // check for current
			if (value < *curSpeed) {
				(*curSpeed)--;
			} else {
				(*curSpeed)++;
			}
		}
		else {
			if (value < *curSpeed) {
				(*curSpeed)++;
			} else {
				(*curSpeed)--;
			}
			incidentcount++;
			if (incidentcount > MAXINCIDENTCOUNT) {
						// *curSpeed = 0; // reset speed
						// analogWrite(pwmPin, abs(*curSpeed));
				setMotor(motor_id, *curSpeed);
				return;
			}
		}
		
		*curSpeed = capSpeed(*curSpeed);

		if (*curSpeed > 0) {
			digitalWrite(directionPin, HIGH);
		} else {
			digitalWrite(directionPin, LOW);
		}

		// analogWrite(pwmPin, abs(*curSpeed));   //PWM Speed Control
		setMotor(motor_id, *curSpeed);

		LOGd(3, "pin: %d, curSpeed: %d, sense: %d, report: %d",
			pwmPin, *curSpeed, *sense, *report);

		delay(delay_time);
	}

	motor_access[motor_id] = false;
	LOGd(2, "done");

}

int capSpeed(int value)
{
// return max(min(value,249),-249);
	return max(min(value,255),-255);   
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
	if (header == NULL) {
		LOGd(1, "wrong json message");
		return -1;
	}
	type = aJson.getObjectItem(header, "type");
	if (type == NULL) {
		LOGd(1, "wrong json message");
		return -1;
	}
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

