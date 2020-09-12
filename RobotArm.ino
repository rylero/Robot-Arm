/*
 * This is the sketch responsible for driving the 6 servo motors of the robot
 * arm as well as receiving roll,pitch and yaw data from the controlling glove.
 * The bluetooth device associated with the arm is in slave mode and the glove is
 * the master.
 * These are the bluetooth settings of the HC-05 bluetooth module connected to the
 * robot arm base:
 * +NAME:WRIST-GLOVE-SLAVE
 * +ROLE:0
 * +ADDR:18:91:d83b73
 * +UART:38400,0,0
 * +VERSION:2.0-20100601
 *
 * https://www.teachmemicro.com/hc-05-bluetooth-command-list/
 */
#include <SoftwareSerial.h>     
#include <Servo.h>

String readString = "";
char c;
SoftwareSerial mySerial(10, 11); // RX, TX 
String IMUparams[3];  // IMU parameters parsed from received string (roll,pitch,yaw)
int roll = 0;
int pitch = 0;
int yaw = 0;

/* Servos */
Servo shoulderYawServo;
int shoulderYaw;
Servo shoulderPitchServo;
int shoulderPitch;
Servo elbowServo;
int elbow;
Servo wristPitchServo;
int wristPitch;
Servo wristRollServo;
int wristRoll;
Servo gripperServo;
int gripper;

// Set the initial position of the robot arm
void initPos() {
    shoulderYaw = 90;
    shoulderPitch = 90;
    elbow = 90;
    wristPitch = 90;
    wristRoll = 90;
    gripper = 90;
}


/* update initial servo position */
void updateWithStates() {
  shoulderYawServo.write(shoulderYaw);
  shoulderPitchServo.write(shoulderPitch);
  elbowServo.write(elbow);
  wristPitchServo.write(wristPitch);
  wristRollServo.write(wristRoll);
  gripperServo.write(gripper);
}



/*
 * Moves the specified servo from the current angle to new angle at a certain speed
 */
void moveServo(Servo whichServo, int currAngle, int newAngle, int speed) {
  int i;
    if (currAngle >= newAngle) {
      for (i = currAngle; i > newAngle; i--) {
        whichServo.write(i);
        delay(speed/(currAngle - newAngle));
      }
    } else {
      for (i = currAngle; i < newAngle; i++) {
        whichServo.write(i);
        delay(speed/(newAngle - currAngle));
      }
    }
}

/*
 * Takes received string from bluetooth reception and
 * parses the 3 seperate roll, pitch and yaw data into
 * IMUparams string array.
 */
int parseBlueTooth(String inString) {
  int i;
  int startIndex =0;
  int fromIndex = 0;
  int paramcnt = 0;

  // loop through and parse comma seperated values
  while (((i = inString.indexOf(',',fromIndex)) != -1) || (paramcnt < 3)) {
    IMUparams[paramcnt] = inString.substring(startIndex,i);
    paramcnt++;
    startIndex = fromIndex = i+1; /* Start looking at character after index of last comma */
    // If we've read 2 params, then last one is to the end of the string
    if (paramcnt == 2) {
      IMUparams[paramcnt] = inString.substring(startIndex);
      paramcnt++;  // Will force us to break out of the while loop
    }
  }
  return paramcnt;
}
void setup()
{
  Serial.begin(9600);  // Initial for serial monitor (if connected)
  mySerial.begin(38400); // Default communication rate of the Bluetooth module (this is configured on the bluetooth device)

  // Attach all 6 robot servos
  shoulderYawServo.attach(4);
  shoulderPitchServo.attach(5);
  elbowServo.attach(6);
  wristPitchServo.attach(7);
  wristRollServo.attach(8);
  gripperServo.attach(9);

  // Initialize the positions of all the robot servos
  initPos();
  updateWithStates();
  Serial.println("initialization complete");
  Serial.println("Testing servos");
  //testServos();
}

void loop() {
  int rc;

  if(mySerial.available() > 0){ // Checks whether data is comming from the serial port
    delay(3);
    c = mySerial.read(); // Reads the data from the serial port
    readString += c;
  }

  /* Wait for newline */
  if (c == 0xA) {

    rc = parseBlueTooth(readString);

    // Make sure we correctly parse all 3 values
    if (rc == 3) {
      Serial.println("Successfully parsed values");
      Serial.println(IMUparams[0]);
      Serial.println(IMUparams[1]);
      Serial.println(IMUparams[2]);

      roll  = map(IMUparams[0].toInt(), -90, 90, 0, 180);
      pitch = map(IMUparams[1].toInt(), -90, 90, 0, 180);
      yaw   = map(IMUparams[2].toInt(), 0, 360, 0, 180);
      Serial.println(roll);
      Serial.println(pitch);
      Serial.println(yaw);
      Serial.println("");

      /* XXX Play with last parameter to see how impacts responsivenes */
      moveServo(wristRollServo, wristRoll, roll, 250);
      moveServo(wristPitchServo, wristPitch, pitch, 250);
      //moveServo(shoulderYawServo, shoulderYaw, yaw, 250);

      
      wristRoll = roll;
      wristPitch = pitch;
      shoulderYaw = yaw;
    } else {
      Serial.print("Unable to parse received string ");
      Serial.println(rc);
    }

    // Reset to start parsing next line
    c = 0;
    readString = "";
  }
}


/* test each servos entire range */
void testServos() {
  
  int savePosition;
   delay(1000); 
  /* Start with gripper */
  savePosition = gripper;
  moveServo(gripperServo, gripper, 0, 1000);
  gripper = 0;
  moveServo(gripperServo, gripper, 180, 1000);
  gripper = 180;
  moveServo(gripperServo, gripper, savePosition, 1000);  
  gripper = savePosition;

  delay(1000);
  /* wristRoll */
  savePosition = wristRoll;
  moveServo(wristRollServo, wristRoll, 0, 1000);
  wristRoll = 0;
  moveServo(wristRollServo, wristRoll, 180, 1000);
  wristRoll = 180;
  moveServo(wristRollServo, wristRoll, savePosition, 1000); 
  wristRoll = savePosition; 

  delay(1000);
  /* wristPitch */
  savePosition = wristPitch;
  moveServo(wristPitchServo, wristPitch, 0, 1000);
  wristPitch = 0;
  moveServo(wristPitchServo, wristPitch, 180, 1000);
  wristPitch = 180;
  moveServo(wristPitchServo, wristPitch, savePosition, 1000);
  wristPitch = savePosition;
    
  delay(1000);
  /* elbow */
  savePosition = elbow;
  moveServo(elbowServo, elbow, 0, 1000);
  elbow = 0;
  moveServo(elbowServo, elbow, 90, 1000);
  elbow = 90;
  moveServo(elbowServo, elbow, savePosition, 1000);  
  elbow = savePosition;
  
  delay(1000);
  /* shoulderPitch */
  savePosition = shoulderPitch;
  moveServo(shoulderPitchServo, shoulderPitch, 180, 1000);
  shoulderPitch = 180;
  moveServo(shoulderPitchServo, shoulderPitch, 90, 1000);
  shoulderPitch = 90;
  moveServo(shoulderPitchServo, shoulderPitch, savePosition, 1000); 
  shoulderPitch = savePosition; 
  
  delay(1000);
  /* shouderYaw */
  savePosition = shoulderYaw;
  moveServo(shoulderYawServo, shoulderYaw, 0, 1000);
  shoulderYaw = 0;
  moveServo(shoulderYawServo, shoulderYaw, 180, 1000);
  shoulderYaw = 180;
  moveServo(shoulderYawServo, shoulderYaw, savePosition, 1000); 
  shoulderYaw = savePosition; 
}
