#define Calibrate 0 //Change to 1 see above for details.
#define calDistance 24 //in inches 24inches or 2 foot

#include <Pixy2.h>

int calWidth = 50.5; //Calibrated width reading
int calHeight = 50.5; //Calibrated height reading
int pixelsWidth;   //read by the camera
int pixelsHeight; //read by the camera
float distanceWidth;   //calculated distance based on the width of the object
float distanceHeight;  //calculated distance based on the height of the object 
float widthOfObject = 4.8; //inches (3.75 inches) real size of your object
float heightOfObject = 4.8; //inches (2.5 inches) real size of your object
int focalLengthWidth;  //calculated focal length for width
int focalLengthHeight; //calculated focal length for height
float avg;
int feet;
int inches;
float distancee;

#include <AFMotor.h>

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);
Pixy2 pixy;
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  Serial.print("Starting...\n");
//  
  focalLengthWidth = (calWidth * calDistance) / widthOfObject;
  focalLengthHeight = (calHeight * calDistance) / heightOfObject;
  
  // turn on motor
//  motorL.setSpeed(0);
//  motorR.setSpeed(0);
//  motorL.run(RELEASE);
//  motorR.run(RELEASE);
  pixy.init();

}

void loop() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  blocks = pixy.ccc.getBlocks();
  
  if (blocks){
    i++;
    if (i%50==0){
      for (j=0; j<blocks; j++){
        pixelsWidth = pixy.ccc.blocks[j].m_width;
        pixelsHeight = pixy.ccc.blocks[j].m_height;
        distanceWidth = (widthOfObject * focalLengthWidth) / pixelsWidth;
        distanceHeight = (heightOfObject * focalLengthHeight) / pixelsHeight;
        avg = (distanceWidth + distanceHeight)/2;
        avg = round(avg);
        distancee = avg; 
        Serial.print("Average: ");
        Serial.print(avg);
        Serial.println("in");
      }  
    }
  }

  uint8_t lli;
  
  //Serial.print("Fucking Forard");
  
  motorL.run(FORWARD);
  for (lli=0; lli<255; lli++) {
    motorL.setSpeed(lli);  
    delay(10);
 }
 
  for (lli=255; lli!=0; lli--) {
    motorL.setSpeed(lli);  
    delay(10);
 }
}
