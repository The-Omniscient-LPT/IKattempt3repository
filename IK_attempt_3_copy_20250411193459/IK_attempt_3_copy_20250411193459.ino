// made by Harry Le Poer Trench 11/4/2025
// sections of code taken from Lab_7_Robot_Arm_Control_Code_RM2025_print_degrees by Rayner Moss,
// the sections copied can be most of the var names, init function and slow raise, other parts are majority own work 
#include <VarSpeedServo.h>
#include <SoftwareSerial.h>

//#define DEBUG // this is how you actually do debug testing
#define PI 3.14159265

#define BASE 11
#define SHOULDER 10
#define ELBOW 9
#define WRIST 6
#define WRIST_ROTATE 5
#define GRIPPER 3

long current_time;
long previous_time;
long polling_time;
long previous_print;
long grip_time;
int polling_interval = 50;
int grip_interval = 1000;
int print_interval = 1000;
int shoulder_min = 840;

int pot_read_counts = 0;
float pot_average_value[6];
float pot_sum[6];

#define p1 A10
#define p2 A11
#define p3 A12
#define p4 A13
#define p5 A14
#define p6 A15
#define ENABLE_PIN 12

VarSpeedServo s1;
VarSpeedServo s2;
VarSpeedServo s3;
VarSpeedServo s4;
VarSpeedServo s5;
VarSpeedServo s6;

int pot[6] = {p1,p2,p3,p4,p5,p6};

int min_distance = 180; // sqrt(125^2+125^2) rounded up
int lengs[5] = {125,125,140,0,0};// TODO: change this, these are wrong, but theyre close enough that they almost work, represent distance between J1-J2, J2-J3, J3-gripper, J1-J3, J1-desired, respectively,
// units are in mm currently but just need to be in same unit as xyz coordinate. 
int servo_speed[6] = {20,15,20,30,50,50};


// not sure if this is needed... remenant of old code, 
int send_pos[6];

// coordinate systems,
double temp_xyz[3] = {0,0,0};
int xyz[3] = {0,0,0};
const int xyz_max[3] = {400,400,200}; // the max values in any given direction. // note: y can go from -max to max, x & z cannot do that,
int gripper_angle = 45;
int servo_angle_5 = 0;
int servo_angle_6 = 0;
int wz[2] = {0,0}; // the planar form of xyz, dont think its that complicated,
int modwz[2] = {0,0}; // wz at the other end of gripper limb, aka wz version of pos J3;
int joint_angles[4] = {0,0,0,0}; // jopint angles are 0-360, the angular difference between the two limbs either side of the joint from the perspective of the floor, joint 0 being the exception, with 0 being straight ahead.
int servo_value[6]; //preprotected servo outputs
int protected_servo_value[6]; //postprotected servo outputs 
bool connected = false;



void setup() {
  pinMode(ENABLE_PIN, OUTPUT);                             // Pin 12 is required to be written HIGH to enable the servo driver shield

  pinMode(p1, INPUT);                                      // Set p1 or pot 1 to be input
  pinMode(p2, INPUT);                                      // Set p2 or pot 2 to be input
  pinMode(p3, INPUT);                                      // Set p3 or pot 3 to be input
  pinMode(p4, INPUT);                                      // Set p4 or pot 4 to be input
  pinMode(p5, INPUT);                                      // Set p5 or pot 5 to be input
  pinMode(p6, INPUT);                                      // Set p6 or pot 6 to be input

  Serial.begin(115200);                                    // Begin serial comms over USB at 115200 baud rate
  Serial1.begin(115200);                                   // Begin serial comms between ESP8266 and Arduino at 115200 baud rate

  digitalWrite(ENABLE_PIN, HIGH);                          // Set the enable pin High on the Braccio shield

  SLOW_RAISE(50);                                          // Initialise the arm by slowly raising to a known position before using pot values

  Serial.println("Arm Raised");

}

void loop() 
{
  current_time = millis();                                  // Variable to store the time that this loop started in milliseconds

  read_pots();                                              // Function to read the pot values and store for averaging

  // If more time than the polling interval has past, read the pots 
  if(current_time - polling_time > polling_interval)
  {
    if (!connected){
      average_pot_values();
      pot_values_to_outputs();
    } else {
      // online code here,
    }
    assign_angles();
    angles_to_uptime();
    servo_protection();
    write_servos();
    polling_time = millis();
  }

  // If the time since the last print is greater than the print interval, do this
  if(current_time - previous_print > print_interval)
  {
    previous_print = millis();
  }
}


void read_pots()
{           
  pot_read_counts++;

  for(int i = 0; i < 6; i++)
  {
    pot_sum[i] += analogRead(pot[i]);
  }
}
//=========================================================================================================
//Average all pot values that have been taken in the time since the last check

void average_pot_values()
{
  for(int i = 0; i < 6; i++)
  {
    pot_average_value[i] = pot_sum[i]/pot_read_counts;
    Serial.print(pot_average_value[i]);
    Serial.print(" ");
  }
  for(int i = 0; i < 6; i++){
     pot_sum[i] = 0;
  }
  Serial.println();
}

void pot_values_to_outputs(){
  temp_xyz[0] = (pot_average_value[0]*xyz_max[0])/1023;
  temp_xyz[1] = (pot_average_value[1]*2*xyz_max[1])/1023-xyz_max[1]/2;
  temp_xyz[2] = (pot_average_value[2]*xyz_max[2])/1023;
  for (int i = 0; i < 3; i++){
    xyz[i] = temp_xyz[i];
  }
  gripper_angle = (pot_average_value[3]/1023)*90; // gri[pper angle is constrained from 0 to 90 to stop it from crashing through the floor
  // take gripper values normally
  servo_value[4] = (pot_average_value[4]/1023)*1855+545;
  servo_value[5] = (pot_average_value[5]/1023)*1000+1400;
}

void printvalues() {
  for (int i = 0; i < 3; i++){
    Serial.print(xyz[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void assign_angles(){ // if this fails it does not change the angles from the previous go, please do not modify joint_angles needlessly, I saw what you did with your other one and if you do, we have to add default values for this,

    //printvalues();

  // first checks that the given point is within possible range, 
  if (sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]) > (lengs[0]+lengs[1]+lengs[2])){
    #ifdef DEBUG
    Serial.println("True out of bounds");
    #endif
    return;
  }
  
  // calculate the rotational position
  joint_angles[0] = radian_to_degrees(atan(xyz[0]/xyz[1]));
  // sets up the new wy
  wz[0] = sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]);
  wz[1] = xyz[2];
  // finds the wz position of J3;
  modwz[0] = wz[0] - cos(degrees_to_radian(gripper_angle))*lengs[2];
  modwz[1] = wz[1] - sin(degrees_to_radian(gripper_angle))*lengs[2];
  // finds the angle of J1,
  lengs[3] = sqrt(modwz[0]*modwz[0]+modwz[1]*modwz[1]);
  // checks that the point is actually in range of the angled limb
  if (lengs[3] > (lengs[0]+lengs[1])){
    #ifdef DEBUG
    Serial.println("J3 out of bounds (too far)");
    #endif
    return;
  }
  // checks if distance is too tight, 
  if (lengs[3] < min_distance) {
    #ifdef DEBUG
    Serial.println("J3 out of bounds (too close)");
    #endif
    return;
  }
  // early pages of well bound turquoise note book for explanation of these two, joint angle 1 is between the floor and limb instead of limb and limb for obvious reasons (there is only 1 limb), 
  //                 angle between ground and target from J1.   angle between target and J2 from J1
  joint_angles[1] = radian_to_degrees(atan(modwz[1]/modwz[0])) + radian_to_degrees(cosine_rule(lengs[1],lengs[0],lengs[3]));
  //                  angle between J1 and J3 from J2
  joint_angles[2] = radian_to_degrees(cosine_rule(lengs[3],lengs[0],lengs[1]));
  //                  angle between target and J1 from J3,                     angle between J1 and J2 from J3
  joint_angles[3] = radian_to_degrees(cosine_rule(lengs[4],lengs[3],lengs[2]))+radian_to_degrees(cosine_rule(lengs[0],lengs[1],lengs[3]));
  

  #ifdef DEBUG
  Serial.println("Angles J0 J1 J2 J3");
  Serial.print("      ");
  for (char i = 0; i < 4; i++){
    Serial.print(joint_angles[i]);
    Serial.print(" ");
  }
  Serial.println();
  #endif
}

void angles_to_uptime(){
  // each joint needs to be assigned individually cause their rules are ... interesting on limits, also angle to angle,
  // joint 0-3 in order,
  // converts between angle in degreees to uptime
  servo_value[0] =  ((joint_angles[0]+90)/180)*1855+545;
  servo_value[1] = ((joint_angles[1])/180)*1855+545;
  servo_value[2] = ((joint_angles[2]-90)/180)*1855+545;
  servo_value[3] = ((joint_angles[3]-90)/180)*1855+545;
  
}

void servo_protection(){
  for (char i = 0; i < 6; i++){
    // TODO: increase constraints so that servos 3 and 4 can bend more than 180 degrees (the robot can do this without breaking,)
    protected_servo_value[i] = constrain(servo_value[i],545,2400); // default
  }
  protected_servo_value[1] = constrain(servo_value[1],840,2100); // this one needs EXTRA shielding,
  // TODO: servo 5 and 6 also need to be constrained, do not know limits, they are covered by current code for 0-180 degrees
}

void write_servos()
{
  s1.write(protected_servo_value[0], servo_speed[0]); send_pos[0] = s1.readMicroseconds();
  s2.write(protected_servo_value[1], servo_speed[1]); send_pos[1] = s2.readMicroseconds();
  s3.write(protected_servo_value[2], servo_speed[2]); send_pos[2] = s3.readMicroseconds();
  s4.write(protected_servo_value[3], servo_speed[3]); send_pos[3] = s4.readMicroseconds();
  s5.write(protected_servo_value[4], servo_speed[4]); send_pos[4] = s5.readMicroseconds();
  s6.write(protected_servo_value[5], servo_speed[5]); send_pos[5] = s6.readMicroseconds();
}
// ========================= suplimentary functions ===========================

int radian_to_degrees(double input){
  double output = input*180/PI;
  return output;
}

double degrees_to_radian(double input){
  double output = input*PI/180;
  return output;
}

double cosine_rule(int a, int b, int c){
  double cosA = sqrt((b*b+c*c-a*a)/(2*b*c));
  return acos(cosA);
}


int SLOW_RAISE(int rate)
{
  bool raised = false;        // Set arm raised boolean to false

  int min = 544;              //Set the minimum value for servo.write in microseconds 
  int max = 2400;             //Set the maximum value for servo.write in microseconds 

  int i = 10;                 //Set Digital Pin 10 or Servo #2 (Shoulder) to be the first to be raised
  int l = min;                //Set variable 'l' to equal variable 'min' 
  int r = max;                //Set variable 'r' to equal variable 'max' 
  int lp = min;               //Set variable 'lp' or l-previous to equal 'min' , this is used to progress the 'initialising bar'


  Serial1.println("AT");
  delay(1000);
  Serial1.println("AT+RST");
  delay(3000);
  Serial1.println("AT+CWMODE=3");
  delay(2000);
  Serial1.println("AT+CIPMUX=1");
  delay(2000);
  Serial1.println("AT+CIPSERVER=1,80");
  delay(1000);
  
  while(raised != true)       //Loop until all servos are confirmed to be raised to roughly 90 degrees using 1500 microseconds as the centre value
  { 

    lp = l;                    //Let 'l previous' equal 'l'

  //---------------------------------------------------------------------------------------------------------------------------------
  //                                           **Only used to display that the code is progressing through the raising procedure**
  //---------------------------------------------------------------------------------------------------------------------------------

    Serial.println();
    Serial.print("Initialising : ");
    
    if(i == 10){Serial.println("Shoulder");}        //Print a different value based on which servo is being initialised based on the value of 'i'
    else if(i == 9){Serial.println("Elbow");}
    else if(i == 6){ Serial.println("Wrist");}
    else if(i == 5){Serial.println("Wrist Rotation");}
    else if(i == 3){Serial.println("Gripper");}
    else if(i == 11){Serial.println("Base");}

  //-----------------------------------------------------------------------------------------------------------------------------------------------
  //          'While' loop used to slowly raise the arm into a vertical position irrespective of what the current potentiometer values are set to
  //-----------------------------------------------------------------------------------------------------------------------------------------------

    while( l < 1500 && r > 1500 )        //Loop until both 'l' and 'r' values are roughly 1500 indicating that the servo is centred
    {
      digitalWrite(i, LOW);             //Write the current servo pin low and then high towards 0 degrees, gradually increasing towards the centre position as the code loops
      delayMicroseconds(l);
      digitalWrite(i, HIGH);
      delayMicroseconds(l);

      digitalWrite(i, LOW);             //Write the current servo pin low and then high towards 180 degrees, gradually decreasing towards the centre position as the code loops
      delayMicroseconds(r);
      digitalWrite(i, HIGH);
      delayMicroseconds(r);

      digitalWrite(i, LOW);             //Write the current servo pin low for 1500us and then high for 1500us 
      delayMicroseconds(1500);          //replicating servo.writeMicroseconds without attaching the servo which will cause rapid movements to centre position
      digitalWrite(i, HIGH);
      delayMicroseconds(1500);      

      l = l + 6;             //Increase the value of 'l' towards 1500 by a value of 8 microseconds
      r = r - 6;             //Decrease the value of 'r' towards 1500 by a value of 8 microseconds

  //-------------------------------------------------------------------------------------------------------------------------------------------------------------
  //       Only used to show the progression of the arm raising process
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------
      if(l - lp > 100)          //Print a 'progress dot' every time variable 'l' increases by 100
      {
        Serial.print(".");
        lp = l;                   //Reset variable 'l previous' to equal 'l' to allow the process to repeat
      }
    }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------
  //          End of the 'while' loop for the specific servo
  //------------------------------------------------------------------------------------------------------------------------------------------------------------- 

    if(i == 10){l = min; r = max; s2.attach(SHOULDER); delay(10); i = 9;}              //Once the current servo has reached the 'middle' position it is attached without issue, reset the variables 'l' and 'r' to their original values,
//                                                                                       then set the 'PRE' value of the servo in the array to 90 degrees and progress onto the next servo in the series
    else if(i == 9){l = min; r = max; s3.attach(ELBOW); delay(10); i = 6;}               
    else if(i == 6){ l = min; r = max; s4.attach(WRIST); delay(10); i = 5;}
    else if(i == 5){ l = min; r = max; s5.attach(WRIST_ROTATE); delay(10); i = 3;}
    else if(i == 3){ l = min; r = max; s6.attach(GRIPPER); delay(10); i = 11;}
    else if(i == 11){l = min; r = max; s1.attach(BASE); delay(10); raised = true;}     //Set 'Base' to 90 degrees last to minimise risk of arm swinging in a non-vertical position, then set boolean 'raised' to true ending the loop
  }
  return 0;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//         End of the safe arm raising 'while' loop
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
}