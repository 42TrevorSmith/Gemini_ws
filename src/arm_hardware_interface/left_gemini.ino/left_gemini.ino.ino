#include <Wire.h>

#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <string.h>
ros::NodeHandle  nh;
//###############################################################################


// DC hobby servo
Servo RE_servo;
Servo LE_servo;
Servo RS_servo;
Servo LS_servo;
Servo W1_servo;
Servo W2_servo;

// hand servos
Servo I_servo;
Servo M_servo;
Servo R_servo;
Servo P_servo;
Servo T1_servo;
Servo T2_servo;

long H = 0;


//transform-------------
BLA::Matrix<4> Sp;

BLA::Matrix<4> Ep;

BLA::Matrix<4> Wp;

BLA::Matrix<4, 4> ROTX;
BLA::Matrix<4, 4> ROTY;
BLA::Matrix<4, 4> ROTZ;
BLA::Matrix<4, 4> TRANS;
BLA::Matrix<4, 4> TF;
//--------------------------
//############################ ROS Stuff ###############################################
void messageCb(const std_msgs::Float32MultiArray& arm_points) {
  Sp.Fill(0);
  Sp(0) = arm_points.data[0];
  Sp(1) = arm_points.data[1];
  Sp(2) = arm_points.data[2];
  Sp(3) = 1;
  Ep.Fill(0);
  Ep(0) = arm_points.data[3];
  Ep(1) = arm_points.data[4];
  Ep(2) = arm_points.data[5];
  Ep(3) = 1;
  Wp.Fill(0);
  Wp(0) = arm_points.data[6];
  Wp(1) = arm_points.data[7];
  Wp(2) = arm_points.data[8];
  Wp(3) = 1;

  H = round(arm_points.data[9]);

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("l_arm_points", messageCb );
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

//############################ Setup ###############################################


void setup() {
  //Serial.begin(9600);           // set up Serial library at 9600 bps
//  Serial.println("Hellow world");

  // assign pwm pins to the arm servos
  RE_servo.attach(4); //purple
  LE_servo.attach(5); //yellow
  RS_servo.attach(6); //orange
  LS_servo.attach(7); //white
  W1_servo.attach(3); //blue
  W2_servo.attach(2); //green

  I_servo.attach(10); //purple  
  M_servo.attach(11); //yellow  
  R_servo.attach(12); //orange  
  P_servo.attach(9); //white    
  T1_servo.attach(13); //blue
  T2_servo.attach(44); //green


  //-------ros stuff
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  //----------Shoulder point
  Sp(0) = 0;
  Sp(1) = -8;
  Sp(2) = 54;
  Sp(3) = 1;

  //----------Elbow point
  Ep(0) = 0;
  Ep(1) = -8; //18
  Ep(2) = 54; //54
  Ep(3) = 1;

  //----------Wrist point
  Wp(0) = 0; //12
  Wp(1) = -8; //18
  Wp(2) = 54; //54
  Wp(3) = 1;

//------------hand closed / open
  H = 0;


}


// Test the servo!------------------------------------------------------------------------------
void loop() {

  
  //========================================= Input ============================================
  
  long roll = 0;
  long pitch = 0;

 

  nh.subscribe(sub);
  str_msg.data = hello; //output;
  chatter.publish( &str_msg );

  long index = H;
  long middle = H;
  long ring = H;
  long pinky = H;
  long thumb = H;
  long thumb_bend = 0;
  
  //========================================== inverse kinematics ==============================
  // -------------------------------------------------shoulder angles
  transform(Sp(0), Sp(1), Sp(2), -1.57, 0, 0);
  BLA::Matrix<4, 4> tf_B_to_S = TF;
  BLA::Matrix<4> E_in_S = tf_B_to_S.Inverse() * Ep;

  double dx1 = E_in_S(0);
  double dy1 = E_in_S(1);
  double dz1 = E_in_S(2);
  double l1 = sqrt((dx1 * dx1) + (dy1 * dy1));
  double link1 = sqrt((dx1 * dx1) + (dy1 * dy1) + (dz1 * dz1));

  long thetaS = long(asin(dy1 / l1) * 180 / 3.14159);
  long phiS = long(asin(l1 / link1) * 180 / 3.14159);
  if (l1 == 0) {
    thetaS = 0;
  }

  double phiS_rad = atan2(l1, dz1);
  double thetaS_rad = atan2(dy1, dx1);
  //-------------------------------------------------elbow angles
  transform(dx1, dy1, dz1, phiS_rad, 0, thetaS_rad);
  BLA::Matrix<4, 4> tf_S_to_E = TF;
  BLA::Matrix<4> W_in_E = tf_S_to_E.Inverse() * tf_B_to_S.Inverse() * Wp;

  double dx2 = W_in_E(0);
  double dy2 = W_in_E(1);
  double dz2 = W_in_E(2);
  double l2 = sqrt(dx2 * dx2 + dy2 * dy2);
  double link2 = sqrt(dx2 * dx2 + dy2 * dy2  + dz2 * dz2);

  long thetaE = long(asin(dy2 / l2) * 180 / 3.14159);
  long phiE = long(asin(l2 / link2) * 180 / 3.14159);
  if (l2 == 0) {
    thetaE = 0;
  }

  //-----------------------------------------constrain IK output to 0-90 deg
  thetaE = constrain(thetaE, 0, 90);
  phiE =  constrain(phiE, 0, 90);
  thetaS = constrain(thetaS, 0, 90);
  phiS = constrain(phiS, 0, 90);


  //=========================================== motor interface =========================================
//    thetaE = 0;
//    phiE = 0;
//    thetaS = 0;
//    phiS = 0;

  // ----------------------------------------------calculate angle to move joint
  long rightE_deg = (thetaE + phiE);
  long leftE_deg = (thetaE - phiE + 90);
  long rightS_deg = (thetaS + phiS);
  long leftS_deg = (thetaS - phiS + 90);
  //-----------------------------------------------covert to pwm
  long rightE_pwm =  rightE_deg * 255 / 360;
  long leftE_pwm =  leftE_deg * 255 / 360;

  long rightS_pwm =  rightS_deg * 255 / 270;
  long leftS_pwm =  leftS_deg * 255 / 270;
  long w1_pwm = (roll) * 255 / 360 + 127;
  long w2_pwm = (pitch) * 255 / 360 + 127;

  //-----------------------------------------------write to motors
  RE_servo.write(rightE_pwm);
  LE_servo.write(leftE_pwm);
  RS_servo.write(rightS_pwm);
  LS_servo.write(leftS_pwm);
  W1_servo.write(w1_pwm);
  W2_servo.write(w2_pwm);

  //-------------------------------------------------actuate fingers
  I_servo.write(index * 255);
  M_servo.write(middle * 255);
  R_servo.write(ring * 255);
  P_servo.write(pinky * 255);
  T1_servo.write(thumb * 255);
  T2_servo.write(thumb_bend * 255);

//  Serial.print("thetaS = ");
//  Serial.print(thetaS);
//  Serial.print("   phiS = ");
//  Serial.print(phiS);
//  Serial.print("   LS pwm = ");
//  Serial.print(leftS_pwm);
//  Serial.print("   RS_pwm = ");
//  Serial.println(rightS_pwm);


  nh.spinOnce();
  delay(1); //500

}
//=================================== FUNCTIONS =============================================

void rotx(double xx) { //must be in radians
  ROTX.Fill(0);
  ROTX(0, 0) = 1;
  ROTX(1, 1) = cos(xx);
  ROTX(1, 2) = -sin(xx);
  ROTX(2, 1) = sin(xx);
  ROTX(2, 2) = cos(xx);
  ROTX(3, 3) = 1;

}


void roty(double yy) { //must be in radians
  ROTY.Fill(0);
  ROTY(0, 0) = cos(yy);
  ROTY(0, 2) = sin(yy);
  ROTY(1, 1) = 1;
  ROTY(0, 2) = -sin(yy);
  ROTY(2, 2) = cos(yy);
  ROTY(3, 3) = 1;


}


void rotz(double zz) { //must be in radians
  ROTZ.Fill(0);
  ROTZ(0, 0) = cos(zz);
  ROTZ(0, 1) = -sin(zz);
  ROTZ(1, 0) = sin(zz);
  ROTZ(1, 1) = cos(zz);
  ROTZ(2, 2) = 1;
  ROTZ(3, 3) = 1;


}

void trans(double x, double y, double z) {
  TRANS.Fill(0);
  TRANS(0, 0) = 1;
  TRANS(1, 1) = 1;
  TRANS(2, 2) = 1;
  TRANS(3, 3) = 1;
  TRANS(0, 3) = x;
  TRANS(1, 3) = y;
  TRANS(2, 3) = z;

}

void transform(double x, double y, double z, double xx, double yy, double zz) {
  trans(x, y, z);
  rotx(xx);
  roty(yy);
  rotz(zz);
  TF =  TRANS * ROTX * ROTY * ROTZ;
}
