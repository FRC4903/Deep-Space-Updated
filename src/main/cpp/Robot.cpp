// Foreward for left is positive.  Foreward for right is negative.
#include "frc/TimedRobot.h"
#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <math.h>
#include <AHRS.h>

using namespace frc;
using namespace std;
using namespace rev;

class Robot: public TimedRobot {
  public:

  Joystick controller1{0};
  Joystick controller2{1};

  TalonSRX frontLeft;
  TalonSRX frontRight;
  TalonSRX backLeft;
  TalonSRX backRight;

  //Gyro
  AHRS *ahrs;
  float gyro=0;

  Robot():
    frontLeft(6),
    frontRight(5),
    backLeft(7),
    backRight(4)
  {ahrs = new AHRS(SPI::Port::kMXP);}
  void RobotInit() {
  }
  void RobotPeriodic() {}

  void AutonomousInit() {
  }

  void AutonomousPeriodic() {
  }

  void TeleopInit() {
    ahrs->Reset();
  }

  void TeleopPeriodic() {
    float joyX=controller1.GetRawAxis(0);
    float joyY=controller1.GetRawAxis(1);
    float turn=controller1.GetRawAxis(4);
    gyro= float(normalizeAngle(ahrs->GetAngle()));
    float test= ahrs->GetAngle();
    SmartDashboard::PutNumber("Test Gyro: ", test);
    SmartDashboard::PutNumber("Gyro: ", gyro);


    joyX=(abs(joyX)<=0.05)? 0 : joyX;
    joyY=(abs(joyY)<=0.05)? 0 : joyY;
    turn=(abs(turn)<=0.05)? 0 : turn;

    //driveSystemBasic(getAngle(joyX,joyY), dist(joyX,joyY), turn, gyro);
    driveSystemAdvanced(getAngle(joyX,joyY), dist(joyX,joyY), turn, gyro);
    SmartDashboard::PutNumber("Angle:", getAngle(joyX,joyY));
    SmartDashboard::PutNumber("Power:", dist(joyX,joyY));
  }
  
  float getAngle(float joyX, float joyY){
    float angleVal=0;
    if (joyX==0 && joyY<=0){return 0;}
    if (joyX==0 && joyY>0){return 180;}
    if (joyX<0 && joyY==0){return 270;}
    if (joyX>0 && joyY==0){return 90;}
    if (joyX>0){
      angleVal= (joyY<0)? (atan(joyX/abs(joyY))*180.0/M_PI) : (180-atan(joyX/joyY)*180.0/M_PI);
      return angleVal;
    }
    angleVal= (joyY<0)? (360.0-atan(abs(joyX)/abs(joyY))*180.0/M_PI) : (atan(abs(joyX)/joyY)*180.0/M_PI+180.0);
    return angleVal;    
}
  void driveSystemBasic(float ang, float mag, float turn, float gyro){
    ang=normalizeAngle(360.0-(gyro-ang));
    SmartDashboard::PutNumber("Angle: ", ang);
    mag= min(1.0f,mag);
    int PR[7]= {1,1,0,-1,-1,-1,0};
    int PL[7]= {0,-1,-1,-1,0,1,1};
    for (int i=0; i<7; i+=1){
      if (22.5+45*i<=ang && ang<67.5+45*i){
        frontLeft.Set(ControlMode::PercentOutput, limitDrive(PR[i]*mag+(turn/2)));
        frontRight.Set(ControlMode::PercentOutput, -limitDrive(PL[i]*mag-(turn/2)));
        backLeft.Set(ControlMode::PercentOutput, limitDrive(PL[i]*mag+(turn/2)));
        backRight.Set(ControlMode::PercentOutput, -limitDrive(PR[i]*mag-(turn/2)));
      }
    }
    if (337.5<=ang || ang<22.5){
      frontLeft.Set(ControlMode::PercentOutput, limitDrive(mag+(turn/2)));
      frontRight.Set(ControlMode::PercentOutput, -limitDrive(mag-(turn/2)));
      backLeft.Set(ControlMode::PercentOutput, limitDrive(mag+(turn/2)));
      backRight.Set(ControlMode::PercentOutput, -limitDrive(mag-(turn/2)));
    }
  }


  void driveSystemAdvanced(float ang, float mag, float turn, float gyro){
    //adds field perspective
    ang=normalizeAngle(360.0-(gyro-ang));
    mag=min(1.0f,mag);
    if (ang<90){
      ang*=M_PI/180.0;
      backRight.Set(ControlMode::PercentOutput, limitDrive(-mag+(turn/2)));
      frontLeft.Set(ControlMode::PercentOutput, limitDrive(mag+(turn/2)));
      frontRight.Set(ControlMode::PercentOutput, limitDrive(-mag*(1.0-tan(ang))/(1.0+tan(ang))+(turn/2)));
      backLeft.Set(ControlMode::PercentOutput, limitDrive(mag*(1.0-tan(ang))/(1.0+tan(ang))+(turn/2)));
    }
    if (ang>=90 && ang<180){
      ang*=M_PI/180.0;
      frontRight.Set(ControlMode::PercentOutput, limitDrive(mag+(turn/2)));
      backLeft.Set(ControlMode::PercentOutput, limitDrive(-mag+(turn/2)));
      backRight.Set(ControlMode::PercentOutput, limitDrive(-mag*(1.0-tan(ang-M_PI/2.0))/(1.0+tan(ang-M_PI/2.0))+(turn/2)));
      frontLeft.Set(ControlMode::PercentOutput, limitDrive(mag*(1.0-tan(ang-M_PI/2.0))/(1.0+tan(ang-M_PI/2.0))+(turn/2)));
    }
    if (ang>=180 && ang<270){
      ang*=M_PI/180.0;
      frontLeft.Set(ControlMode::PercentOutput, limitDrive(-mag+(turn/2)));
      backRight.Set(ControlMode::PercentOutput, limitDrive(mag+(turn/2)));
      frontRight.Set(ControlMode::PercentOutput, limitDrive(-mag*(tan(ang-M_PI)-1.0)/(1.0+tan(ang-M_PI))+(turn/2)));
      backLeft.Set(ControlMode::PercentOutput, limitDrive(mag*(tan(ang-M_PI)-1.0)/(1.0+tan(ang-M_PI))+(turn/2)));
    }
    if (ang>=270 && ang<360){
      ang*=M_PI/180.0;
      frontRight.Set(ControlMode::PercentOutput, limitDrive(-mag+(turn/2)));
      backLeft.Set(ControlMode::PercentOutput, limitDrive(mag+(turn/2)));
      frontLeft.Set(ControlMode::PercentOutput, limitDrive(mag*(tan(ang-3.0*M_PI/2.0)-1.0)/(1.0+tan(ang-3.0*M_PI/2.0))+(turn/2)));
      backRight.Set(ControlMode::PercentOutput, limitDrive(-mag*(tan(ang-3.0*M_PI/2.0)-1.0)/(1.0+tan(ang-3.0*M_PI/2.0))+(turn/2)));
    }
  }
  float limitDrive(float val){
    return max(-1.0f,min(1.0f,val));
  }
  float dist(float point1, float point2){
    return pow(pow(point1,2)+pow(point2,2),0.5);
  }
  float normalizeAngle(float angle){
    if (angle<0){return 360+int(angle)%360;}
    return int(angle)%360;
  }

};



#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif