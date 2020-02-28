#include "WPILib.h"
class VisionSample2014 : public SimpleRobot
{
        Joystick lStick;
        Joystick rStick;
        Joystick sStick;
        Jaguar leftJag1;
        Jaguar leftJag2;
        Jaguar rightJag1;
        Jaguar rightJag2;
        CANJaguar vacuum2;
        CANJaguar vacuum1;
        CANJaguar armJag;
        Talon armWheels;
        Compressor compressor;
        Solenoid shootPiston1;
        Solenoid shootPiston2;
        Solenoid shootPiston3;
        Solenoid shootPiston4;
        Solenoid armBrake;
        Solenoid Shifter;
        Encoder rightDrive;
        Encoder leftDrive;
        AnalogChannel potentiometer
        DriverStationLCD *DriverStation;
        Timer time;
        DigitalModule *mod;
        I2C *arduino;
        int sender [1];
        int counter;
        float resist[10];
        bool on;
        float Current;
        bool ball;
        bool there_yet;
        float holder;
        float volts;
        float roundabout;
        double d;
        bool waited;
        int modulated;
        bool timeReached;
        bool moved;
        int location;
        bool movedBack;
        bool hasntgone;
        bool done;
        float theTimeKeeper;
        bool re;
        float potFlux;
        bool reset;
public:
        VisionSample2014(void):
                lStick(1), 
                rStick(2), 
                sStick(3),
                leftJag1(1), 
                leftJag2(2),
                rightJag1(3), 
                rightJag2(4),
                vacuum2(3),
                vacuum1(7),
                armJag(8),
                armWheels(5),
                compressor(10,1),
                shootPiston1(1),
                shootPiston2(2),
                shootPiston3(3),
                shootPiston4(4),
                armBrake(5),
                Shifter(6),
                rightDrive(1,2,true,Encoder::k4X),
                leftDrive(3,4,true,Encoder::k4X),
                potentiometer(1)
        {
                vacuum1.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
                vacuum1.ChangeControlMode(CANJaguar::kPercentVbus);
                vacuum1.SetVoltageRampRate(64);
                vacuum1.SetSafetyEnabled(false);
                vacuum1.EnableControl();
                vacuum2.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
                vacuum2.ChangeControlMode(CANJaguar::kPercentVbus);
                vacuum2.SetVoltageRampRate(90);
                vacuum2.SetSafetyEnabled(false);
                vacuum2.EnableControl();
                armJag.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
                armJag.ChangeControlMode(CANJaguar::kVoltage);
                armJag.EnableControl(0.0);
                rightDrive.SetDistancePerPulse(0.02531645569620253164556962025316);
                rightDrive.SetMinRate(10);
                rightDrive.Start();
                leftDrive.SetDistancePerPulse(0.02531645569620253164556962025316);
                leftDrive.SetMinRate(10);
                leftDrive.Start();
                compressor.Start();
                armBrake.Set(1);
                shootPistons(0);
                Shifter.Set(0);
                DriverStation = DriverStationLCD::GetInstance();
                ball=true;
                there_yet=false;
                mod = DigitalModule::GetInstance(1);
                arduino=mod->GetI2C(168);
                on=false;
                holder=0;
                counter=0;
                potFlux=.04;
                roundabout=2.45+potFlux;
                modulated=0;
                waited=false;
                timeReached=false;
                moved=false;
                location=100;
                done=false;
                movedBack=false;
                hasntgone=true;
                theTimeKeeper=0;
                re=true;
                reset=false;
        }
        void ManualShift()
        {
                if(lStick.GetRawButton(6)||lStick.GetRawButton(7)||rStick.GetRawButton(10)||rStick.GetRawButton(11))
                {
                        Shifter.Set(1);
                }
                else
                {
                        Shifter.Set(0);
                }
        }
        void shootPistons(bool ii)
        {
                shootPiston1.Set(ii);
                shootPiston2.Set(ii);
                shootPiston3.Set(ii);
                shootPiston4.Set(ii);
        }
        void DriveForward(int distance)
        {
                if(!reset)
                {
                        leftDrive.Reset();
                        rightDrive.Reset();
                        reset=true;
                }
                if((fabs(rightDrive.GetDistance())+fabs(leftDrive.GetDistance()))/2<distance)
                {
                        if(fabs(rightDrive.GetDistance())>fabs(leftDrive.GetDistance()))
                        {
                                SetLeftJags(-.5);
                                SetRightJags(1);
                        }
                        else if(fabs(rightDrive.GetDistance())<fabs(leftDrive.GetDistance()))
                        {
                                SetLeftJags(-1);
                                SetRightJags(.5);
                        }
                        else
                        {
                                SetLeftJags(-1);
                                SetRightJags(1);
                        }
                        roundabout=2.665+potFlux;
                }
                else
                {
                        SetLeftJags(0);
                        SetRightJags(0);
                        shootPistons(1);
                        armJag.Set(0);
                        Wait(1);
                        shootPistons(0);
                        ball=false;        
                        hasntgone=false;
                        moved=true;
                        armWheels.Set(-1);
                }
        }
        void DriveBack(int distance)
        {
                if((fabs(rightDrive.GetDistance())+fabs(leftDrive.GetDistance()))/2>distance)//96
                {
                        if(fabs(rightDrive.GetDistance())>fabs(leftDrive.GetDistance()))
                        {
                                SetLeftJags(1);
                                SetRightJags(-.5);
                        }
                        else if(fabs(rightDrive.GetDistance())<fabs(leftDrive.GetDistance()))
                        {
                                SetLeftJags(.5);
                                SetRightJags(-1);
                        }
                        else
                        {
                                SetLeftJags(1);
                                SetRightJags(-1);
                        }
                }
                else
                {
                        SetLeftJags(0);
                        SetRightJags(0);
                        armWheels.Set(-1);
                        time.Reset();
                        moved=false;
                        movedBack=true;
                        ball=true;
                }
        }
        void Autonomous(void)
        {
                waited=false;
                time.Start();
                there_yet=0;
                while (IsAutonomous() && IsEnabled())
                {
                        arduino->Write(84,80);
                        timeReached=time.Get()>1?true:false;
                        if(!timeReached)
                        {
                                vacuum1.Set(-1);
                                vacuum2.Set(-1);
                        }
                        else
                        {
                                vacuum1.Set(-.66);
                                vacuum2.Set(-.66);
                        }
                        if(moved&&!movedBack&&!done)
                        {
                                roundabout=1.967+potFlux;
                                DriveBack(5);
                        }
                        else if(movedBack&&time.Get()>1.5&&!done)
                        {
                                if(re)
                                {
                                        leftDrive.Reset();
                                        rightDrive.Reset();
                                        re=!re;
                                }
                                if((fabs(rightDrive.GetDistance())+fabs(leftDrive.GetDistance()))/2<96)
                                {
                                        if(fabs(rightDrive.GetDistance())>fabs(leftDrive.GetDistance()))
                                        {
                                                SetLeftJags(-.5);
                                                SetRightJags(1);
                                        }
                                        else if(fabs(rightDrive.GetDistance())<fabs(leftDrive.GetDistance()))
                                        {
                                                SetLeftJags(-1);
                                                SetRightJags(.5);
                                        }
                                        else
                                        {
                                                SetLeftJags(-1);
                                                SetRightJags(1);
                                        }
                                        roundabout=2.67+potFlux;
                                }
                                else
                                {
                                        roundabout=2.67+potFlux;
                                        SetLeftJags(0);
                                        SetRightJags(0);
                                        shootPistons(1);
                                        ball=false;
                                        done=true;
                                        movedBack=true;
                                        moved=true;
                                }
                                armWheels.Set(0);
                        }
                        else if(timeReached&&ball&&hasntgone&&!done)
                        {
                                DriveForward(75);
                        }
                        if(done)
                        {
                                roundabout=2.67+potFlux;
                        }
                        SetArmPosition(roundabout);
                        DriverStation->Printf(DriverStationLCD::kUser_Line3,1,"%f %f",lStick.GetThrottle(),roundabout);
                        DriverStation->UpdateLCD();
                }
        }
        void SetArmPosition(float roundSetArmPosition)
        {
                armBrake.Set(1);
                float position = -150*(potentiometer.GetVoltage()-roundSetArmPosition);
                DriverStation->Printf(DriverStationLCD::kUser_Line3,1,"%f",position);
                if(position>12)
                {
                                position=12;
                }
                else if(position<-12)
                {
                        position=-12;
                }
                armJag.Set(position);
        }
        void SetRightJags(float ii)
        {
                rightJag1.Set(ii);
                rightJag2.Set(ii);
        }
        void SetLeftJags(float ii)
        {
                leftJag1.Set(ii);
                leftJag2.Set(ii);
        }
        void Drive()
        {
                SetLeftJags(-rStick.GetY());
                SetRightJags(lStick.GetY());
        }
        void OperatorControl(void)
        {   
                rightDrive.Reset();
                leftDrive.Reset();
                while (IsOperatorControl())
                {
                        if(sStick.GetRawButton(2))
                        {
                                armWheels.Set(-1);
                                vacuum1.Set(-1);                                
                                vacuum2.Set(-1);
                        }
                        else if (sStick.GetRawButton(11))
                        {
                                vacuum1.Set(0);                                
                                vacuum2.Set(0);
                        }
                        else if(sStick.GetRawButton(1))
                        {
                                shootPistons(1);
                                ball=false;                                                        
                        }
                        else if(sStick.GetRawButton(3))
                        {
                                shootPiston4.Set(1);
                                shootPiston3.Set(1);
                                shootPiston2.Set(1);
                                vacuum1.Set(-.40);                                
                                vacuum2.Set(-.40);
                        }
                        else if(!sStick.GetRawButton(1)&&!sStick.GetRawButton(10)&&!sStick.GetRawButton(2))
                        {
                                vacuum1.Set(-.60);                                
                                vacuum2.Set(-.60);
                                armWheels.Set(0);
                                shootPistons(0);
                        }
                        Drive();
                        if((fabs(rightDrive.GetRate()/12)+(fabs(leftDrive.GetRate())/12))/2>=5||
                                        lStick.GetRawButton(6)||lStick.GetRawButton(7)||rStick.GetRawButton(10)||rStick.GetRawButton(11))
                        {
                                Shifter.Set(1);
                        }
                        else if((fabs(rightDrive.GetRate()/12<3)&&(fabs(leftDrive.GetRate())/12)<3)&&
                                        !(lStick.GetRawButton(6)&&lStick.GetRawButton(7)&&rStick.GetRawButton(10)&&rStick.GetRawButton(11)))
                        {
                                Shifter.Set(0);
                        }
                        if(sStick.GetRawButton(10))
                        {
                                armBrake.Set(1);
                                roundabout=2.675+potFlux;
                                there_yet=false;
                        }
                        if(sStick.GetRawButton(9))
                        {
                                armBrake.Set(1);
                                roundabout=2.295+potFlux;
                                there_yet=false;
                        }
                        if(sStick.GetRawButton(7))
                        {
                                armBrake.Set(1);
                                roundabout=2.35+potFlux;
                                there_yet=false;
                        }
                        if(sStick.GetRawButton(8))
                        {
                                armBrake.Set(1);
                                roundabout=2.55+potFlux;
                                there_yet=false;
                        }
                        if(sStick.GetRawButton(12))
                        {
                                armBrake.Set(1);
                                roundabout=1.967+potFlux;
                                there_yet=false;
                        }
                        SetArmPosition(roundabout);
                        if(sStick.GetRawButton(8))
                        {
                                arduino->Write(84,76);
                        }
                        else
                        {
                                arduino->Write(84,84);
                        }
                        if(Current!=0)
                        {
                                Current = fabs(vacuum1.GetOutputCurrent())+fabs(vacuum2.GetOutputCurrent());        
                                volts = fabs(vacuum1.GetOutputVoltage())+fabs(vacuum2.GetOutputVoltage());
                                resist[counter%10]=volts/Current;
                                holder=0;
                                for(int x=0,z=0;x<10;x++)
                                {
                                        if(resist[x]!=0)
                                        {
                                                holder+=resist[x];
                                        }
                                        else
                                        {
                                                z++;
                                        }
                                        if(x==9)
                                        {
                                                holder/=10-z;
                                        }
                                }
                        }
                        DriverStation->Printf(DriverStationLCD::kUser_Line2,1,"Current resistance: %f  ",(volts/Current));
                        DriverStation->Printf(DriverStationLCD::kUser_Line5,1,"%f %f",fabs(rightDrive.GetDistance()),fabs(leftDrive.GetDistance()));
                        DriverStation->Printf(DriverStationLCD::kUser_Line6,1,"Advrage speed %s",((fabs(rightDrive.GetRate()/12)+fabs(leftDrive.GetRate())/12)>=3&&
                                        (fabs(rStick.GetY())>=.9||fabs(lStick.GetY())>=.9))?"true ":"false");
                        DriverStation->Printf(DriverStationLCD::kUser_Line4,1,"%f  %f",potentiometer.GetVoltage(),potentiometer.GetValue());
                        DriverStation->Printf(DriverStationLCD::kUser_Line1,1,"%f  %f",armJag.GetOutputVoltage());
                        DriverStation->Printf(DriverStationLCD::kUser_Line3,1,"current: %f  ",(vacuum1.GetOutputCurrent()+vacuum2.GetOutputCurrent())/2);
                        DriverStation->Printf(DriverStationLCD::kUser_Line3,1,"%f&f",(fabs(rightDrive.GetDistance())+fabs(leftDrive.GetDistance()))/2,sStick.GetThrottle());
                        DriverStation->UpdateLCD();
                        counter++;
                        Wait(0.005);
                }
        }
};
START_ROBOT_CLASS(VisionSample2014);
