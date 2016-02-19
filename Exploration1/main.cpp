#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHMotor.h>

ButtonBoard buttons(FEHIO::Bank3);
AnalogInputPin CDSCell(FEHIO::P0_0);
FEHServo Servo(FEHServo::Servo0);
DigitalInputPin Switch1(FEHIO::P0_1);
DigitalInputPin Switch2(FEHIO::P0_2);
DigitalInputPin Switch3(FEHIO::P0_3);
DigitalInputPin Switch4(FEHIO::P0_4);
FEHMotor rightMotor(FEHMotor::Motor0,12.0);
FEHMotor leftMotor(FEHMotor::Motor1,12.0);


//int main(void)
//{
//    LCD.Clear( FEHLCD::Black );
//    LCD.SetFontColor( FEHLCD::White );

//    while(true)
//    {
//        LCD.WriteLine(CDSCell.Value());
//        Sleep(1000);
//    }
//}

//int main(void)
//{
//    Servo.TouchCalibrate();
//}

//int main(void){
//    Servo.SetMax(2500);
//    Servo.SetMin(500);
//    while(true){
//        Servo.SetDegree(54.545*CDSCell.Value());
//    }
//}

int main(void){
    //forward
    rightMotor.SetPercent(50);
    leftMotor.SetPercent(50);
    //hit
    while(Switch1.Value() && Switch2.Value()){}
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(1000);
    //adjust
    rightMotor.SetPercent(-50);
  //  leftMotor.SetPercent(-20);
    while(Switch3.Value() && Switch4.Value()){}
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(500);
    while(Switch3.Value() || Switch4.Value()){
        if(Switch3.Value()){
            leftMotor.SetPercent(-20);
        }
        if(Switch4.Value()){
            rightMotor.SetPercent(-20);
        }
    }
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(1000);
    //forward
    rightMotor.SetPercent(50);
    leftMotor.SetPercent(50);
    while(Switch1.Value() && Switch2.Value()){}
    rightMotor.Stop();
    leftMotor.Stop();
   // rightMotor.SetPercent(-50);
    leftMotor.SetPercent(-50);
    while(Switch3.Value() && Switch4.Value()){}
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(500);
    while(Switch3.Value() || Switch4.Value()){
        if(Switch3.Value()){
            leftMotor.SetPercent(-20);
        }
        if(Switch4.Value()){
            rightMotor.SetPercent(-20);
        }
    }
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(1000);
    rightMotor.SetPercent(50);
    leftMotor.SetPercent(50);
    while(Switch1.Value() && Switch2.Value()){}
    rightMotor.Stop();
    leftMotor.Stop();

}
