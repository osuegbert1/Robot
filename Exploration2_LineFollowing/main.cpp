#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <stdlib.h>

AnalogInputPin leftSensor(FEHIO::P0_2);
AnalogInputPin middleSensor(FEHIO::P0_1);
AnalogInputPin rightSensor(FEHIO::P0_0);
FEHMotor right_motor(FEHMotor::Motor1,12.0);
FEHMotor left_motor(FEHMotor::Motor0,12.0);
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P1_0);
DigitalEncoder left_encoder(FEHIO::P1_1);

#define LINE_ON_RIGHT 0
#define ON_LINE 1
#define LINE_ON_LEFT 2
#define FASTER 50
#define SLOWER 20
#define LEFT_THRESHOLD 2.1
#define MIDDLE_THRESHOLD 2.5
#define RIGHT_THRESHOLD 1.5

//int main() {
//    int state = LINE_ON_LEFT;
//    while (true) {
//        switch(state) {

//        case LINE_ON_LEFT:
//            LCD.WriteLine("IN LINE_ON_LEFT");
//            right_motor.SetPercent(FASTER);
//            left_motor.SetPercent(SLOWER);
//            if (middleSensor.Value() > MIDDLE_THRESHOLD) {
//                state = ON_LINE;
//            }
//            if( rightSensor.Value() > RIGHT_THRESHOLD) {
//                state = LINE_ON_RIGHT;
//            }
//            break;

//        case LINE_ON_RIGHT:
//            LCD.WriteLine("IN LINE_ON_RIGHT");
//            right_motor.SetPercent(SLOWER);
//            left_motor.SetPercent(FASTER);
//            if (middleSensor.Value() > MIDDLE_THRESHOLD) {
//                state = ON_LINE;
//            }
//            if(leftSensor.Value() > LEFT_THRESHOLD) {
//                state = LINE_ON_LEFT;
//            }
//            break;

//        case ON_LINE:
//            LCD.WriteLine("IN ON_LINE");
//            right_motor.SetPercent(SLOWER);
//            left_motor.SetPercent(SLOWER);
//            if( rightSensor.Value() > RIGHT_THRESHOLD) {
//                state = LINE_ON_RIGHT;
//            }
//            if( leftSensor.Value() > LEFT_THRESHOLD) {
//                state = LINE_ON_LEFT;
//            }
//            break;
//        }
//        Sleep(1);
//    }
//}


int main(){
    while(true){
        if(leftSensor.Value()>LEFT_THRESHOLD){
            right_motor.SetPercent(FASTER);
            left_motor.SetPercent(SLOWER);
            while(middleSensor.Value()<MIDDLE_THRESHOLD){}
        }
        if(rightSensor.Value()>RIGHT_THRESHOLD){
            right_motor.SetPercent(SLOWER);
            left_motor.SetPercent(FASTER);
            while(middleSensor.Value()<MIDDLE_THRESHOLD){}
        }
        if(middleSensor.Value()>MIDDLE_THRESHOLD){
            right_motor.SetPercent(SLOWER);
            left_motor.SetPercent(SLOWER);
        }
    }
}


/*CALIBRATION*/
//int main(){
//    while(true){
//        if(buttons.MiddlePressed()){
//            LCD.Write("Left: ");
//            LCD.WriteLine(leftSensor.Value());
//            LCD.Write("Middle: ");
//            LCD.WriteLine(middleSensor.Value());
//            LCD.Write("Right: ");
//            LCD.WriteLine(rightSensor.Value());
//        }
//    }
//}


/*SHAFT ENCODING*/
//void move_forward(int percent, int counts) //using encoders
//{
//    //Reset encoder counts
//    right_encoder.ResetCounts();
//    left_encoder.ResetCounts();

//    //Set both motors to desired percent
//    right_motor.SetPercent(percent);
//    left_motor.SetPercent(percent);



//    //While the average of the left and right encoder are less than counts,
//    //keep running motors
//    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

//    //Turn off motors
//    right_motor.Stop();
//    left_motor.Stop();
//    Sleep(0.5);
//}

//void turn(int leftPercent, int rightPercent, int counts) //using encoders
//{
//    //Reset encoder counts
//    right_encoder.ResetCounts();
//    left_encoder.ResetCounts();

//    //Set both motors to desired percent
//    right_motor.SetPercent(rightPercent);
//    left_motor.SetPercent(leftPercent);



//    //While the average of the left and right encoder are less than counts,
//    //keep running motors
//    while((abs(left_encoder.Counts()) + abs(right_encoder.Counts())) / 2. < counts);

//    //Turn off motors
//    right_motor.Stop();
//    left_motor.Stop();
//    Sleep(0.5);
//}

//#define TURN90 205

//int main(void)
//{
//    int motor_percent = 25; //Input power level here

//    //Initialize the screen (GO BUCKS!)
//    LCD.Clear( FEHLCD::Scarlet );
//    LCD.SetFontColor( FEHLCD::Gray );

//    while(!buttons.MiddlePressed()); //Wait for middle button to be pressed
//    while(buttons.MiddlePressed()); //Wait for middle button to be unpressed

//    move_forward(motor_percent, 567); //see function
//    turn(-1*motor_percent,motor_percent,TURN90);
//    move_forward(motor_percent, 395);
//    turn(motor_percent,-1*motor_percent,TURN90);
//    move_forward(motor_percent, 170);
//}









