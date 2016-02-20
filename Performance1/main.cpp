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
#define TURN90 205

/*SHAFT ENCODING*/
void shaftEncodingStrait(int percent, int counts)
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((abs(left_encoder.Counts()) + abs(right_encoder.Counts())) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.5);
}

void shaftEncodingTurn(int leftPercent, int rightPercent, int counts)
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(rightPercent);
    left_motor.SetPercent(leftPercent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((abs(left_encoder.Counts()) + abs(right_encoder.Counts())) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.5);
}


int main(void)
{
    int motor_percent = 25; //Input power level here

    //Initialize the screen (GO BUCKS!)
    LCD.Clear( FEHLCD::Scarlet );
    LCD.SetFontColor( FEHLCD::Gray );

    while(!buttons.MiddlePressed()); //Wait for middle button to be pressed
    while(buttons.MiddlePressed()); //Wait for middle button to be unpressed

    shaftEncodingStrait(motor_percent, 567);
    shaftEncodingTurn(-1*motor_percent,motor_percent,TURN90);
    shaftEncodingStrait(motor_percent, 395);
    shaftEncodingTurn(motor_percent,-1*motor_percent,TURN90);
    shaftEncodingStrait(motor_percent, 170);
}
