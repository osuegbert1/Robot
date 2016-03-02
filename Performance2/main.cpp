#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <stdlib.h>

AnalogInputPin CDScell(FEHIO::P2_0);
AnalogInputPin leftSensor(FEHIO::P1_0);
AnalogInputPin middleSensor(FEHIO::P1_1);
AnalogInputPin rightSensor(FEHIO::P0_2);
FEHMotor right_motor(FEHMotor::Motor1,12.0);
FEHMotor left_motor(FEHMotor::Motor0,12.0);
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_0);

#define FASTER 50
#define SLOWER 15
#define LEFT_THRESHOLD 1.5
#define MIDDLE_THRESHOLD 1.5
#define RIGHT_THRESHOLD 1.5
#define TURN_90_COUNTS 700
#define ENCODING_SPEED 40
#define RED_BLUE_THRESHOLD .59
#define NO_LIGHT_THRESHOLD 1.0
#define RED_LIGHT 1
#define BLUE_LIGHT 2
#define NO_LIGHT 3

void drive(int percent){
    right_motor.SetPercent(percent);
    left_motor.SetPercent(1+percent);
}

void stop() {
    right_motor.Stop();
    left_motor.Stop();
    Sleep(.5);
}

/*SHAFT ENCODING*/
void clearCounts() {
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}

int updateCount() {
    return (abs(left_encoder.Counts()) + abs(right_encoder.Counts())) / 2.0;
}

void shaftEncodingStraight(int percent, int counts)
{
    //Reset encoder counts
    clearCounts();

    //Set both motors to desiRED_LIGHT percent
    drive(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while(updateCount() < counts){
        LCD.WriteLine(updateCount());
    }

    //Turn off motors
    stop();
}

void shaftEncodingTurnLeft(int percent, int counts)
{
    //Reset encoder counts
    clearCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while(updateCount() < counts){
        LCD.WriteLine(updateCount());
    }

    //Turn off motors
    stop();
}

void shaftEncodingTurnRight(int percent, int counts)
{
    //Reset encoder counts
    clearCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while(updateCount() < counts){
        LCD.WriteLine(updateCount());
    }

    //Turn off motors
    stop();
}


/*LINE FOLLOWING*/
bool onLine() {
    return (leftSensor.Value() < LEFT_THRESHOLD || middleSensor.Value() < MIDDLE_THRESHOLD || rightSensor.Value() < RIGHT_THRESHOLD);
}

void followLine() {
    if(leftSensor.Value()<LEFT_THRESHOLD){
        right_motor.SetPercent(FASTER);
        left_motor.SetPercent(SLOWER);
    }
    if(rightSensor.Value()<RIGHT_THRESHOLD){
        right_motor.SetPercent(SLOWER);
        left_motor.SetPercent(FASTER);
    }
    if(middleSensor.Value()<MIDDLE_THRESHOLD){
        right_motor.SetPercent(SLOWER);
        left_motor.SetPercent(SLOWER);
    }
}

/*CDS CELL SENSING*/
int senseLight() {
    int light;
    if(CDScell.Value() < RED_BLUE_THRESHOLD) {
        light = RED_LIGHT;
    }
    else if(CDScell.Value() > RED_BLUE_THRESHOLD && CDScell.Value() < NO_LIGHT_THRESHOLD) {
        light = BLUE_LIGHT;
    }
    else {
        light = NO_LIGHT;
    }
}

/*RPS*/
void check_x_plus(float x_coordinate) //using RPS while robot is in the +x direction
{
    //check whether the robot is within an acceptable range
    while(RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1)
    {
        LCD.Write("X-Coor: ");
        LCD.WriteLine(RPS.X());
        if(RPS.X() > x_coordinate)
        {
            drive(-20);
            Sleep(.2);
            stop();
        }
        else if(RPS.X() < x_coordinate)
        {
            drive(20);
            Sleep(.2);
            stop();
        }
    }
}

void check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
{
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
    {
        LCD.Write("Y-Coor: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(20);
            Sleep(.5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(-20);
            Sleep(.5);
            stop();
        }
    }
}

void check_y_plus(float y_coordinate) //using RPS while robot is in the +y direction
{
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
    {
        LCD.Write("Y-Coor: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(-20);
            Sleep(.5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(20);
            Sleep(.5);
            stop();
        }
    }
}

void check_heading(float heading) //using RPS
{
    //you will need to fill out this one yourself and take into account
    //the edge conditions (when you want the robot to go to 0 degrees
    //or close to 0 degrees)
    double startPoint = RPS.Heading();

    while(RPS.Heading() < heading - 1 || RPS.Heading() > heading + 1){
        if (heading - startPoint > 0 ){
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (heading-startPoint < 180){
                turnLeft(13);
                Sleep(.2);
                stop();
            } else {
                turnRight(13);
                Sleep(.2);
                stop();
            }
        } else {
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (startPoint-heading < 180) {
                turnRight(13);
                Sleep(.5);
                stop();
            }else {
                turnLeft(13);
                Sleep(.2);
                stop();
            }
        }
        startPoint = RPS.Heading();
    }
}

int main(void) {
    //Initialize the screen
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    while(senseLight() == NO_LIGHT){
        LCD.WriteLine(senseLight());
    }

    //Wait for middle button to be pressed and unpressed
    //    while(!buttons.MiddlePressed()){}
    //    while(buttons.MiddlePressed()){}

    shaftEncodingStraight(ENCODING_SPEED, 1000);
    //while(!buttons.MiddlePressed()){}
    //while(buttons.MiddlePressed()){}
    shaftEncodingTurn(-1*ENCODING_SPEED,ENCODING_SPEED,300);
    //while(!buttons.MiddlePressed()){}
    //while(buttons.MiddlePressed()){}
    shaftEncodingStraight(100,2000);

//    clearCounts();
//    drive(20);
//    while(updateCount() < 600){
//        LCD.WriteLine(updateCount());
//    }
//    while(!onLine()){}
//    int startTime = TimeNow();
//    while(TimeNow() - startTime < 4 && senseLight() == NO_LIGHT) {
//        followLine();
//        LCD.WriteLine(onLine());
//    }
//    stop();

//    switch (senseLight()) {
//    case BLUE_LIGHT:
//        LCD.Clear(FEHLCD::Black);
//        LCD.WriteLine("BLUE");
//        break;
//    case RED_LIGHT:
//        LCD.Clear(FEHLCD::Black);
//        LCD.WriteLine("RED");
//        break;
//    default:
//        LCD.Clear(FEHLCD::Black);
//        LCD.WriteLine("RED");
//    }

}
