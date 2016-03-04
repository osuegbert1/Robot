#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <stdlib.h>

DigitalInputPin frontLeftBump(FEHIO::P2_3);
DigitalInputPin frontRightBump(FEHIO::P2_2);
DigitalInputPin backLeftBump(FEHIO::P2_2);
DigitalInputPin backRightBump(FEHIO::P0_3);
AnalogInputPin CDScell(FEHIO::P2_0);
AnalogInputPin leftSensor(FEHIO::P1_2);
AnalogInputPin middleSensor(FEHIO::P1_1);
AnalogInputPin rightSensor(FEHIO::P1_0);
FEHMotor right_motor(FEHMotor::Motor1,12.0);
FEHMotor left_motor(FEHMotor::Motor0,12.0);
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_1);

#define FASTER 60
#define SLOWER 10
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

void waitForMiddlePress() {
    while(!buttons.MiddlePressed()){}
    while(buttons.MiddlePressed()){}
}

void drive(int percent){
    right_motor.SetPercent(percent);
    left_motor.SetPercent(1+percent);
}

void turnLeft(int percent) {
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);
}

void turnRight(int percent) {
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);
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

void shaftEncodingStraight(int percent, double distance)
{
    int counts = 318*distance/(2.5*3.1415);
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
    turnLeft(percent);

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
    turnRight(percent);

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

void printLight(){
    switch (senseLight()) {
    case BLUE_LIGHT:
        LCD.Clear(FEHLCD::Black);
        LCD.WriteLine("BLUE");
        break;
    case RED_LIGHT:
        LCD.Clear(FEHLCD::Black);
        LCD.WriteLine("RED");
        break;
    default:
        LCD.Clear(FEHLCD::Black);
        LCD.WriteLine("RED");
    }
}

/*RPS*/
void check_x_plus(float x_coordinate) //using RPS while robot is in the +x direction
{
    //check whether the robot is within an acceptable range
    while(RPS.X() < x_coordinate - .5 || RPS.X() > x_coordinate + .5)
    {
        LCD.Write("X-Coor: ");
        LCD.WriteLine(RPS.X());
        if(RPS.X() > x_coordinate)
        {
            drive(-12);
            while(RPS.X() < x_coordinate - .5 || RPS.X() > x_coordinate + .5);
            stop();
        }
        else if(RPS.X() < x_coordinate)
        {
            drive(12);
            while(RPS.X() < x_coordinate - .5 || RPS.X() > x_coordinate + .5);
            stop();
        }
    }
}

void check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
{
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5)
    {
        LCD.Write("Y-Coor: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(-12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
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

            drive(-12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            drive(12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
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

    while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2){
        if (heading - startPoint > 0 ){
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (heading-startPoint < 180){
                turnLeft(10);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            } else {
                turnRight(10);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            }
        } else {
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (startPoint-heading < 180) {
                turnRight(10);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            }else {
                turnLeft(10);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            }
        }
        startPoint = RPS.Heading();
    }
}

void goToX(double xCoord, int percent) {
    drive(percent);
    while(RPS.X() < xCoord-1 || RPS.X() > xCoord+1){
        LCD.Write("X Coord: ");
        LCD.WriteLine(RPS.X());
    }
}

void goToY(double yCoord, int percent) {
    drive(percent);
    while(RPS.Y() < yCoord-1 || RPS.Y() > yCoord+1){
        LCD.Write("Y Coord: ");
        LCD.WriteLine(RPS.Y());
    }
}

int main(void) {
    //Initialize the screen
    RPS.InitializeTouchMenu();
    LCD.Write("Heading: ");
    LCD.WriteLine(RPS.Heading());
    waitForMiddlePress();

    //    while(senseLight() == NO_LIGHT){
    //        LCD.WriteLine(senseLight());
    //    }

    goToX(28,25);
    check_x_plus(28);
    check_heading(90);
    waitForMiddlePress();
    //    drive(50);
    //    Sleep(3.0);
    //    stop();
    while(RPS.Y()<46.5){
        drive(20);
    }
    stop();
    goToY(46.5,-15);
    check_y_plus(46.5);

    shaftEncodingTurnLeft(20,200);
    check_heading(188);

    drive(20);
    while(!onLine());

    shaftEncodingStraight(10,3);
    shaftEncodingTurnLeft(20,125);
    if(RPS.Heading()>0){
        while(RPS.Heading() < 250){
            turnLeft(10);
        }
    }

    while(frontLeftBump.Value() || frontRightBump.Value()){
        if(onLine() && frontLeftBump.Value() && frontRightBump.Value()){
            LCD.WriteLine(frontLeftBump.Value());
            followLine();
        }
        else {
            drive(12);
        }
    }
    stop();

    shaftEncodingStraight(-10,8);

    shaftEncodingTurnRight(20,110);

    drive(15);
    Sleep(1.5);
    while(!onLine);

    shaftEncodingStraight(10,3);
    shaftEncodingTurnLeft(20,125);
    if(RPS.Heading()>0){
        while(RPS.Heading() < 250){
            turnLeft(10);
        }
    }

    while(frontLeftBump.Value() || frontRightBump.Value()){
        if(onLine() && frontLeftBump.Value() && frontRightBump.Value()){
            followLine();
        }
        else {
            drive(12);
        }
    }
    stop();

    shaftEncodingStraight(-10,8);

}




