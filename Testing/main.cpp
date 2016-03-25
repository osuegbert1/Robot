#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <stdlib.h>

DigitalInputPin frontLeftBump(FEHIO::P2_3);
DigitalInputPin frontRightBump(FEHIO::P0_4);
DigitalInputPin backLeftBump(FEHIO::P2_2);
DigitalInputPin backRightBump(FEHIO::P0_3);

AnalogInputPin CDScell(FEHIO::P1_6);
AnalogInputPin CDScellFront(FEHIO::P2_0);

AnalogInputPin leftSensor(FEHIO::P1_4);
AnalogInputPin middleSensor(FEHIO::P1_2);
AnalogInputPin rightSensor(FEHIO::P1_0);

FEHServo armServo(FEHServo::Servo0);
FEHServo clawServo(FEHServo::Servo2);

FEHMotor rightMotor(FEHMotor::Motor1,12.0);
FEHMotor leftMotor(FEHMotor::Motor0,12.0);

DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P0_1);

ButtonBoard buttons(FEHIO::Bank3);

#define FASTER 60
#define SLOWER 25
#define LEFT_THRESHOLD 1.5
#define MIDDLE_THRESHOLD 1.5
#define RIGHT_THRESHOLD 1.5
#define TURN_90_COUNTS 300
#define ENCODING_SPEED 40
#define RED_BLUE_THRESHOLD .59
#define NO_LIGHT_THRESHOLD 1.0
#define RED_LIGHT 1
#define BLUE_LIGHT 2
#define NO_LIGHT 3

void waitForMiddlePress() {
    LCD.WriteLine("Waiting for middle press");
    while(!buttons.MiddlePressed()){}
    while(buttons.MiddlePressed()){}
}


void drive(int percent){
    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(1+percent);
}

void turnLeft(int percent) {
    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(-percent);
}

void turnRight(int percent) {
    rightMotor.SetPercent(-percent);
    leftMotor.SetPercent(percent);
}

void stop() {
    rightMotor.Stop();
    leftMotor.Stop();
    Sleep(.5);
}

/*SHAFT ENCODING*/
void clearCounts() {
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
}

int updateCount() {
    return (leftEncoder.Counts() + rightEncoder.Counts()) / 2.0;
}

void shaftEncodingStraight(int percent, double distance)
{
    int counts = 318*distance/(2.5*3.1415);

    clearCounts();
    drive(percent);

    while(updateCount() < counts){
        LCD.WriteLine(updateCount());
    }

    stop();
}

void shaftEncodingTurnLeft(int percent, int counts)
{
    clearCounts();
    turnLeft(percent);

    while(updateCount() < counts){
        LCD.WriteLine(updateCount());
    }

    stop();
}

void shaftEncodingTurnRight(int percent, int counts)
{
    clearCounts();
    turnRight(percent);

    while(updateCount() < counts){
        LCD.WriteLine(updateCount());
    }

    stop();
}


/*LINE FOLLOWING*/
bool onLine() {
    return (leftSensor.Value() < LEFT_THRESHOLD || middleSensor.Value() < MIDDLE_THRESHOLD || rightSensor.Value() < RIGHT_THRESHOLD);
}

void followLine() {
    if(leftSensor.Value()<LEFT_THRESHOLD){
        rightMotor.SetPercent(FASTER);
        leftMotor.SetPercent(SLOWER);
    }
    if(rightSensor.Value()<RIGHT_THRESHOLD){
        rightMotor.SetPercent(SLOWER);
        leftMotor.SetPercent(FASTER);
    }
    if(middleSensor.Value()<MIDDLE_THRESHOLD){
        rightMotor.SetPercent(SLOWER);
        leftMotor.SetPercent(SLOWER);
    }
}

void alignOnLine() {
    if(leftSensor.Value()<LEFT_THRESHOLD){
        rightMotor.SetPercent(25);
        leftMotor.Stop();
    }
    if(rightSensor.Value()<RIGHT_THRESHOLD){
        leftMotor.SetPercent(25);
        rightMotor.Stop();
    }
    if(middleSensor.Value()<MIDDLE_THRESHOLD && rightSensor.Value()>RIGHT_THRESHOLD && leftSensor.Value()>LEFT_THRESHOLD){
        stop();
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

int senseLightFront() {
    int light;
    if(CDScellFront.Value() < RED_BLUE_THRESHOLD) {
        light = RED_LIGHT;
    }
    else if(CDScellFront.Value() > RED_BLUE_THRESHOLD && CDScellFront.Value() < NO_LIGHT_THRESHOLD) {
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
void checkXPlus(float x_coordinate) //using RPS while robot is in the +x direction
{
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

void checkYMinus(float y_coordinate) //using RPS while robot is in the -y direction
{
    while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5)
    {
        LCD.Write("Y-Coor: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.Y() > y_coordinate)
        {
            drive(12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            drive(-12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
    }
}

void checkYPlus(float y_coordinate) //using RPS while robot is in the +y direction
{
    while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5)
    {
        LCD.Write("Y-Coor: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.Y() > y_coordinate)
        {
            drive(-12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            drive(12);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
    }
}

void checkHeading(float heading) //using RPS
{
    double startPoint = RPS.Heading();

    while(RPS.Heading() < heading - 2.5 || RPS.Heading() > heading + 2.5){
        if (heading - startPoint > 0 ){
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (heading-startPoint < 180){
                turnLeft(22);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            } else {
                turnRight(22);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            }
        } else {
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (startPoint-heading < 180) {
                turnRight(22);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2);
                stop();
            }else {
                turnLeft(22);
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
    stop();
}

void goToY(double yCoord, int percent) {
    drive(percent);
    while(RPS.Y() < yCoord-1 || RPS.Y() > yCoord+1){
        LCD.Write("Y Coord: ");
        LCD.WriteLine(RPS.Y());
    }
    stop();
}

/*BUMP SWITCHES*/
void alignFront() {
    while(frontLeftBump.Value() && frontRightBump.Value());
    stop();
    while(frontLeftBump.Value() || frontRightBump.Value()){
        if(frontLeftBump.Value()){
            leftMotor.SetPercent(40);
        }
        if(frontRightBump.Value()){
            rightMotor.SetPercent(40);
        }
    }
    Sleep(1.5);
    stop();
}

int main(void) {
    //Initialize the screen
//    RPS.InitializeTouchMenu();
//    LCD.Write("Heading: ");
//    LCD.WriteLine(RPS.Heading());
    //waitForMiddlePress();

//    clawServo.TouchCalibrate();
//    waitForMiddlePress();
//    clawServo.TouchCalibrate();
//    waitForMiddlePress();

    armServo.SetMin(898);
    armServo.SetMax(1899);
    clawServo.SetMin(500);
    clawServo.SetMax(1645);
    clawServo.SetDegree(20);
    armServo.SetDegree(10);

    int degree = 20;
    while(true){
        clawServo.SetDegree(degree);
        while(!buttons.MiddlePressed()){
            if(buttons.LeftPressed()){
                degree--;
                LCD.WriteLine(degree);
                Sleep(.1);
            }
            if(buttons.RightPressed()){
                degree++;
                LCD.WriteLine(degree);
                Sleep(.1);
            }
        }
    }

//    while(true){
//        waitForMiddlePress();
//        LCD.Write("OnLine: ");
//        LCD.WriteLine(onLine());
//        LCD.Write("Left: ");
//        LCD.WriteLine(leftSensor.Value());
//        LCD.Write("Middle: ");
//        LCD.WriteLine(middleSensor.Value());
//        LCD.Write("Right: ");
//        LCD.WriteLine(rightSensor.Value());
//        waitForMiddlePress();
//    }

//    while(true){
//        if (onLine()){
//            alignOnLine();
//            LCD.WriteLine("Aligning on Line");
//        }
//        else{
//            drive(10);
//        }
//    }
}


