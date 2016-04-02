#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <stdlib.h>

DigitalInputPin frontLeftBump(FEHIO::P0_6);
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
#define RED_BLUE_THRESHOLD .6
#define NO_LIGHT_THRESHOLD 1.95
#define RED_LIGHT 1
#define BLUE_LIGHT 2
#define NO_LIGHT 3
#define DEFAULT_ARM_ANGLE 90
#define DEFAULT_CLAW_ANGLE 180
#define ARM_TO_SWITCH_ANGLE 60
#define CHECK_HEADING_SPEED 20.5

/*TESTING*/
void waitForMiddlePress() {
    LCD.WriteLine("Waiting for middle press");
    while(!buttons.MiddlePressed()){}
    while(buttons.MiddlePressed()){}
}

/*MISCELANEOUS*/
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

    int time = TimeNow();
    while(updateCount() < counts && TimeNow()- time < 8){
        LCD.WriteLine(updateCount());
    }

    stop();
}

void shaftEncodingTurnLeft(int percent, int counts)
{
    clearCounts();
    turnLeft(percent);

    int time = TimeNow();
    while(updateCount() < counts && TimeNow()- time < 8){
        LCD.WriteLine(updateCount());
    }

    stop();
}

void shaftEncodingTurnRight(int percent, int counts)
{
    clearCounts();
    turnRight(percent);

    int time = TimeNow();
    while(updateCount() < counts && TimeNow()- time < 8){
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
            drive(-18);
            while(RPS.X() < x_coordinate - .5 || RPS.X() > x_coordinate + .5);
            stop();
        }
        else if(RPS.X() < x_coordinate)
        {
            drive(18);
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
            drive(18);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            drive(-18);
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
            drive(-18);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
        else if(RPS.Y() < y_coordinate)
        {
            drive(18);
            while(RPS.Y() < y_coordinate - .5 || RPS.Y() > y_coordinate + .5);
            stop();
        }
    }
}

bool detectStopped(){
    bool stopped = false;
    int counts = updateCount();
    Sleep(.15);
    if(updateCount()-counts == 0){
        stopped = true;
    }
    return stopped;
}

void checkHeading(float heading) //using RPS
{
    double startPoint = RPS.Heading();

    while(RPS.Heading() < heading - 2.5 || RPS.Heading() > heading + 2.5){
        int speed = CHECK_HEADING_SPEED;
        if (heading - startPoint > 0 ){
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (heading-startPoint < 180){
                turnLeft(CHECK_HEADING_SPEED);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2){
                    if(detectStopped()){
                        speed++;
                    } else {
                        speed = CHECK_HEADING_SPEED;
                    }
                    turnLeft(speed);
                }
                stop();
            } else {
                turnRight(CHECK_HEADING_SPEED);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2){
                    if(detectStopped()){
                        speed++;
                    } else {
                        speed = CHECK_HEADING_SPEED;
                    }
                    turnRight(speed);
                }
                stop();
            }
        } else {
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            if (startPoint-heading < 180) {
                turnRight(CHECK_HEADING_SPEED);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2){
                    if(detectStopped()){
                        speed++;
                    } else {
                        speed = CHECK_HEADING_SPEED;
                    }
                    turnRight(speed);
                }
                stop();
            }else {
                turnLeft(CHECK_HEADING_SPEED);
                while(RPS.Heading() < heading - 2 || RPS.Heading() > heading + 2){
                    if(detectStopped()){
                        speed++;
                    } else {
                        speed = CHECK_HEADING_SPEED;
                    }
                    turnLeft(speed);
                }
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

void driveAlongXtoYminus(int percent, double xCoord, double yCoord){
    drive(percent);
    int time = TimeNow();
    while(RPS.Y()> yCoord && TimeNow()-time < 15){
        LCD.Write("X: ");
        LCD.Write(RPS.X());
        LCD.Write("    Y: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.X() < xCoord -.5){
            rightMotor.SetPercent(percent+2*percent);
            Sleep(.25);
            rightMotor.SetPercent(percent);
            leftMotor.SetPercent(percent+2*percent);
            Sleep(.25);
            drive(percent);
            checkHeading(270);
        }
        else if(RPS.X() > xCoord +.5){
            leftMotor.SetPercent(percent+2*percent);
            Sleep(.25);
            leftMotor.SetPercent(percent);
            rightMotor.SetPercent(percent+2*percent);
            Sleep(.25);
            drive(percent);
            checkHeading(270);
        }
        else{
            drive(percent);
        }
    }
    drive(percent);
}

void driveAlongXtoYplus(int percent, double xCoord, double yCoord){
    int time = TimeNow();
    while(RPS.Y() < yCoord && TimeNow() - time < 15){
        LCD.Write("X: ");
        LCD.Write(RPS.X());
        LCD.Write("    Y: ");
        LCD.WriteLine(RPS.Y());
        if(RPS.X()<xCoord-.75){
            shaftEncodingTurnRight(30,80);
            goToX(xCoord,percent);
            shaftEncodingTurnLeft(30,80);
            checkHeading(90);
        }
        else if(RPS.X()>xCoord+.75){
            shaftEncodingTurnLeft(30,50);
            goToX(xCoord,percent);
            shaftEncodingTurnRight(30,50);
            checkHeading(90);
        }
        else{
            drive(percent);
        }
    }
    drive(percent);
}

/*BUMP SWITCHES*/
void alignFront() {
    while(frontLeftBump.Value() && frontRightBump.Value());
    stop();
    while(frontLeftBump.Value() || frontRightBump.Value()){
        if(frontLeftBump.Value()){
            leftMotor.SetPercent(50);
        }
        if(frontRightBump.Value()){
            rightMotor.SetPercent(50);
        }
    }
    Sleep(1.5);
    stop();
}

/*COURSE ACTIONS*/
void prepForSupplies(){
    clawServo.SetDegree(100);
    armServo.SetDegree(23);
}

void pickUpSupplies(){
    armServo.SetDegree(8);
    shaftEncodingStraight(20,1);
    Sleep(.5);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE+2);
    armServo.SetDegree(30);
    Sleep(.2);
    shaftEncodingStraight(-20,3);
}


/*-------------------MAIN--------------------------------------------------------*/
int main(void) {
    LCD.Clear(BLACK);

    armServo.SetMin(898);
    armServo.SetMax(1899);
    clawServo.SetMin(500);
    clawServo.SetMax(1645);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);
    armServo.SetDegree(122);

    RPS.InitializeTouchMenu();
    LCD.Write("Heading: ");
    LCD.WriteLine(RPS.Heading());
    LCD.Write("OnLine: ");
    LCD.WriteLine(onLine());
    int time = TimeNow();
    while(CDScell.Value() > 1.35 && TimeNow() - time <= 35){
        LCD.WriteLine(senseLight());
    }


    /*SWITCHES*/
    double yBeforeSwitch = 18;
    /*WHITE SWITCH*/
    drive(35);
    Sleep(7.0);
    stop();
    shaftEncodingStraight(-30,7);
//    /*RED SWITCH*/
//    shaftEncodingTurnLeft(35,75);
//    checkHeading(105);
//    drive(20);
//    Sleep(5.5);
//    stop();
//    drive(-20);
//    Sleep(2.5);
//    shaftEncodingTurnRight(35,150);
//    checkHeading(75);
//    drive(20);
//    Sleep(5.5);
//    stop();
//    drive(-20);
//    Sleep(2.5);
//    stop();
    //    shaftEncodingStraight(20,3);
    //    shaftEncodingTurnRight(30,50);
    //    checkHeading(90);
    //    driveAlongXtoYplus(20,2,yBeforeSwitch);
    //    checkHeading(90);
//    drive(18);
//    Sleep(1.75);
//    stop();
//    shaftEncodingStraight(-30,11);
//    /*BLUE SWITCH*/
//    shaftEncodingTurnRight(30,100);
//    shaftEncodingStraight(20,4);
//    shaftEncodingTurnLeft(30,100);
//    checkHeading(90);
//    driveAlongXtoYplus(20,9,yBeforeSwitch);
//    drive(18);
//    Sleep(1.75);
//    stop();
//    shaftEncodingStraight(-30,11);
    turnRight(40);
    while(RPS.Heading()>6);
    stop();

    goToX(23.8,30);
    checkXPlus(23.8);
    shaftEncodingTurnRight(30,350);
    checkHeading(275);
    prepForSupplies();
    drive(20);
    while(RPS.Y()>18){
        LCD.Write("X-Coord: ");
        LCD.WriteLine(RPS.X());
    }
    stop();

    pickUpSupplies();


    shaftEncodingTurnRight(30,100);
    shaftEncodingStraight(20,1);

    /*DROP SUPPLIES*/
    armServo.SetDegree(10);
    Sleep(.5);
    clawServo.SetDegree(30);
    Sleep(.5);
    armServo.SetDegree(DEFAULT_ARM_ANGLE);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);

    shaftEncodingTurnRight(30,500);
    checkHeading(90);

    driveAlongXtoYplus(20,28.3,18);
    stop();
    //waitForMiddlePress();

    rightMotor.SetPercent(60);
    leftMotor.SetPercent(61);
    clearCounts();
    while(updateCount()<1300 && RPS.Y()<37);
    drive(25);
    while(RPS.Y()<37);
    stop();
    armServo.SetDegree(180);

    //waitForMiddlePress();


    driveAlongXtoYplus(20,29.3,56.7);
    //    drive(20);
    //    rightMotor.SetPercent(20);
    //    leftMotor.SetPercent(21.3);
    //    while(RPS.Y()<57.8){
    //        LCD.Write("X-Coord: ");
    //        LCD.WriteLine(RPS.X());
    //    }
    stop();
    //    turnLeft(20);
    //    Sleep(.25);
    //    stop();
    checkHeading(90);

    /*PRESS FUEL BUTTON*/
//    if(senseLightFront() == RED_LIGHT){
//        LCD.SetBackgroundColor(RED);
//        //waitForMiddlePress();
//        shaftEncodingStraight(-20,9);
//        checkHeading(90);
//        armServo.SetDegree(5);
//        clawServo.SetDegree(34.5);
//        Sleep(1.0);
//        drive(22);
//        Sleep(1.6);
//        stop();
//        Sleep(5.5);
//    } else {
        LCD.SetBackgroundColor(BLUE);
        //waitForMiddlePress();
        drive(25);
        Sleep(1.5);
        stop();
        Sleep(5.5);
        shaftEncodingStraight(-20,5);
    //}

    armServo.SetDegree(DEFAULT_ARM_ANGLE);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);

    shaftEncodingStraight(-20,9);
    //    armServo.SetDegree(150);
    //    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);
    //    shaftEncodingTurnLeft(35,360);
    //    checkHeading(180);
    //    drive(35);
    //    alignFront();
    //    shaftEncodingStraight(-25,6);
    //    shaftEncodingTurnLeft(25,335);
    //    drive(25);
    //    alignFront();

    //    shaftEncodingStraight(-20,5);
    //    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);
    //    armServo.SetDegree(ARM_TO_SWITCH_ANGLE);
    //    shaftEncodingStraight(20,3);
    //    armServo.SetDegree(150);
    //    shaftEncodingTurnRight(25,80);
    //waitForMiddlePress();

    LCD.Clear(BLACK);

    //waitForMiddlePress();
    shaftEncodingTurnLeft(30,400);
    shaftEncodingStraight(20,1.5);
    shaftEncodingTurnLeft(30,380);
    checkHeading(269);
    drive(30);
    while(RPS.Y() > 22);
    checkYMinus(22);
    stop();
    shaftEncodingTurnRight(25,200);
    checkHeading(200);

    //waitForMiddlePress();
    drive(25);

}

