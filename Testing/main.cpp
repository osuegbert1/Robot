#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHBattery.h>
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
#define CHECK_HEADING_SPEED 23

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
    int time = TimeNow();
    while(RPS.X() < xCoord-1 || RPS.X() > xCoord+1 && TimeNow() - time < 7){
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
        if(RPS.X() < xCoord - 1){
            double preTurnX = RPS.X();
            shaftEncodingTurnLeft(30,80);
            double xAdjust = preTurnX - RPS.X();
            goToX(xCoord-xAdjust, percent);
            shaftEncodingTurnRight(30,80);
            checkHeading(270);
        }
        else if(RPS.X() > xCoord + 1){
            double preTurnX = RPS.X();
            shaftEncodingTurnRight(30,80);
            double xAdjust = RPS.X() - preTurnX;
            goToX(xCoord+xAdjust, percent);
            shaftEncodingTurnLeft(30,80);
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
        if(RPS.X()<xCoord - 1){
            double preTurnX = RPS.X();
            shaftEncodingTurnRight(30,80);
            double xAdjust = preTurnX - RPS.X();
            goToX(xCoord-xAdjust,percent);
            shaftEncodingTurnLeft(30,80);
            checkHeading(90);
        }
        else if(RPS.X()>xCoord + 1){
            double preTurnX = RPS.X();
            shaftEncodingTurnLeft(30,80);
            double xAdjust = RPS.X() - preTurnX;
            goToX(xCoord+xAdjust,percent);
            shaftEncodingTurnRight(30,80);
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
    drive(25);
    Sleep(1.0);
    stop();
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE+2);
    armServo.SetDegree(30);
    Sleep(.2);
    shaftEncodingStraight(-28,3);
}

void initialize(double &suppliesY, double &suppliesX, double &rampY, double &rampX, double &fuelLightX, double &fuelLightY){
    RPS.InitializeTouchMenu();
    LCD.Clear(BLACK);
    LCD.Write("Voltage: ");
    LCD.Write(Battery.Voltage());
    LCD.WriteLine("/11.5");
    LCD.WriteLine("");
    LCD.Write("Heading: ");
    LCD.WriteLine(RPS.Heading());

    prepForSupplies();
    LCD.WriteLine("ALIGN TO SUPPLIES");
    while(!buttons.MiddlePressed());
    suppliesX = RPS.X();
    suppliesY = RPS.Y();
    LCD.Write("X-Coord: ");
    LCD.WriteLine(RPS.X());
    LCD.Write("Y-Coord: ");
    LCD.WriteLine(RPS.Y());
    while(!buttons.MiddleReleased());
    Sleep(.2);
    armServo.SetDegree(DEFAULT_ARM_ANGLE);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);
    LCD.WriteLine("ALIGN TO RAMP");
    while(!buttons.MiddlePressed());
    rampX = RPS.X();
    rampY = RPS.Y();
    LCD.Write("X-Coord: ");
    LCD.WriteLine(RPS.X());
    LCD.Write("Y-Coord: ");
    LCD.WriteLine(RPS.Y());
    while(!buttons.MiddleReleased());
    Sleep(.2);
    armServo.SetDegree(180);
    LCD.WriteLine("ALIGN TO FUEL LIGHT");
    while(!buttons.MiddlePressed());
    fuelLightX = RPS.X();
    fuelLightY = RPS.Y();
    LCD.Write("X-Coord: ");
    LCD.WriteLine(RPS.X());
    LCD.Write("Y-Coord: ");
    LCD.WriteLine(RPS.Y());
    while(!buttons.MiddleReleased());
    Sleep(.2);
    LCD.SetBackgroundColor(GREEN);
    LCD.WriteLine("PRESS MIDDLE BUTTON TO ARM");
    while(!buttons.MiddlePressed());
    while(!buttons.MiddleReleased());
}

int main(void) {
    LCD.Clear(BLACK);

    armServo.SetMin(898);
    armServo.SetMax(1899);
    clawServo.SetMin(500);
    clawServo.SetMax(1645);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);
    armServo.SetDegree(180);

        RPS.InitializeTouchMenu();
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());


    while(true){
        LCD.WriteLine("Light");
        while(!buttons.MiddlePressed()){
            if(buttons.LeftPressed()){
                LCD.Write("Middle Light: ");
                LCD.WriteLine(CDScell.Value());
                Sleep(.5);
            }
            if(buttons.RightPressed()){
                LCD.Write("Front Light: ");
                LCD.WriteLine(CDScellFront.Value());
                Sleep(.5);
            }
        }
        Sleep(1.0);
        LCD.WriteLine("RPS");
        while(!buttons.MiddlePressed()){
            if(buttons.RightPressed()){
                LCD.Write("X: ");
                LCD.WriteLine(RPS.X());
                LCD.Write("Y: ");
                LCD.WriteLine(RPS.Y());
                LCD.Write("Heading: ");
                LCD.WriteLine(RPS.Heading());
                Sleep(.5);
            }
        }
        Sleep(1.0);
    }


//        while(true){
//            LCD.WriteLine("Drive Straight");
//            waitForMiddlePress();
//            drive(25);
//            waitForMiddlePress();
//            stop();
//            LCD.WriteLine("Turn Left");
//            waitForMiddlePress();
//            turnLeft(30);
//            waitForMiddlePress();
//            stop();
//            LCD.WriteLine("Turn Right");
//            waitForMiddlePress();
//            turnRight(30);
//            waitForMiddlePress();
//            stop();

//        }

    //    while(true){
    //        driveAlongXtoYminus(15,28.2,18);
    //        stop();
    //        waitForMiddlePress();
    //    }

    int armDegree = 23;
    int clawDegree = DEFAULT_CLAW_ANGLE+2;
    while(true){
        LCD.WriteLine("ARM");
        while(!buttons.MiddlePressed()){
            armServo.SetDegree(armDegree);
            if(buttons.LeftPressed()){
                armDegree--;
                LCD.WriteLine(armDegree);
                Sleep(.1);
            }
            if(buttons.RightPressed()){
                armDegree++;
                LCD.WriteLine(armDegree);
                Sleep(.1);
            }
        }
        Sleep(1.0);
        LCD.WriteLine("CLAW");
        while(!buttons.MiddlePressed()){
            clawServo.SetDegree(clawDegree);
            if(buttons.LeftPressed()){
                clawDegree--;
                LCD.WriteLine(clawDegree);
                Sleep(.1);
            }
            if(buttons.RightPressed()){
                clawDegree++;
                LCD.WriteLine(clawDegree);
                Sleep(.1);
            }
        }
        Sleep(1.0);
    }


    clawServo.SetDegree(100);
    armServo.SetDegree(23);

    Sleep(1.0);
    armServo.SetDegree(8);
    shaftEncodingStraight(20,1);
    Sleep(1.0);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE+2);
    armServo.SetDegree(30);
    Sleep(.2);
    shaftEncodingStraight(-20,3);


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


