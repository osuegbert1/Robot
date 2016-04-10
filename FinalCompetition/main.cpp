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
#define RED_BLUE_THRESHOLD .7
#define NO_LIGHT_THRESHOLD 1.85
#define RED_LIGHT 1
#define BLUE_LIGHT 2
#define NO_LIGHT 3
#define DEFAULT_ARM_ANGLE 90
#define DEFAULT_CLAW_ANGLE 180
#define ARM_TO_SWITCH_ANGLE 60
#define CHECK_HEADING_SPEED 21.5

/*TESTING*/
void waitForMiddlePress() {
    LCD.WriteLine("Waiting for middle press");
    while(!buttons.MiddlePressed()){}
    while(buttons.MiddlePressed()){}
}

/*CORE DRIVE FUNCTIONS*/
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
    while(RPS.Heading() < heading - 2.7 || RPS.Heading() > heading + 2.7){
        int speed = CHECK_HEADING_SPEED;
        if ((heading - startPoint > 0 && heading - startPoint < 180) || (heading - startPoint <=0 && startPoint - heading >= 180)){
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            turnLeft(CHECK_HEADING_SPEED);
            while(RPS.Heading() < heading - 2.5 || RPS.Heading() > heading + 2.5){
                if(detectStopped()){
                    speed++;
                } else {
                    speed = CHECK_HEADING_SPEED;
                }
                turnLeft(speed);
            }
            stop();
        } else {
            LCD.Write("Heading: ");
            LCD.WriteLine(RPS.Heading());
            turnRight(CHECK_HEADING_SPEED);
            int counts = 0;
            while((RPS.Heading() < heading - 2.5 || RPS.Heading() > heading + 2.5) && counts < 600){
                if(detectStopped()){
                    speed++;
                } else {
                    speed = CHECK_HEADING_SPEED;
                }
                turnRight(speed);
                counts++;
            }
            stop();
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
int numberOfBumpsPressed(){
    int bumpsPressed = 0;
    if(!frontLeftBump.Value() & !frontRightBump.Value()){
        bumpsPressed = 2;
    }
    else if(!frontLeftBump.Value() || !frontRightBump.Value()){
        bumpsPressed = 1;
    }
    return bumpsPressed;
}

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
    Sleep(.7);
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


/*---------------------------------------------------------MAIN---------------------------------------------------------*/
int main(void) {
    armServo.SetMin(898);
    armServo.SetMax(1899);
    clawServo.SetMin(500);
    clawServo.SetMax(1645);

    double suppliesY,suppliesX,rampY,rampX,fuelLightX,fuelLightY;
    initialize(suppliesY,suppliesX,rampY,rampX,fuelLightX,fuelLightY);

    LCD.Clear(BLACK);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);
    armServo.SetDegree(122);

    int time = TimeNow();
    while(CDScell.Value() > 1.5 && TimeNow() - time < 33){
        LCD.WriteLine(senseLight());
    }


    /*SWITCHES*/
    /*WHITE SWITCH*/
    drive(35);
    while(numberOfBumpsPressed() == 0);
    Sleep(.7);
    stop();
    shaftEncodingStraight(-30,7);

    /*SUPPLIES*/
    turnRight(40);
    while(RPS.Heading()>6);
    stop();

    goToX(23.8,30);
    checkXPlus(23.8);

    shaftEncodingTurnRight(40,350);
    checkHeading(270);

    prepForSupplies();
    driveAlongXtoYminus(25,suppliesX,suppliesY);
    stop();

    pickUpSupplies();

    shaftEncodingTurnRight(40,100);
    shaftEncodingStraight(20,1);

    /*DROP SUPPLIES*/
    armServo.SetDegree(8);
    clawServo.SetDegree(30);
    Sleep(.8);
    armServo.SetDegree(DEFAULT_ARM_ANGLE);
    Sleep(.1);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);

    /*RAMP*/
    shaftEncodingTurnRight(40,550);
    checkHeading(90);

    driveAlongXtoYplus(25,rampX,rampY);
    stop();

    drive(73);
    clearCounts();
    while(updateCount()<1200 && RPS.Y()<37);
    drive(30);
    while(RPS.Y()<37);
    stop();
    armServo.SetDegree(180);

    /*FUEL LIGHT*/
    driveAlongXtoYplus(25,fuelLightX,fuelLightY);
    stop();
    checkHeading(90);
    checkYPlus(fuelLightY);

    /*PRESS FUEL BUTTON*/
    if(senseLightFront() == RED_LIGHT){
        LCD.Clear(RED);
        shaftEncodingStraight(-28,9);
        checkHeading(90);
        armServo.SetDegree(5);
        clawServo.SetDegree(34.5);
        Sleep(1.0);
        drive(25);
        Sleep(1.6);
        stop();
        Sleep(5.5);
    } else {
        LCD.Clear(BLUE);
        drive(25);
        Sleep(1.5);
        stop();
        Sleep(5.5);
        shaftEncodingStraight(-20,5);
    }

    armServo.SetDegree(DEFAULT_ARM_ANGLE);
    clawServo.SetDegree(DEFAULT_CLAW_ANGLE);

    LCD.Clear(BLACK);
    shaftEncodingStraight(-30,9);

    /*GO DOWN RAMP*/
    shaftEncodingTurnLeft(40,400);
    shaftEncodingStraight(30,.8);
    shaftEncodingTurnLeft(40,320);
    checkHeading(270);
    armServo.SetDegree(240);
    drive(40);
    while(RPS.Y() > 22){
        if(numberOfBumpsPressed() > 0){
            stop();
            shaftEncodingStraight(-30,6);
            shaftEncodingTurnLeft(40,100);
            shaftEncodingStraight(30,3.5);
            shaftEncodingTurnRight(40,100);
            checkHeading(270);
        }
    }
    checkYMinus(22);
    stop();

    /*FINAL BUTTON*/
    shaftEncodingTurnRight(35,250);
    checkHeading(205);

    drive(45);

    /*FAILSAFE FOR END BUTTON*/
    while(numberOfBumpsPressed() == 0);
    Sleep(.75);
    shaftEncodingStraight(-40,5);
    shaftEncodingTurnRight(40,50);
    drive(45);
}

