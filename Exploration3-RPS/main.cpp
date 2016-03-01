//#include <FEHLCD.h>
//#include <FEHIO.h>
//#include <FEHUtility.h>
//#include <FEHRPS.h>

/* RPS Test*/
//int main(void)
//{
//    float touch_x, touch_y;

//    //Call this function to initialize the RPS to a course
//    RPS.InitializeTouchMenu();

//    //Wait for touchscreen to be pressed
//    LCD.WriteLine("Press Screen to Start");
//    while(!LCD.Touch(&touch_x, &touch_y));

//    LCD.Clear();

//    //Write initial screen info
//    LCD.WriteRC("RPS Test Program",0,0);
//    LCD.WriteRC("X Position:",2,0);
//    LCD.WriteRC("Y Position:",3,0);
//    LCD.WriteRC("   Heading:",4,0);

//    LCD.WriteRC("  R Switch:",6,0);
//    LCD.WriteRC("  W Switch:",7,0);
//    LCD.WriteRC("  B Switch:",8,0);

//    while( true )
//    {
//        LCD.WriteRC(RPS.X(),2,12); //update the x coordinate
//        LCD.WriteRC(RPS.Y(),3,12); //update the y coordinate
//        LCD.WriteRC(RPS.Heading(),4,12); //update the heading
//        if(RPS.RedSwitchDirection() == 1) LCD.WriteRC("Forward ",6,12);
//        else LCD.WriteRC("Backward",6,12);
//        if(RPS.WhiteSwitchDirection() == 1) LCD.WriteRC("Forward ",7,12);
//        else LCD.WriteRC("Backward",7,12);
//        if(RPS.BlueSwitchDirection() == 1) LCD.WriteRC("Forward ",8,12);
//        else LCD.WriteRC("Backward",8,12);
//        Sleep(10); //wait for a 10ms to avoid updating the screen too quickly
//    }

//    //we will never get here because of the infinite while loop
//    return 0;
//}



/*Skeleton Program*/
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <math.h>
#include <FEHSD.h>


DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_0);
FEHMotor right_motor(FEHMotor::Motor1, 12.0);
FEHMotor left_motor(FEHMotor::Motor0, 12.0);

void stop() {
    right_motor.Stop();
    left_motor.Stop();
    Sleep(.25);
}

void drive(int percent) {
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
}

void turnLeft(int percent){
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-1*percent);
}

void turnRight(int percent){
    right_motor.SetPercent(-1*percent);
    left_motor.SetPercent(percent);
}

void move_forward(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void turn_right(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void turn_left(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

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
            }
            else {
                turnRight(13);
                Sleep(.2);
                stop();
            }
        }else {
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

int main(void)
{
    float touch_x,touch_y;

    RPS.InitializeTouchMenu();

    LCD.WriteLine("RPS & Data Logging Test");
    LCD.WriteLine("Press Screen To Start");
    while(!LCD.Touch(&touch_x,&touch_y)); //Wait for touchscreen press

    //STUDENT CODE HERE
    SD.OpenLog();

    SD.Printf("%f, %f\n", RPS.X(), RPS.Y());
    move_forward(20,243);
    check_y_plus(16.6);
    SD.Printf("%f, %f\n", RPS.X(), RPS.Y());
    turn_right(15,150);
    check_heading(0);
    move_forward(20,850);
    check_x_plus(27.8);
    SD.Printf("%f, %f\n", RPS.X(), RPS.Y());
    turn_right(15,150);
    check_heading(270);
    move_forward(20,240);
    check_y_minus(13.5);
    SD.Printf("%f, %f\n", RPS.X(), RPS.Y());


    SD.CloseLog();
    return 0;
}


/*SD Test*/
//#include <FEHLCD.h>
//#include <FEHIO.h>
//#include <FEHUtility.h>
//#include <FEHRPS.h>
//#include <FEHSD.h>

//int main(void)
//{
//    float touch_x, touch_y, time_start;
//    bool heart_beat = true;

//    //Call this function to initialize the RPS to a course
//    RPS.InitializeTouchMenu();
//    //Create New Log
//    SD.OpenLog();

//    //Wait for touchscreen to be pressed
//    LCD.WriteLine("Press Screen to Start");
//    while(!LCD.Touch(&touch_x, &touch_y));
//    LCD.WriteLine("Begin!");
//    //Get starting time
//    time_start = TimeNow();
//    //Continue in loop for 30 seconds
//    while(TimeNow() - time_start < 30.)
//    {
//        //Print x and y coordinates to log
//        if(heart_beat) LCD.WriteRC("<3",4,0);
//        else LCD.WriteRC("  ",4,0);
//        SD.Printf("%f, %f\n", RPS.X(), RPS.Y());
//        //Update every 10ms
//        Sleep(10);
//        heart_beat = !heart_beat;
//    }
//    //Close log
//    SD.CloseLog();
//    LCD.WriteLine("RPS Data Log Complete");
//    return 0;
//}
