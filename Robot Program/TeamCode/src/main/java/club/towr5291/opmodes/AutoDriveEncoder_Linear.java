/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package club.towr5291.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.sixValues;
import club.towr5291.functions.AStarGetPathEnhanced;
import club.towr5291.functions.FileLogger;
import club.towr5291.robotconfig.HardwareDriveMotors;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Ians Auto Drive Linear", group="5291Test")
public class AutoDriveEncoder_Linear extends LinearOpMode
{

    private static final String TAG = "AutoDriveEncoder_Linear";

    /* Declare OpMode members. */
    private HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 560 ;     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    private static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;    // This is < 1.0 if geared UP, Tilerunner is geared up
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     WHEEL_ACTUAL_FUDGE      = 1;        // Fine tuning amount
    private static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
    private static final double     ROBOT_TRACK             = 16.5;     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    private static final double     COUNTS_PER_DEGREE       =  ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;

    //set up the variables for the file logger
    private String startDate;
    private FileLogger fileLogger;
    private int debug = 3;

    private double distanceToEndLeft;
    private double distanceToEndRight;
    private double distanceToEnd;
    private double distanceFromStartLeft;
    private double distanceFromStartRight;
    private double distanceFromStart;

    private int mStartPositionLeft;
    private int mStartPositionRight;

    //set up range sensor variables
    private ModernRoboticsI2cRangeSensor rangeSensorLeft;
    private ModernRoboticsI2cRangeSensor rangeSensorRight;
    private boolean rangeError = false;

    //set up colour sensor variables
    ColorSensor colorSensor;    // Hardware Device Object
    private boolean colorError = false;

    //set up Gyro variables
    private boolean gyroError = false;
    //set up gyro sensor
    private ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    private int gyroXVal, gyroYVal, yroZVal = 0;     // Gyro rate Values
    private int gyroHeading = 0;              // Gyro integrated heading
    private int gyroAngleZ = 0;
    private boolean gyroLastResetState = false;
    private boolean gyroCurResetState  = false;

    //define each state for the step.  Each step should go through some of the states below
    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED,
        STATE_ASTAR_PRE_INIT,
        STATE_ASTAR_INIT,
        STATE_ASTAR_RUNNING,
        STATE_ASTAR_ERROR,
        STATE_ASTAR_COMPLETE
    }

    // set up the variables for the state engine
    private int mCurrentStep = 1;                                                       // Current Step in State Machine.
    private int mCurrentAStarStep = 1;                                                  // Current Step in AStar State Machine.
    private stepState mCurrentStepState;                                                // Current State Machine State.
    private stepState mCurrentDriveState;                                               // Current State of Drive.
    private stepState mCurrentTankTurnState;                                            // Current State of Tank Turn.
    private stepState mCurrentPivotTurnState;                                           // Current State of Pivot Turn.
    private stepState mCurrentRadiusTurnState;                                          // Current State of Radius Turn.

    private double mStepTimeout;
    private double mStepSpeed;
    private String mRobotCommand;
    private boolean mRobotStepComplete;
    private double mStepTurnL;
    private  double mStepTurnR;
    private double mStepDistance;
    private int mStepLeftTarget;
    private int mStepRightTarget;
    private boolean baseStepComplete = false;
    boolean armStepComplete = true;
    static final double INCREMENT  = 0.03;                                              // amount to ramp motor each cycle
    private ElapsedTime mStateTime = new ElapsedTime();                                 // Time into current state

    private HashMap<String,LibraryStateSegAuto> autonomousSteps = new HashMap<String,LibraryStateSegAuto>();

    private HashMap<String,String> powerTable = new HashMap<String,String>();

    private void loadPowerTable ()
    {
        powerTable.put(String.valueOf(0.5), ".2");
        powerTable.put(String.valueOf(1), ".3");
        powerTable.put(String.valueOf(2), ".4");
        powerTable.put(String.valueOf(4), ".5");
        powerTable.put(String.valueOf(6), ".6");
        powerTable.put(String.valueOf(8), ".7");
        powerTable.put(String.valueOf(10), ".8");
        powerTable.put(String.valueOf(12), ".9");
    }

    private void loadStaticSteps ()
    {
        // Valid Commands Angle
        // RT = Right Turn Angle
        // LT = Left Turn Angle
        // LP = Left Pivot Angle
        // RP = Left Pivot Angle
        // LR = Left turn with radius
        // RR = Right turn with radius
        // FW = Drive Forward Distance
        // RV = Drive Backward Distance
        // AS = AutoStar From Current Pos to X,Y
        // FN = Special Function
        //  Red Beacon 1 = (12, 24)
        //  Red Beacon 2 = (12, 84)
        //  Blue Beacon 1 = (24,12)
        //  Blue Beacon 2 = (84,12)
        //                                                              step time, comm,  parm, parm, parm, parm, parm, parm, powe  comp
        //                                                                   out   and    1     2     3     4     5     6     r     lete
        //                                                                   s                                                %
        autonomousSteps.put(String.valueOf(1), new LibraryStateSegAuto (1,   10,  "FW12", 0,    0,    0,    0,    0,    0,    1,    false));
        autonomousSteps.put(String.valueOf(2), new LibraryStateSegAuto (2,   10,  "FW12", 0,    0,    0,    0,    0,    0,    1,    false));
        //autonomousSteps.put(String.valueOf(2), new LibraryStateSegAuto (2,   10,  "AS  ", 120,  110,  0,    12,   24,   270,  0.5,  false));
      //autonomousSteps.put(String.valueOf(3), new LibraryStateSegAuto (3,   10,  "RT90", 0,    0,    0,    0,    0,    0,    0.5,  false));
      //autonomousSteps.put(String.valueOf(4), new LibraryStateSegAuto (4,   10,  "RV6" , 0,    0,    0,    0,    0,    0,    0.3,  false));

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        if (debug >= 1)
        {
            startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
            runtime.reset();
            telemetry.addData("FileLogger: ", runtime.toString());
            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        }

        try {
            // get a reference to a Modern Robotics GyroSensor object.
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

            // calibrate the gyro, this takes a few seconds
            gyro.calibrate();
        } catch (Exception e) {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "Gyro Error " +  e.getMessage());
            }
            gyroError = true;
        }

        LibraryStateSegAuto processingSteps = new LibraryStateSegAuto(0,0,"",0,0,0,0,0,0,0,false);
        AStarGetPathEnhanced getPathValues = new AStarGetPathEnhanced();
        sixValues[] pathValues = new sixValues[1000];
        A0Star a0Star = new A0Star();
        String fieldOutput;
        String strAngleChange;
        int BlueRed;
        HashMap<String,LibraryStateSegAuto> autonomousStepsAStar = new HashMap<>();

        loadStaticSteps();                                                          //load all the steps into the hashmaps
        loadPowerTable();                                                           //load the power table

        /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robotDrive.init(hardwareMap);

        //get a reference to the range sensors
        try {
            rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensorrangeleft");
            rangeSensorRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensorrangeright");
        } catch (Exception e) {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "range Error " +  e.getMessage());
            }
            rangeError = true;
        }

        // get a reference to our ColorSensor object.
        try {
                colorSensor = hardwareMap.colorSensor.get("sensorcolor");
        } catch (Exception e) {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "color Error " +  e.getMessage());
            }
            colorError = true;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTankTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;
        mCurrentPivotTurnState = stepState.STATE_INIT;
        mCurrentRadiusTurnState = stepState.STATE_INIT;

        if (!gyroError) {
            while (!isStopRequested() && gyro.isCalibrating()) {
                sleep(50);
                idle();
            }
        }

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Init Complete");
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "mCurrentStepState:- " + mCurrentStepState + " mCurrentStepState " + mCurrentStepState);
            }
            switch (mCurrentStepState)
            {
                case STATE_INIT:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "About to check if step exists " + mCurrentStep);
                    }
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousSteps.containsKey(String.valueOf(mCurrentStep)))
                    {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Step Exists TRUE " + mCurrentStep + " about to get the values from the step");
                        }
                        processingSteps = autonomousSteps.get(String.valueOf(mCurrentStep));
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Got the values for step " + mCurrentStep + " about to decode");
                        }

                        //decode the step from hashmap
                        initStep(processingSteps);
                    }
                    else  //if no steps left in hashmap then complete
                    {
                        mCurrentStepState = stepState.STATE_FINISHED;
                    }
                }
                break;
                case STATE_START:
                {

                }
                break;
                case STATE_RUNNING:
                {
                    //TankTurnStep();
                    //PivotTurnStep();
                    //RadiusTurnStep();
                    DriveStep();
                    //if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentPivotTurnState == stepState.STATE_COMPLETE) && (mCurrentTankTurnState == stepState.STATE_COMPLETE) && (mCurrentRadiusTurnState == stepState.STATE_COMPLETE))
                        if ((mCurrentDriveState == stepState.STATE_COMPLETE))
                    {
                        mCurrentStepState = stepState.STATE_COMPLETE;
                    }
                }
                break;
                case STATE_PAUSE:
                {

                }
                break;
                case STATE_COMPLETE:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "Step Complete - Current Step:- " + mCurrentStep);
                    }

                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;

                }
                break;
                case STATE_TIMEOUT:
                {
                    setDriveMotorPower(0);
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_FINISHED;
                }
                break;
                case STATE_ERROR:
                {
                    telemetry.addData("STATE", "ERROR WAITING TO FINISH " + mCurrentStep);
                }
                break;
                case STATE_FINISHED:
                {
                    setDriveMotorPower(0);
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "Step Complete - FINISHED");
                    }
                    telemetry.addData("STATE", "FINISHED " + mCurrentStep);
                }
                break;
                case STATE_ASTAR_PRE_INIT:
                {
                    mCurrentAStarStep = 1;                                          //init the Step for AStar
                    //get start point
                    //get end point
                    int startX = (int)processingSteps.getmRobotParm1();
                    int startY = (int)processingSteps.getmRobotParm2();
                    int startZ = (int)processingSteps.getmRobotParm3();
                    int endX = (int)processingSteps.getmRobotParm4();
                    int endY = (int)processingSteps.getmRobotParm5();
                    int endDir = (int)processingSteps.getmRobotParm6();

                    //process path
                    pathValues = getPathValues.findPathAStar(startX, startY, startZ, endX, endY, endDir);  //for enhanced

                    String[][] mapComplete = new String[A0Star.FIELDWIDTH][A0Star.FIELDWIDTH];

                    //write path to logfile to verify path
                    if (startX < startY)
                    {
                        BlueRed = 2;  //RED
                    }
                    else
                    {
                        BlueRed = 1;  //BLUE
                    }

                    for (int y = 0; y < a0Star.fieldLength; y++)
                    {
                        for (int x = 0; x < a0Star.fieldWidth; x++)
                        {
                            if (BlueRed == 2) {
                                if (a0Star.walkableRed[y][x]) {
                                    if ((x == startX) && (y == startY)) {
                                        mapComplete[y][x] = "S";
                                    } else {
                                        mapComplete[y][x] = "1";
                                    }
                                } else {
                                    if ((x == startX) && (y == startY)) {
                                        mapComplete[y][x] = "1";
                                    } else {
                                        mapComplete[y][x] = "0";
                                    }
                                }
                            }
                            else
                            {
                                if (a0Star.walkableBlue[y][x]) {
                                    if ((x == startX) && (y == startY)) {
                                        mapComplete[y][x] = "S";
                                    } else {
                                        mapComplete[y][x] = "1";
                                    }
                                } else {
                                    if ((x == startX) && (y == startY)) {
                                        mapComplete[y][x] = "1";
                                    } else {
                                        mapComplete[y][x] = "0";
                                    }
                                }

                            }
                        }
                    }

                    //plot out path..
                    for (int i = 0; i < pathValues.length; i++)
                    {
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent(TAG,"Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                        }
                        if (((int)pathValues[i].val1 == 0) && ((int)pathValues[i].val3 == 0) && ((int)pathValues[i].val2 == 0) && ((int)pathValues[i].val4 == 0))
                            break;
                        mapComplete[(int)pathValues[i].val3][(int)pathValues[i].val2] = "P";
                        if ((pathValues[i].val2 == startX) && (pathValues[i].val3 == startY))
                        {
                            mapComplete[(int) pathValues[i].val3][(int) pathValues[i].val2] = "S";
                        }
                    }
                    mapComplete[endY][endX] = "E";
                    fieldOutput ="";
                    for (int y = 0; y < a0Star.fieldLength; y++)
                    {
                        for (int x = 0; x < a0Star.fieldWidth; x++)
                        {
                            fieldOutput = "" + fieldOutput + mapComplete[y][x];
                        }
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent(TAG, fieldOutput);
                        }
                        fieldOutput = "";
                    }

                    //load path into hashmap
                    boolean dirChanged;
                    int startStraightSection = 0;
                    int numberOfMoves;
                    int key = 0;

                    for (int i = 0; i < pathValues.length; i++)
                    {

                        if (((int)pathValues[i].val1 == 0) && ((int)pathValues[i].val2 == 0) && ((int)pathValues[i].val3 == 0))
                            break;
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent(TAG,"Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                        }
                        //need to work out if there is a turn
                        //if its the first step, then direction is StartDir
                        if (i == 0) {
                            if (startZ != pathValues[i].val4) {  //need to turn
                                strAngleChange = getAngle(startZ, (int) pathValues[i].val4);
                                autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 5, strAngleChange, 0, 0, 0, 0, 0, 0, 1, false));
                                key++;
                                dirChanged = true;
                            }
                            else
                            {
                                dirChanged = false;    //no change in direction
                            }
                        }
                        else
                        {
                            if (pathValues[i-1].val4 != pathValues[i].val4) {  //need to turn
                                strAngleChange = getAngle((int)pathValues[i-1].val4, (int)pathValues[i].val4);
                                autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   5,  strAngleChange, 0,    0,    0,    0,    0,    0,    1,    false));
                                key++;
                                dirChanged = true;
                            }
                            else
                            {
                                dirChanged = false;    //no change in direction
                            }
                        }
                        //need to add all straight steps together and get a single straight step
                        if (dirChanged)
                        {
                            //need to know the direction we were travelling at to calculate the distance.
                            // If angle is 0, 90, 270, or 180 then distance is 1 inch per i,
                            // if it is 45, 135, 315, 225 then it is root 1 inch per i
                            numberOfMoves = (i - 1) - startStraightSection;
                            //get the direction prior to turning
                            if (i==0)
                            {
                                switch (startZ)
                                {
                                    case 0:
                                    case 90:
                                    case 180:
                                    case 270:
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   5, "FW" + numberOfMoves , 0,    0,    0,    0,    0,    0,    1,    false));
                                        key++;
                                        break;
                                    case 45:
                                    case 135:
                                    case 225:
                                    case 315:
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   5, "FW" + (int)(numberOfMoves * 1.4142 ) , 0,    0,    0,    0,    0,    0,    1,    false));
                                        key++;
                                        break;
                                }
                            }
                            else
                            {
                                switch ((int)pathValues[i-1].val4)
                                {
                                    case 0:
                                    case 90:
                                    case 180:
                                    case 270:
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   5, "FW" + numberOfMoves , 0,    0,    0,    0,    0,    0,    1,    false));
                                        key++;
                                        break;
                                    case 45:
                                    case 135:
                                    case 225:
                                    case 315:
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   5, "FW" + (int)(numberOfMoves * 1.4142 ) , 0,    0,    0,    0,    0,    0,    1,    false));
                                        key++;
                                        break;
                                }
                            }
                            startStraightSection = i;
                        }

                        //find end of sequence
                        if (((int)pathValues[i].val1 == 0) && ((int)pathValues[i].val3 == 0) && ((int)pathValues[i].val2 == 0) && ((int)pathValues[i].val4 == 0))
                        {
                            //last step was the last, need to check if we are facing the right way, if not we need to rotate the robot
                            if (((int)pathValues[i-1].val4) != endDir)
                            {
                                strAngleChange = getAngle((int)pathValues[i-1].val4, endDir);
                                autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key,   5,  strAngleChange, 0,    0,    0,    0,    0,    0,    1,    false));
                                key++;
                            }
                            else
                            {
                                //we are facing the right way, nothing to do
                            }
                        }
                    }
                    mCurrentAStarStep = 0;
                    mCurrentStepState = stepState.STATE_ASTAR_INIT;

                }
                break;
                case STATE_ASTAR_INIT:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "About to check if step exists " + mCurrentAStarStep);
                    }
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousSteps.containsKey(String.valueOf(mCurrentAStarStep)))
                    {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Step Exists TRUE " + mCurrentAStarStep + " about to get the values from the step");
                        }
                        processingSteps = autonomousStepsAStar.get(String.valueOf(mCurrentAStarStep));      //read the step from the hashmap
                        autonomousStepsAStar.remove(String.valueOf(mCurrentAStarStep));                     //remove the step from the hashmap
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Got the values for step " + mCurrentAStarStep + " about to decode and removed them");
                        }

                        //decode the step from hashmap
                        initAStarStep(processingSteps);
                    }
                    else  //if no steps left in hashmap then complete
                    {
                        mCurrentStepState = stepState.STATE_ASTAR_COMPLETE;
                    }
                }
                break;
                case STATE_ASTAR_RUNNING:
                {
                    //move robot according AStar hashmap
                    TankTurnStep();
                    PivotTurnStep();
                    RadiusTurnStep();
                    DriveStep();
                    if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentPivotTurnState == stepState.STATE_COMPLETE) && (mCurrentTankTurnState == stepState.STATE_COMPLETE) && (mCurrentRadiusTurnState == stepState.STATE_COMPLETE))
                    {
                        //increment ASTar Steps Counter
                        mCurrentAStarStep++;
                        mCurrentStepState = stepState.STATE_ASTAR_INIT;
                    }

                }
                break;
                case STATE_ASTAR_ERROR:
                {
                    //do something on error
                }
                break;
                case STATE_ASTAR_COMPLETE:
                {
                    //empty hashmap ready for next AStar processing.
                    //clear AStar step counter ready for next AStar process
                    mCurrentAStarStep = 0;

                    //when complete, keep processing normal step
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "A* Path Completed:- " + mCurrentStep);
                    }

                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;

                }
                break;
            }

            //check timeout vale
            if ((mStateTime.seconds() > mStepTimeout  ) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED)))
            {
                //  Transition to a new state.
                mCurrentStepState = stepState.STATE_TIMEOUT;
            }
        }
        if (debug >= 1)
        {
            if (fileLogger != null)
            {
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }
    }


    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    private void initStep (LibraryStateSegAuto mStateSegAuto) {
        double mRobotParm1;
        double mRobotParm2;
        double mRobotParm3;
        double mRobotParm4;
        double mRobotParm5;
        double mRobotParm6;


        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Starting to Decode Step ");
        }

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();

        mStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mStepSpeed = mStateSegAuto.getmRobotSpeed();
        mRobotCommand = mStateSegAuto.getmRobotCommand();
        mRobotParm1 = mStateSegAuto.getmRobotParm1();
        mRobotParm2 = mStateSegAuto.getmRobotParm2();
        mRobotParm3 = mStateSegAuto.getmRobotParm3();
        mRobotParm4 = mStateSegAuto.getmRobotParm4();
        mRobotParm5 = mStateSegAuto.getmRobotParm5();
        mRobotParm6 = mStateSegAuto.getmRobotParm6();
        mRobotStepComplete = mStateSegAuto.getmRobotStepComplete();

        if (debug >= 3) {
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 0)    :- " + mRobotCommand.substring(0, 0));
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 1)    :- " + mRobotCommand.substring(0, 1));
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 2)    :- " + mRobotCommand.substring(0, 2));
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 3)    :- " + mRobotCommand.substring(0, 3));
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(1)       :- " + mRobotCommand.substring(1));
        }

        mCurrentStepState = stepState.STATE_RUNNING;

        switch (mRobotCommand.substring(0, 2))
        {
            case "LT":
                mCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "RT":
                mCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "LP":
                mCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "RP":
                mCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "LR":  // Left turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "RR":  // Right turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "FW":  // Drive forward a distance in inches and power setting
                mCurrentDriveState = stepState.STATE_INIT;
                break;
            case "AS":  // Plot a course using A* algorithm, accuracy in Parm 1
                mCurrentStepState = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "FN":  //  Run a special Function with Parms

                break;
        }

        if (debug >= 2) {
            fileLogger.writeEvent("initStep()", "Current Step        :- " + mCurrentStep);
            fileLogger.writeEvent("initStep()", "mStepTimeout        :- " + mStepTimeout);
            fileLogger.writeEvent("initStep()", "mStepSpeed          :- " + mStepSpeed);
            fileLogger.writeEvent("initStep()", "mRobotCommand       :- " + mRobotCommand);
            fileLogger.writeEvent("initStep()", "mRobotParm1         :- " + mRobotParm1);
            fileLogger.writeEvent("initStep()", "mRobotParm2         :- " + mRobotParm2);
            fileLogger.writeEvent("initStep()", "mRobotParm3         :- " + mRobotParm3);
            fileLogger.writeEvent("initStep()", "mRobotParm4         :- " + mRobotParm4);
            fileLogger.writeEvent("initStep()", "mRobotParm5         :- " + mRobotParm5);
            fileLogger.writeEvent("initStep()", "mRobotParm6         :- " + mRobotParm6);
            fileLogger.writeEvent("initStep()", "mStepDistance       :- " + mStepDistance);
            fileLogger.writeEvent("initStep()", "mStepTurnL          :- " + mStepTurnL);
            fileLogger.writeEvent("initStep()", "mStepTurnR          :- " + mStepTurnR);
            fileLogger.writeEvent("initStep()", "mRobotStepComplete  :- " + mRobotStepComplete);
        }
    }

    private void initAStarStep (LibraryStateSegAuto mStateSegAuto) {
        double mRobotParm1;
        double mRobotParm2;
        double mRobotParm3;
        double mRobotParm4;
        double mRobotParm5;
        double mRobotParm6;

        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Starting to Decode AStar Step ");
        }

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();

        mStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mStepSpeed = mStateSegAuto.getmRobotSpeed();
        mRobotCommand = mStateSegAuto.getmRobotCommand();
        mRobotParm1 = mStateSegAuto.getmRobotParm1();
        mRobotParm2 = mStateSegAuto.getmRobotParm2();
        mRobotParm3 = mStateSegAuto.getmRobotParm3();
        mRobotParm4 = mStateSegAuto.getmRobotParm4();
        mRobotParm5 = mStateSegAuto.getmRobotParm5();
        mRobotParm6 = mStateSegAuto.getmRobotParm6();
        mRobotStepComplete = mStateSegAuto.getmRobotStepComplete();

        mCurrentStepState = stepState.STATE_ASTAR_RUNNING;

        switch (mRobotCommand.substring(0, 2))
        {
            case "LT":
                mCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "RT":
                mCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "LP":
                mCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "RP":
                mCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "LR":  // Left turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "RR":  // Right turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "FW":  // Drive forward a distance in inches and power setting
                mCurrentDriveState = stepState.STATE_INIT;
                break;
            case "AS":  // Plot a course using A* algorithm, accuracy in Parm 1
                mCurrentStepState = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "FN":  //  Run a special Function with Parms

                break;
        }

        if (debug >= 2) {
            fileLogger.writeEvent("initAStarStep()", "Current Step        :- " + mCurrentStep);
            fileLogger.writeEvent("initAStarStep()", "mStepTimeout        :- " + mStepTimeout);
            fileLogger.writeEvent("initAStarStep()", "mStepSpeed          :- " + mStepSpeed);
            fileLogger.writeEvent("initAStarStep()", "mRobotCommand       :- " + mRobotCommand);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm1         :- " + mRobotParm1);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm2         :- " + mRobotParm2);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm3         :- " + mRobotParm3);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm4         :- " + mRobotParm4);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm5         :- " + mRobotParm5);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm6         :- " + mRobotParm6);
            fileLogger.writeEvent("initAStarStep()", "mStepDistance       :- " + mStepDistance);
            fileLogger.writeEvent("initAStarStep()", "mStepTurnL          :- " + mStepTurnL);
            fileLogger.writeEvent("initAStarStep()", "mStepTurnR          :- " + mStepTurnR);
            fileLogger.writeEvent("initAStarStep()", "mRobotStepComplete  :- " + mRobotStepComplete);
        }
    }

    private void DriveStep()
    {
        double mStepSpeedTemp;

        switch (mCurrentDriveState)
        {
            case STATE_INIT:
            {
                mStepDistance = 0;

                switch (mRobotCommand.substring(0, 2)) {
                    case "FW":  // Drive forward a distance in inches and power setting
                        mStepDistance = Double.parseDouble(mRobotCommand.substring(2));
                        break;
                }

                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningDriveStep()", "mStepDistance   :- " + mStepDistance);
                    fileLogger.writeEvent("runningDriveStep()", "mStepDistance   :- " + mStepDistance);
                }
                // Determine new target position
                mStartPositionLeft = robotDrive.leftMotor1.getCurrentPosition();
                mStartPositionRight = robotDrive.rightMotor1.getCurrentPosition();

                mStepLeftTarget = mStartPositionLeft + (int) (mStepDistance * COUNTS_PER_INCH);
                mStepRightTarget = mStartPositionRight + (int) (mStepDistance * COUNTS_PER_INCH);

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);

                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningDriveStep()", "mStepLeftTarget :- " + mStepLeftTarget);
                    fileLogger.writeEvent("runningDriveStep()", "mStepRightTarget:- " + mStepRightTarget);
                }

                // set motor controller to mode, Turn On RUN_TO_POSITION
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mCurrentDriveState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING:
            {
                mStepSpeedTemp = mStepSpeed;

                // ramp up speed - need to write function to ramp up speed
                distanceFromStartLeft = Math.abs(mStartPositionLeft - robotDrive.leftMotor1.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceFromStartRight = Math.abs(mStartPositionRight - robotDrive.rightMotor1.getCurrentPosition()) / COUNTS_PER_INCH;

                //if moving ramp up
                distanceFromStart = (distanceFromStartLeft + distanceFromStartRight) / 2;

                //determine how close to target we are
                distanceToEndLeft = (mStepLeftTarget - robotDrive.leftMotor1.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceToEndRight = (mStepRightTarget - robotDrive.rightMotor1.getCurrentPosition()) / COUNTS_PER_INCH;

                //if getting close ramp down speed
                distanceToEnd = (distanceToEndLeft + distanceToEndRight) / 2;

                if ((distanceFromStart <= 0.5 ) || (distanceToEnd <= 0.5 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(0.5)));
                }
                else if ((distanceFromStart <= 1 ) || (distanceToEnd <= 1 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(1)));
                }
                else if ((distanceFromStart <= 2 ) || (distanceToEnd <= 2 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(2)));
                }
                else if ((distanceFromStart <= 4 ) || (distanceToEnd <= 4 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(4)));
                }
                else if ((distanceFromStart <= 6 ) || (distanceToEnd <= 6 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(6)));
                }
                else if ((distanceFromStart <= 8 ) || (distanceToEnd <= 8 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(8)));
                }
                else if ((distanceFromStart <= 10 ) || (distanceToEnd <= 10 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(10)));
                }
                else if ((distanceFromStart <= 12 ) || (distanceToEnd <= 12 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(12)));
                }

                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(mStepSpeedTemp * mStepSpeedTemp));

                //if within error margin stop
                if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy())
                {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningDriveStep()", "distanceFromStart " + distanceFromStart + " distanceToEnd " + distanceToEnd + " Power Level " + mStepSpeedTemp + " Running to target   " + mStepLeftTarget + " " + mStepRightTarget + " Running at position " + robotDrive.leftMotor1.getCurrentPosition() + " " + robotDrive.rightMotor1.getCurrentPosition());
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                }
                else
                {
                    // Stop all motion;
                    setDriveMotorPower(0);
                    baseStepComplete = true;
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningDriveStep()", "Complete         ");
                    }
                    mCurrentDriveState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    //--------------------------------------------------------------------------
    //  Execute the state.
    //--------------------------------------------------------------------------

   private void PivotTurnStep ()  //should be same as radius turn with radius of 1/2 robot width, so this function can be deleted once radius turn is completed
    {
        switch (mCurrentPivotTurnState) {
            case STATE_INIT: {
                mStepTurnL = 0;
                mStepTurnR = 0;

                switch (mRobotCommand.substring(0, 2)) {
                    case "LT":
                        mStepTurnL = Double.parseDouble(mRobotCommand.substring(2));
                        mStepTurnR = 0;
                        mCurrentTankTurnState = stepState.STATE_INIT;
                        break;
                    case "RT":
                        mStepTurnL = 0;
                        mStepTurnR = Double.parseDouble(mRobotCommand.substring(2));
                        mCurrentTankTurnState = stepState.STATE_INIT;
                        break;
                }
                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningTurnStep()", "mStepTurnL      :- " + mStepTurnL);
                    fileLogger.writeEvent("runningTurnStep()", "mStepTurnR      :- " + mStepTurnR);
                }
                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()", "Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                    }
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()", "mStepLeftTarget:-  " + mStepLeftTarget);
                    }
                    // pass target position to motor controller
                    robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                    // set motor controller to mode
                    robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveLeftMotorPower(Math.abs(.3));
                }
                else {
                    // Determine new target position
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()", "Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                    }
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()", "mStepRightTarget:- " + mStepRightTarget);
                    }
                    // pass target position to motor controller
                    robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveRightMotorPower(Math.abs(.3));
                }
                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningTurnStep()", "mStepLeftTarget :- " + mStepLeftTarget);
                    fileLogger.writeEvent("runningTurnStep()", "mStepRightTarget:- " + mStepRightTarget);
                }
                mCurrentPivotTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningTurnStep()", "Running         ");
                    fileLogger.writeEvent("runningTurnStep()", "Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                    fileLogger.writeEvent("runningTurnStep()", "Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                }
                if (mStepTurnR == 0) {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()", "Running         ");
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.leftMotor1.isBusy()) {
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        }
                        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()","Running         " );
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.rightMotor1.isBusy()) {
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        }
                        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(0);
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    }
                    mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    private void TankTurnStep ()
    {
        switch (mCurrentTankTurnState) {
            case STATE_INIT: {

                mStepTurnL = 0;
                mStepTurnR = 0;

                switch (mRobotCommand.substring(0, 2)) {
                    case "LT":
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                        }
                        mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() - (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                        }

                        break;
                    case "RT":
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                        }
                        mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() - (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        if (debug >= 3)
                        {
                            fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                        }

                        break;
                }

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);
                // set motor controller to mode
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(.5));

                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );
                }

                mCurrentTankTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                }

                telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());

                switch (mRobotCommand.substring(0, 2)) {
                    case "LT":
                        if (!robotDrive.leftMotor1.isBusy())
                        {
                            if (debug >= 3)
                            {
                                fileLogger.writeEvent("runningTurnStep()","Complete         " );
                            }
                            setDriveMotorPower(0);
                            mCurrentTankTurnState = stepState.STATE_COMPLETE;
                        }
                        break;
                    case "RT":
                        if (!robotDrive.rightMotor1.isBusy())
                        {
                            if (debug >= 3)
                            {
                                fileLogger.writeEvent("runningTurnStep()", "Complete         ");
                            }
                            setDriveMotorPower(0);
                            mCurrentTankTurnState = stepState.STATE_COMPLETE;
                        }
                        break;
                }
            }
            break;
        }
    }

    private void RadiusTurnStep ()
    {
        switch (mCurrentRadiusTurnState) {
            case STATE_INIT: {


                mCurrentRadiusTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING:
            {

                setDriveMotorPower(0);
                if (debug >= 3)
                {
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                }
                mCurrentRadiusTurnState = stepState.STATE_COMPLETE;

            }
            break;
        }
    }

    private String getAngle(int angle1, int angle2)
    {
        switch (angle1)
        {
            case 0:
                switch (angle2)
                {
                    case 45:
                        return "RT45";
                    case 90:
                        return "RT90";
                    case 135:
                        return "RT135";
                    case 180:
                        return "RT180";
                    case 225:
                        return "LT135";
                    case 270:
                        return "LT90";
                    case 315:
                        return "LT45";
                }
                break;
            case 45:
                switch (angle2)
                {
                    case 0:
                        return "LT45";
                    case 90:
                        return "RT45";
                    case 135:
                        return "RT90";
                    case 180:
                        return "RT135";
                    case 225:
                        return "RT180";
                    case 270:
                        return "LT135";
                    case 315:
                        return "LT90";
                }
                break;
            case 90:
                switch (angle2)
                {
                    case 0:
                        return "LT90";
                    case 45:
                        return "LT45";
                    case 135:
                        return "RT45";
                    case 180:
                        return "RT90";
                    case 225:
                        return "RT135";
                    case 270:
                        return "RT180";
                    case 315:
                        return "LT135";
                }
                break;
            case 135:
                switch (angle2)
                {
                    case 0:
                        return "LT135";
                    case 45:
                        return "LT90";
                    case 90:
                        return "LT45";
                    case 180:
                        return "RT45";
                    case 225:
                        return "RT90";
                    case 270:
                        return "RT135";
                    case 315:
                        return "RT180";
                }
                break;
            case 180:
                switch (angle2)
                {
                    case 0:
                        return "LT180";
                    case 45:
                        return "LT135";
                    case 90:
                        return "LT90";
                    case 135:
                        return "LT45";
                    case 225:
                        return "RT45";
                    case 270:
                        return "RT90";
                    case 315:
                        return "RT135";
                }
                break;
            case 225:
                switch (angle2)
                {
                    case 0:
                        return "RT135";
                    case 45:
                        return "LT180";
                    case 90:
                        return "LT135";
                    case 135:
                        return "LT90";
                    case 180:
                        return "LT45";
                    case 270:
                        return "RT45";
                    case 315:
                        return "RT90";
                }
                break;
            case 270:
                switch (angle2)
                {
                    case 0:
                        return "RT90";
                    case 45:
                        return "RT45";
                    case 90:
                        return "LT180";
                    case 135:
                        return "LT135";
                    case 180:
                        return "LT90";
                    case 225:
                        return "LT45";
                    case 315:
                        return "RT135";
                }
                break;
            case 315:
                switch (angle2)
                {
                    case 0:
                        return "RT135";
                    case 45:
                        return "RT90";
                    case 90:
                        return "RT45";
                    case 135:
                        return "LT180";
                    case 180:
                        return "LT135";
                    case 225:
                        return "LT90";
                    case 270:
                        return "LT45";
                }
                break;
        }
        return "ERROR";
    }

    //set the drive motors power, both left and right
    private void setDriveMotorPower (double power) {
        setDriveRightMotorPower(power);
        setDriveLeftMotorPower(power);
    }

    //set the right drive motors power
    private void setDriveRightMotorPower (double power) {
        robotDrive.rightMotor1.setPower(power);
        robotDrive.rightMotor2.setPower(power);
    }

    //set the left motors drive power
    private void setDriveLeftMotorPower (double power) {
        robotDrive.leftMotor1.setPower(power);
        robotDrive.leftMotor2.setPower(power);
    }
}
