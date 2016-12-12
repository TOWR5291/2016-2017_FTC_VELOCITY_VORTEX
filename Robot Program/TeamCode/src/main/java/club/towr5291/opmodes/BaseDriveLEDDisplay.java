package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryStateSegAuto;
import club.towr5291.robotconfig.HardwareArmMotors;
import club.towr5291.robotconfig.HardwareDriveMotors;


@TeleOp(name = "LED Show", group = "5291")
//@Disabled
public class BaseDriveLEDDisplay extends LinearOpMode {
    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "AutoDriveTeam5291";

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String alliancePosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;

    // Declare OpMode members.
    private HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareArmMotors armDrive   = new HardwareArmMotors();   // Use a Pushbot's hardware

    //variable for the counter when loading steps into the hashap, each step must have a unique step ID
    private int loadStep = 1;

    private ElapsedTime     runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

     //define each state for the step.  Each step should go through some of the states below
    // set up the variables for the state engine
    private int mintCurrentStep = 1;                                                       // Current Step in State Machine.
    private stepState mintCurrentStepState;                                                // Current State Machine State.
    private stepState mintCurrentDriveState;                                               // Current State of Drive.
    private stepState mintCurrentDriveHeadingState;                                        // Current State of Drive Heading.
    private stepState mintCurrentTankTurnState;                                            // Current State of Tank Turn.
    private stepState mintCurrentPivotTurnState;                                           // Current State of Pivot Turn.
    private stepState mintCurrentRadiusTurnState;                                          // Current State of Radius Turn.
    private stepState mintCurrentDelayState;                                               // Current State of Delay (robot doing nothing)
    private stepState mintCurrentVuforiaState;                                             // Current State of Vuforia Localisation
    private stepState mintCurrentVuforiaMoveState;                                         // Current State of Vuforia Move
    private stepState mintCurrentVuforiaTurnState;                                         // Current State of Vuforia Turn
    private stepState mintCurrentBeaconColourState;                                        // Current State of Beacon Colour
    private stepState mintCurrentAttackBeaconState;                                        // Current State of Attack Beacon
    private stepState mint5291CurrentShootParticleState;                                   // Current State of Shooting Ball in Vortex
    private stepState mint11231CurrentShootParticleState;                                  // Current State of Shooting Ball in Vortex
    private stepState mintCurrentTankTurnGyroHeadingState;                                 // Current State of Tank Turn using Gyro
    private stepState mintCurrentTankTurnGyroBasicState;                                   // Current State of Tank Turn using Gyro Basic
    private stepState mint5291CurrentLineFindState;                                        // Current State of the special function to move forward until line is found
    private stepState mint5291GyroTurnEncoderState;                                        // Current State of the Turn function that take the Gyro as an initial heading


    private LEDState mint5291LEDStatus;                                                   // Flash the LED based on teh status

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
    private enum LEDState {
        STATE_ERROR,
        STATE_TEAM,
        STATE_MOVING,
        STATE_BEACON,
        STATE_SUCCESS,
        STATE_FINISHED
    }

    private boolean readyToCapture = false;

    //variable for the state engine, declared here so they are accessible throughout the entire opmode with having to pass them through each function
    private int mintStartPositionLeft1;                      //Left Motor 1  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionLeft2;                      //Left Motor 2  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight1;                     //Right Motor 1 - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight2;                     //Right Motor 2 - start position of the robot in inches, starts from 0 to the end
    private int mintStepLeftTarget1;                         //Left Motor 1   - encoder target position
    private int mintStepLeftTarget2;                         //Left Motor 2   - encoder target position
    private int mintStepRightTarget1;                        //Right Motor 1  - encoder target position
    private int mintStepRightTarget2;                        //Right Motor 2  - encoder target position
    private double mdblStepTimeout;                          //Timeout value ofthe step, the step will abort if the timeout is reached
    private double mdblStepSpeed;                            //When a move command is executed this is the speed the motors will run at
    private String mdblRobotCommand;                         //The command the robot will execute, such as move forward, turn right etc
    private double mdblRobotParm1;                           //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm2;                           //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm3;                           //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm4;                           //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm5;                           //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm6;                           //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private boolean mdblRobotStepComplete;                   //flag to write back to the step if we finish the step, currently not used
    private double mdblStepTurnL;                            //used when decoding the step, this will indicate if the robot is turning left
    private double mdblStepTurnR;                            //used when decoding the step, this will indicate if the robot is turning right
    private double mdblRobotTurnAngle;                       //used to determine angle the robot will turn
    private double mdblStepDistance;                         //used when decoding the step, this will indicate how far the robot is to move in inches
    //private double mdblArcLengthRadiusTurn;                  //used to calculate the arc length when doing a radius turn
    private double mdblArcLengthRadiusTurnInner;             //used to calculate the arc length when doing a radius turn
    private double mdblArcLengthRadiusTurnOuter;             //used to calculate the arc length when doing a radius turn
    private double mdblSpeedOuter;                           //used to calculate the speed of the outer wheels during the turn
    private double mdblSpeedInner;                           //used to calculate the speed of the inner wheels during the turn
    private boolean mblnParallel;                            //used to determine if next step will run in parallel - at same time
    private boolean mblnRobotLastPos;                        //used to determine if next step will run from end of last step or from encoder position
    private int mintLastEncoderDestinationLeft1;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationLeft2;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight1;            //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight2;            //used to store the encoder destination from current Step
    private boolean mblnNextStepLastPos;                     //used to detect using encoders or previous calc'd position
    private int mintStepDelay;                               //used when decoding the step, this will indicate how long the delay is on ms.
    private boolean mblnDisableVisionProcessing = false;     //used when moving to disable vision to allow faster speed reading encoders.
    private boolean mblnBaseStepComplete = false;
    private boolean mblnArmStepComplete = true;
    private ElapsedTime mStateTime = new ElapsedTime();     // Time into current state, used for the timeout

    //hashmap for the steps to be stored in.  A Hashmap is like a fancy array
    private HashMap<String,LibraryStateSegAuto> autonomousSteps = new HashMap<String,LibraryStateSegAuto>();


    //LED Strips
    DeviceInterfaceModule dim;                  // Device Object
    final int GREEN1_LED_CHANNEL = 0;
    final int RED1_LED_CHANNEL = 1;
    final int BLUE1_LED_CHANNEL = 2;
    final int GREEN2_LED_CHANNEL = 3;
    final int RED2_LED_CHANNEL = 4;
    final int BLUE2_LED_CHANNEL = 5;
    final boolean LedOn = false;
    final boolean LedOff = true;
    private Constants.BeaconColours mColour;
    private double mdblLastOn;
    private double mdblLastOff;
    private boolean mblnLEDON;
    private int mintCounts = 0;


    //**********************************************************
    //   _____ ___   ___  __
    //  | ____|__ \ / _ \/_ |
    //  | |__    ) | (_) || |
    //  |___ \  / / \__, || |
    //     _) |/ /_   / / | |
    //  |____/|____| /_/  |_|
    //
    //  the code below is for team 5291 autonomous
    //**********************************************************
    private void loadStaticSteps5291 ()
    {
        //           time, comm,  paral, lastp  parm, parm, parm, parm, parm, parm, powe
        //           out   and                   1     2     3     4     5     6     r
        //            s                                                              %
        loadSteps(40, "DEL30000", false, false,   0,    0,    0,    0,    0,    0,    0);
    }

    private void loadSteps(int timeOut, String command, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6, double power)
    {
        autonomousSteps.put(String.valueOf(loadStep), new LibraryStateSegAuto (loadStep, timeOut, command, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6, power, false));
        loadStep++;
    }

    private void insertSteps(int timeOut, String command, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6, double power, int insertlocation)
    {
        Log.d("insertSteps", "insert location " + insertlocation + " timout " + timeOut + " command " + command + " parallel " + parallel + " lastPos " + lastPos + " parm1 " + parm1 + " parm2 " + parm2 + " parm3 " + parm3 + " parm4 " + parm4 + " parm5 " + parm5 + " parm6 " + parm6 + " power " + power);
        HashMap<String,LibraryStateSegAuto> autonomousStepsTemp = new HashMap<String,LibraryStateSegAuto>();
        LibraryStateSegAuto processingStepsTemp;

        //move all the steps from current step to a temp location
        for (int loop = insertlocation; loop < loadStep; loop++)
        {
            processingStepsTemp = autonomousSteps.get(String.valueOf(loop));
            Log.d("insertSteps", "Reading all the next steps " + loop + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand() );
            autonomousStepsTemp.put(String.valueOf(loop), autonomousSteps.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "All steps loaded to a temp hasmap");

        //insert the step we want
        autonomousSteps.put(String.valueOf(insertlocation), new LibraryStateSegAuto (loadStep, timeOut, command, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6, power, false));
        Log.d("insertSteps", "Inserted New step");

        //move all the other steps back into the sequence
        for (int loop = insertlocation; loop < loadStep; loop++)
        {
            processingStepsTemp = autonomousStepsTemp.get(String.valueOf(loop));
            Log.d("insertSteps", "adding these steps back steps " + (loop + 1) + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand() );
            autonomousSteps.put(String.valueOf(loop + 1), autonomousStepsTemp.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "Re added all the previous steps");

        //increment the step counter as we inserted a new step
        loadStep++;
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Init1     ",  "Starting!");
        telemetry.update();
        //
        // get a reference to a Modern Robotics DIM, and IO channels.
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping

        dim.setDigitalChannelMode(GREEN1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE1_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(GREEN2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(RED2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel
        dim.setDigitalChannelMode(BLUE2_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT); // Set the direction of each channel

        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);

        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        alliancePosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");

        LibraryStateSegAuto processingSteps = new LibraryStateSegAuto(0,0,"",false,false,0,0,0,0,0,0,0,false);


        if (debug >= 1)
        {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
            Log.d(TAG, "Log Started");
            runtime.reset();
            telemetry.addData("FileLogger: ", runtime.toString());
            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        }


        mintCurrentStepState = stepState.STATE_INIT;
        mintCurrentTankTurnState = stepState.STATE_COMPLETE;
        mintCurrentDriveState = stepState.STATE_COMPLETE;
        mintCurrentDriveHeadingState = stepState.STATE_COMPLETE;
        mintCurrentPivotTurnState = stepState.STATE_COMPLETE;
        mintCurrentRadiusTurnState = stepState.STATE_COMPLETE;
        mintCurrentDelayState = stepState.STATE_COMPLETE;
        mintCurrentVuforiaState = stepState.STATE_COMPLETE;
        mintCurrentVuforiaMoveState = stepState.STATE_COMPLETE;
        mintCurrentVuforiaTurnState = stepState.STATE_COMPLETE;
        mintCurrentAttackBeaconState = stepState.STATE_COMPLETE;
        mintCurrentBeaconColourState = stepState.STATE_COMPLETE;
        mint5291CurrentShootParticleState = stepState.STATE_COMPLETE;
        mint11231CurrentShootParticleState = stepState.STATE_COMPLETE;
        mintCurrentTankTurnGyroHeadingState = stepState.STATE_COMPLETE;
        mintCurrentTankTurnGyroBasicState = stepState.STATE_COMPLETE;
        mint5291CurrentLineFindState = stepState.STATE_COMPLETE;
        mint5291GyroTurnEncoderState = stepState.STATE_COMPLETE;

        mint5291LEDStatus = LEDState.STATE_FINISHED;
        mColour = Constants.BeaconColours.BEACON_RED_BLUE;

        mblnNextStepLastPos = false;

        loadStaticSteps5291();

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Init Complete");
            Log.d(TAG, "Init Complete");
        }

        //show options on the driver station phone
        telemetry.addData("Init11     ",  "Complete");
        telemetry.addData("Team #     ",  teamNumber);
        telemetry.addData("Alliance   ",  allianceColor);
        telemetry.addData("Start Pos  ",  alliancePosition);
        telemetry.addData("Start Del  ",  delay);
        telemetry.addData("# Beacons  ",  numBeacons);
        telemetry.addData("Robot      ",  robotConfig);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //the main loop.  this is where the action happens
        while (opModeIsActive())
        {

            switch (mintCurrentStepState)
            {
                case STATE_INIT:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "mintCurrentStepState:- " + mintCurrentStepState + " mintCurrentStepState " + mintCurrentStepState);
                        fileLogger.writeEvent(TAG, "About to check if step exists " + mintCurrentStep);
                        Log.d(TAG, "mintCurrentStepState:- " + mintCurrentStepState + " mintCurrentStepState " + mintCurrentStepState);
                        Log.d(TAG, "About to check if step exists " + mintCurrentStep);
                    }
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousSteps.containsKey(String.valueOf(mintCurrentStep)))
                    {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Step Exists TRUE " + mintCurrentStep + " about to get the values from the step");
                            Log.d(TAG, "Step Exists TRUE " + mintCurrentStep + " about to get the values from the step");
                        }
                        processingSteps = autonomousSteps.get(String.valueOf(mintCurrentStep));
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Got the values for step " + mintCurrentStep + " about to decode");
                            Log.d(TAG, "Got the values for step " + mintCurrentStep + " about to decode");
                        }
                        //decode the step from hashmap
                        initStep(processingSteps);
                    }
                    else  //if no steps left in hashmap then complete
                    {
                        mintCurrentStepState = stepState.STATE_FINISHED;
                    }
                }
                break;
                case STATE_START:
                {

                }
                break;
                case STATE_RUNNING:
                {
                    DelayStep();


                    if ((mintCurrentDelayState == stepState.STATE_COMPLETE) &&
                            (mintCurrentBeaconColourState == stepState.STATE_COMPLETE) &&
                            (mintCurrentAttackBeaconState == stepState.STATE_COMPLETE) &&
                            (mintCurrentVuforiaTurnState == stepState.STATE_COMPLETE) &&
                            (mintCurrentVuforiaState == stepState.STATE_COMPLETE) &&
                            (mintCurrentVuforiaMoveState == stepState.STATE_COMPLETE)  &&
                            (mintCurrentDriveState == stepState.STATE_COMPLETE) &&
                            (mintCurrentDriveHeadingState == stepState.STATE_COMPLETE) &&
                            (mintCurrentPivotTurnState == stepState.STATE_COMPLETE) &&
                            (mintCurrentTankTurnState == stepState.STATE_COMPLETE) &&
                            (mint5291CurrentShootParticleState == stepState.STATE_COMPLETE) &&
                            (mint11231CurrentShootParticleState == stepState.STATE_COMPLETE) &&
                            (mint5291CurrentLineFindState == stepState.STATE_COMPLETE) &&
                            (mint5291GyroTurnEncoderState == stepState.STATE_COMPLETE) &&
                            (mintCurrentTankTurnGyroHeadingState == stepState.STATE_COMPLETE) &&
                            (mintCurrentTankTurnGyroBasicState == stepState.STATE_COMPLETE) &&
                            (mintCurrentRadiusTurnState == stepState.STATE_COMPLETE))
                    {
                        mintCurrentStepState = stepState.STATE_COMPLETE;
                    }

                    if (mblnParallel) {
                        //mark this step as complete and do next one, the current step should continue to run.  Not all steps are compatible with being run in parallel
                        // like drive steps, turns etc
                        // Drive forward and shoot
                        // Drive forward and detect beacon
                        // are examples of when parallel steps should be run
                        // errors will occur if other combinations are run
                        mintCurrentStepState = stepState.STATE_COMPLETE;
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
                        fileLogger.writeEvent(TAG, "Step Complete - Current Step:- " + mintCurrentStep);
                        Log.d(TAG, "Step Complete - Current Step:- " + mintCurrentStep);
                    }

                    //  Transition to a new state and next step.
                    mintCurrentStep++;
                    mintCurrentStepState = stepState.STATE_INIT;
                }
                break;
                case STATE_TIMEOUT:
                {

                    //  Transition to a new state.
                    mintCurrentStepState = stepState.STATE_FINISHED;
                }
                break;
                case STATE_ERROR:
                {
                    telemetry.addData("STATE", "ERROR WAITING TO FINISH " + mintCurrentStep);
                }
                break;
                case STATE_FINISHED:
                {

                    telemetry.addData("STATE", "FINISHED " + mintCurrentStep);
                    if (debug >= 1)
                    {
                        if (fileLogger != null)
                        {
                            fileLogger.writeEvent(TAG, "Step FINISHED - FINISHED");
                            fileLogger.writeEvent(TAG, "Stopped");
                            Log.d(TAG, "FileLogger Stopped");
                            fileLogger.close();
                            fileLogger = null;
                        }
                    }
                }
                break;
            }

            //check timeout vale
            if ((mStateTime.seconds() > mdblStepTimeout  ) && ((mintCurrentStepState != stepState.STATE_ERROR) && (mintCurrentStepState != stepState.STATE_FINISHED)))
            {
                if (debug >= 1)
                {
                    fileLogger.writeEvent(TAG, "Timeout:- ");
                    Log.d(TAG, "Timeout:- ");
                }
                //  Transition to a new state.
                mintCurrentStepState = stepState.STATE_TIMEOUT;
            }

            //process LED status
            //ERROR - FLASH RED 3 TIMES
            switch (mint5291LEDStatus) {
                case STATE_TEAM:        //FLASH Alliance Colour
                    if (allianceColor.equals("Red"))
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                    else if (allianceColor.equals("Blue"))
                        LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOn);
                    else
                        LedState(LedOn, LedOn, LedOn, LedOn, LedOn, LedOn);
                    break;
                case STATE_ERROR:       //Flash RED 3 times Rapidly
                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        LedState(LedOff, LedOn, LedOff, LedOff, LedOn, LedOff);
                    } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 750))) {
                        mdblLastOff = mStateTime.milliseconds();
                        mblnLEDON = false;
                        LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                    }
                    break;
                case STATE_SUCCESS:       //Flash GREEN 3 times Rapidly
                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 250))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                    } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 250))) {
                        mdblLastOff = mStateTime.milliseconds();
                        mblnLEDON = false;
                        LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOff);
                        mintCounts ++;
                    }
                    if (mintCounts >= 5) {
                        mintCounts = 0;
                        mint5291LEDStatus = LEDState.STATE_TEAM;
                    }
                    break;
                case STATE_BEACON:       //

                    if ((!mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOff + 500))) {
                        mdblLastOn = mStateTime.milliseconds();
                        mblnLEDON = true;
                        if (mColour == Constants.BeaconColours.BEACON_BLUE_RED) {    //means red is to the right
                            LedState(LedOff, LedOff, LedOn, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.BeaconColours.BEACON_RED_BLUE) {
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.BeaconColours.BEACON_BLUE) {
                            LedState(LedOn, LedOff, LedOff, LedOff, LedOff, LedOff);
                        } else if (mColour == Constants.BeaconColours.BEACON_BLUE) {
                            LedState(LedOff, LedOn, LedOff, LedOff, LedOff, LedOff);
                        }
                    } else  if ((mblnLEDON) && (mStateTime.milliseconds() > (mdblLastOn + 500))) {
                        mdblLastOff = mStateTime.milliseconds();
                        mblnLEDON = false;
                        if (mColour == Constants.BeaconColours.BEACON_BLUE_RED) {    //means red is to the right
                            LedState(LedOff, LedOff, LedOff, LedOff, LedOn, LedOff);
                        } else if (mColour == Constants.BeaconColours.BEACON_RED_BLUE) {
                            LedState(LedOff, LedOff, LedOff, LedOff, LedOff, LedOn);
                        } else if (mColour == Constants.BeaconColours.BEACON_BLUE) {
                            LedState(LedOff, LedOff, LedOff, LedOn, LedOff, LedOff);
                        } else if (mColour == Constants.BeaconColours.BEACON_BLUE) {
                            LedState(LedOff, LedOff, LedOff, LedOff, LedOn, LedOff);
                        }
                        mintCounts ++;
                    }
                    if (mintCounts >= 10) {
                        mintCounts = 0;
                        mint5291LEDStatus = LEDState.STATE_TEAM;
                    }
                    break;
                case STATE_FINISHED:      //Solid Green
                    LedState(LedOn, LedOff, LedOff, LedOn, LedOff, LedOff);
                    break;

            }

            telemetry.update();
        }

        //opmode not active anymore

    }

    private void LedState (boolean g1, boolean r1, boolean b1, boolean g2, boolean r2, boolean b2) {

        dim.setDigitalChannelState(GREEN1_LED_CHANNEL, g1);   //turn LED ON
        dim.setDigitalChannelState(RED1_LED_CHANNEL, r1);
        dim.setDigitalChannelState(BLUE1_LED_CHANNEL, b1);
        dim.setDigitalChannelState(GREEN2_LED_CHANNEL, g2);   //turn LED ON
        dim.setDigitalChannelState(RED2_LED_CHANNEL, r2);
        dim.setDigitalChannelState(BLUE2_LED_CHANNEL, b2);

    }
    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    private void initStep (LibraryStateSegAuto mStateSegAuto) {
        mdblStepDistance = 0;

        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Starting to Decode Step ");
            Log.d(TAG, "Starting to Decode Step ");
        }

        // Reset the state time, and then change to next state.
        mblnBaseStepComplete = false;
        mStateTime.reset();

        mdblStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mdblStepSpeed = mStateSegAuto.getmRobotSpeed();
        mdblRobotCommand = mStateSegAuto.getmRobotCommand();
        mdblRobotParm1 = mStateSegAuto.getmRobotParm1();
        mdblRobotParm2 = mStateSegAuto.getmRobotParm2();
        mdblRobotParm3 = mStateSegAuto.getmRobotParm3();
        mdblRobotParm4 = mStateSegAuto.getmRobotParm4();
        mdblRobotParm5 = mStateSegAuto.getmRobotParm5();
        mdblRobotParm6 = mStateSegAuto.getmRobotParm6();
        mblnParallel =  mStateSegAuto.getmRobotParallel();
        mblnRobotLastPos = mStateSegAuto.getmRobotLastPos();
        mdblRobotStepComplete = mStateSegAuto.getmRobotStepComplete();

        if (debug >= 3) {
            fileLogger.writeEvent("initStep()", "mdblRobotCommand.substring(0, 3)    :- " + mdblRobotCommand.substring(0, 3));
            fileLogger.writeEvent("initStep()", "mdblRobotCommand.substring(3)       :- " + mdblRobotCommand.substring(3));
        }

        mintCurrentStepState = stepState.STATE_RUNNING;



        switch (mdblRobotCommand.substring(0, 3))
        {
            case "DEL":
                mintCurrentDelayState = stepState.STATE_INIT;
                break;
            case "GTH":
                mintCurrentTankTurnGyroHeadingState = stepState.STATE_INIT;
                break;
            case "GTB":  // Shoot the Particle balls
                mintCurrentTankTurnGyroBasicState = stepState.STATE_INIT;
                break;
            case "LTE":
                mintCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "RTE":
                mintCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "LPE":
                mintCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "RPE":
                mintCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "LRE":  // Left turn with a Radius in Parm 1
                mintCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "RRE":  // Right turn with a Radius in Parm 1
                mintCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "FWE":  // Drive forward a distance in inches and power setting
                //mintCurrentDriveState = stepState.STATE_INIT;
                mintCurrentDriveHeadingState = stepState.STATE_INIT;
                break;
            case "ASE":  // Plot a course using A* algorithm, accuracy in Parm 1
                mintCurrentStepState = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "VFL":  // Position the robot using vuforia parameters ready fro AStar  RObot should postion pointing to Red wall and Blue wall where targets are located
                mintCurrentVuforiaState = stepState.STATE_INIT;
                break;
            case "VME":  // Move the robot using localisation from the targets
                mintCurrentVuforiaMoveState = stepState.STATE_INIT;
                break;
            case "VTE":  // Turn the Robot using information from Vuforia and Pythag
                mintCurrentVuforiaTurnState = stepState.STATE_INIT;
                break;
            case "BCL":  // Get the beacon colour and move the robot to press the button
                mintCurrentBeaconColourState = stepState.STATE_INIT;
                break;
            case "ATB":  // Press the beacon button robot to press the button
                mintCurrentAttackBeaconState = stepState.STATE_INIT;
                break;
            case "ST1":  // Shoot the Particle balls
                mint5291CurrentShootParticleState = stepState.STATE_INIT;
                break;
            case "ST2":  // Shoot the Particle balls
                mint11231CurrentShootParticleState = stepState.STATE_INIT;
                break;
            case "SF1":  // Special Function, 5291 Move forward until line is found
                mint5291CurrentLineFindState = stepState.STATE_INIT;
                break;
            case "GTE":  // Special Function, 5291 Move forward until line is found
                mint5291GyroTurnEncoderState = stepState.STATE_INIT;
                break;
            case "FNC":  //  Run a special Function with Parms

                break;
        }

        if (debug >= 2) {
            fileLogger.writeEvent("initStep()", "Current Step          :- " + mintCurrentStep);
            fileLogger.writeEvent("initStep()", "mdblStepTimeout       :- " + mdblStepTimeout);
            fileLogger.writeEvent("initStep()", "mdblStepSpeed         :- " + mdblStepSpeed);
            fileLogger.writeEvent("initStep()", "mdblRobotCommand      :- " + mdblRobotCommand);
            fileLogger.writeEvent("initStep()", "mblnParallel          :- " + mblnParallel);
            fileLogger.writeEvent("initStep()", "mblnRobotLastPos      :- " + mblnRobotLastPos);
            fileLogger.writeEvent("initStep()", "mdblRobotParm1        :- " + mdblRobotParm1);
            fileLogger.writeEvent("initStep()", "mdblRobotParm2        :- " + mdblRobotParm2);
            fileLogger.writeEvent("initStep()", "mdblRobotParm3        :- " + mdblRobotParm3);
            fileLogger.writeEvent("initStep()", "mdblRobotParm4        :- " + mdblRobotParm4);
            fileLogger.writeEvent("initStep()", "mdblRobotParm5        :- " + mdblRobotParm5);
            fileLogger.writeEvent("initStep()", "mdblRobotParm6        :- " + mdblRobotParm6);
            fileLogger.writeEvent("initStep()", "mdblStepDistance      :- " + mdblStepDistance);
            fileLogger.writeEvent("initStep()", "mdblStepTurnL         :- " + mdblStepTurnL);
            fileLogger.writeEvent("initStep()", "mdblStepTurnR         :- " + mdblStepTurnR);
            fileLogger.writeEvent("initStep()", "mdblRobotStepComplete :- " + mdblRobotStepComplete);
        }
    }

    private void DelayStep ()
    {
        switch (mintCurrentDelayState) {
            case STATE_INIT: {
                mintStepDelay = Integer.parseInt(mdblRobotCommand.substring(3));
                mintCurrentDelayState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("DelayStep()", "Init Delay Time    " + mintStepDelay);
                    Log.d("DelayStep()", "Init Delay Time    " + mintStepDelay);
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= mintStepDelay) {
                    if (debug >= 1) {
                        fileLogger.writeEvent("DelayStep()", "Complete         ");
                        Log.d("DelayStep()", "Complete         ");
                    }
                    mintCurrentDelayState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }


}
