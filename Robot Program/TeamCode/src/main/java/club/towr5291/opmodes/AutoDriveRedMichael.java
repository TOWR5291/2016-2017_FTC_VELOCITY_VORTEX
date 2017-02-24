package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryStateSegAutoOld;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensors;


/**
 * Created by ianhaden on 2/09/16.
 */

@Autonomous(name="Pushbot: Auto Drive Red", group="5291Test")
@Disabled
public class AutoDriveRedMichael extends OpMode {
    /* Declare OpMode members. */
    HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use base drive hardware configuration

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    static final double     DRIVE_GEAR_REDUCTION    = 1.333 ;   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_ACTUAL_FUDGE      = 1;        // Fine tuning amount
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
    static final double     ROBOT_TRACK             = 16.5;     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    static final double     COUNTS_PER_DEGREE       =  ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;

    HardwareSensors robotSensors   = new HardwareSensors();     // Use base drive hardware configuration

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

       //define each state for the step.  Each step should go through some of the states below
    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED
    }

    private int mCurrentStep = 0;                               // Current State Machine State.
    private stepState  mCurrentStepState;                       // Current State Machine State.
    private stepState  mCurrentDriveState;                      // Current State Machine State.
    private stepState  mCurrentTurnState;                       // Current State Machine State.
    private LibraryStateSegAutoOld[] mStateSegAuto;
    double mStepTimeout;
    double mStepDistance;
    double mStepSpeed;
    String mRobotDirection;
    double mStepTurnL;
    double mStepTurnR;
    int mStepLeftTarget;
    int mStepRightTarget;
    boolean baseStepComplete = false;
    boolean armStepComplete = true;

    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();           // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();         // Time into current state

    //this is the sequence the state machine will follow
    private LibraryStateSegAutoOld[] mRobotAutonomous = {
            //                          time, head, dist, powe
            //                          out   ing   ance  r
            //                           s    deg   inch   %
            new LibraryStateSegAutoOld ( 10,  "45",   96,  1 ),
            new LibraryStateSegAutoOld ( 10,  "45",   96,  1 )

    };

    /*
    * Code to run ONCE when the driver hits INIT
    */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        robotDrive.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.update();
        robotDrive.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fileLogger.writeEvent("init()","Init Complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        fileLogger.writeEvent("start()","START PRESSED: ");


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        fileLogger.writeEvent("loop()","STATE: " + String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString() + " Current Turn State:- " + mCurrentTurnState.toString()+ " Current Drive State:- " + mCurrentDriveState.toString());
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("STATE", String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString());
        // Execute the current state.  Each STATE's case code does the following:

        switch (mCurrentStepState)
        {
            case STATE_INIT:
                {
                    initStep(mRobotAutonomous);
                }
                break;
            case STATE_START:
                {

                }
                break;
            case STATE_RUNNING:
                {
                    runningTurnStep ();
                    runningDriveStep();
                    if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentTurnState == stepState.STATE_COMPLETE) && (armStepComplete))
                    {
                        //  Transition to a new state.
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
                    fileLogger.writeEvent("loop()","Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                    if ((mCurrentStep) < (mRobotAutonomous.length - 1)) {
                        fileLogger.writeEvent("loop()","Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                        //  Transition to a new state and next step.
                        mCurrentStep++;
                        mCurrentStepState = stepState.STATE_INIT;

                    } else {
                        fileLogger.writeEvent("loop()","STATE_COMPLETE - Setting FINISHED ");
                        //  Transition to a new state.
                        mCurrentStepState = stepState.STATE_FINISHED;
                    }
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
                    telemetry.addData("STATE", "FINISHED " + mCurrentStep);
                }
            break;

        }

        //check timeout vale
        if ((mStateTime.seconds() > mStepTimeout  ) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED))) {
            //  Transition to a new state.
            mCurrentStepState = stepState.STATE_TIMEOUT;
        }

        telemetry.update();
    }


     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        telemetry.addData("FileLogger Op Stop: ", runtime.toString());
        if (fileLogger != null) {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }


    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    public void initStep (LibraryStateSegAutoOld[] step)
    {

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();
        mStateSegAuto = step;
        mStepTimeout = mStateSegAuto[mCurrentStep].mRobotTimeOut;
        mStepDistance =  mStateSegAuto[mCurrentStep].mRobotDistance;
        mStepSpeed = mStateSegAuto[mCurrentStep].mRobotSpeed;
        mRobotDirection = mStateSegAuto[mCurrentStep].mRobotDirection;

        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 0)    :- " + mRobotDirection.substring(0, 0)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 1)    :- " + mRobotDirection.substring(0, 1)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 2)    :- " + mRobotDirection.substring(0, 2)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 3)    :- " + mRobotDirection.substring(0, 3)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(1)       :- " + mRobotDirection.substring(1)  );

        if (mRobotDirection.substring(0, 1).equals("L")) {
            mStepTurnL = Double.parseDouble(mRobotDirection.substring(1));
            mStepTurnR = 0;
        } else if (mRobotDirection.substring(0, 1).equals("R" )) {
            mStepTurnL = 0;
            mStepTurnR = Double.parseDouble(mRobotDirection.substring(1));
        } else {
            mStepTurnL = 0;
            mStepTurnR = 0;
            mCurrentTurnState = stepState.STATE_COMPLETE;
        }

        mCurrentStepState = stepState.STATE_RUNNING;

//        fileLogger.writeEvent("initStep()","Current Step    :- " + mCurrentStep  );
//        fileLogger.writeEvent("initStep()","mStepTimeout    :- " + mStepTimeout  );
//        fileLogger.writeEvent("initStep()","mStepDistance   :- " + mStepDistance  );
//        fileLogger.writeEvent("initStep()","mStepSpeed      :- " + mStepSpeed  );
//        fileLogger.writeEvent("initStep()","mRobotDirection :- " + mRobotDirection  );
//        fileLogger.writeEvent("initStep()","mStepTurnL      :- " + mStepTurnL  );
//        fileLogger.writeEvent("initStep()","mStepTurnR      :- " + mStepTurnR  );

    }

    //--------------------------------------------------------------------------
    //  Execute the state.
    //--------------------------------------------------------------------------
    public void runningTurnStepPivot ()
    {
       switch (mCurrentTurnState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningTurnStep()","mStepTurnL      :- " + mStepTurnL  );
                fileLogger.writeEvent("runningTurnStep()","mStepTurnR      :- " + mStepTurnR  );

                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                    // pass target position to motor controller
                    robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                    // set motor controller to mode
                    robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveLeftMotorPower(Math.abs(.5));
                }
                else {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                    // pass target position to motor controller
                    robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveRightMotorPower(Math.abs(.5));
                }

                fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );

                mCurrentTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                fileLogger.writeEvent("runningTurnStep()","Running         " );
                fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                if (mStepTurnR == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.leftMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.rightMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(0);
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    mCurrentTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    public void runningTurnStep ()
    {
        switch (mCurrentTurnState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningTurnStep()","mStepTurnL      :- " + mStepTurnL  );
                fileLogger.writeEvent("runningTurnStep()","mStepTurnR      :- " + mStepTurnR  );

                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int)(0.5 * mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() - (int)(0.5 * mStepTurnL * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                }
                else {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() - (int)(0.5 * mStepTurnR * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int)(0.5 * mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                }

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);
                // set motor controller to mode
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(.5));

                fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );

                mCurrentTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                fileLogger.writeEvent("runningTurnStep()","Running         " );
                fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                if (mStepTurnR == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.leftMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    if (!robotDrive.rightMotor1.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(0);
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    mCurrentTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    public void runningDriveStep()
    {
        if (mCurrentTurnState == stepState.STATE_COMPLETE) {
            switch (mCurrentDriveState) {
                case STATE_INIT: {
                    fileLogger.writeEvent("runningDriveStep()","mStepDistance   :- " + mStepDistance  );
                    fileLogger.writeEvent("runningDriveStep()","mStepDistance   :- " + mStepDistance  );

                    // Determine new target position
                    mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                    mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                    // pass target position to motor controller
                    robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                    robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);

                    fileLogger.writeEvent("runningDriveStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                    fileLogger.writeEvent("runningDriveStep()","mStepRightTarget:- " + mStepRightTarget  );

                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // set power on motor controller to start moving
                    setDriveMotorPower(Math.abs(mStepSpeed));

                    mCurrentDriveState = stepState.STATE_RUNNING;
                }
                break;
                case STATE_RUNNING: {
                    if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy()) {
                        telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    } else {
                        // Stop all motion;
                        setDriveMotorPower(0);
                        baseStepComplete = true;
                        fileLogger.writeEvent("runningDriveStep()","Complete         " );
                        mCurrentDriveState = stepState.STATE_COMPLETE;
                    }
                }
                break;
            }
        }
    }

    void setDriveMotorPower (double power) {
        setDriveRightMotorPower(power);
        setDriveLeftMotorPower(power);
    }

    void setDriveRightMotorPower (double power) {
        robotDrive.rightMotor1.setPower(power);
        robotDrive.rightMotor2.setPower(power);

    }

    void setDriveLeftMotorPower (double power) {
        robotDrive.leftMotor1.setPower(power);
        robotDrive.leftMotor2.setPower(power);

    }

}
