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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

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

@Autonomous(name="Auto Drive Linear", group="5291Test")
public class AutoDriveEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;    // This is < 1.0 if geared UP, Tilerunner is geared up
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_ACTUAL_FUDGE      = 1;        // Fine tuning amount
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
    static final double     ROBOT_TRACK             = 16.5;     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    static final double     COUNTS_PER_DEGREE       =  ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;

    //set up the variables for the file logger
    private String startDate;
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

    // set up the variables for the state engine
    private int mCurrentStep = 0;                               // Current State Machine State.
    private stepState mCurrentStepState;                        // Current State Machine State.
    private stepState mCurrentDriveState;                       // Current State Machine State.
    private stepState mCurrentTurnState;                        // Current State Machine State.
    private LibraryStateSegAuto[] mStateSegAuto;
    private double mStepTimeout;
    double mStepSpeed;
    String mRobotCommand;
    double mRobotParm1;
    double mRobotParm2;
    double mRobotParm3;
    double mStepTurnL;
    double mStepTurnR;
    double mStepDistance;
    int mStepLeftTarget;
    int mStepRightTarget;
    boolean baseStepComplete = false;
    boolean armStepComplete = true;
    static final double INCREMENT   = 0.03;                     // amount to ramp motor each cycle
    private ElapsedTime mStateTime = new ElapsedTime();         // Time into current state

    //this is the sequence the state machine will follow
    private LibraryStateSegAuto[] mRobotAutonomous = {
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
            //
            //                        time, comm,  parm, parm, parm, powe
            //                        out   and    1     2     3     r
            //                         s                             %
            new LibraryStateSegAuto ( 10,  "LT90", 0,    0,    0,    0.5 ),
            new LibraryStateSegAuto ( 10,  "RT90", 0,    0,    0,    0.5 ),
            new LibraryStateSegAuto ( 10,  "FW12", 0,    0,    0,    1 ),
            new LibraryStateSegAuto ( 10,  "RV6" , 0,    0,    0,    0.3 )

    };

    @Override
    public void runOpMode() throws InterruptedException {

        // Set the start date for the logger filename and timestamps
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        // Start the filelogger, and initialise the first few lines
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robotDrive.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robotDrive.leftMotor1.getCurrentPosition(),
                robotDrive.rightMotor1.getCurrentPosition());
        telemetry.update();


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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

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
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robotDrive.leftMotor1.setTargetPosition(newLeftTarget);
            robotDrive.rightMotor1.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setDriveMotorPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robotDrive.leftMotor1.getCurrentPosition(),
                        robotDrive.rightMotor1.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            setDriveMotorPower(0);

            // Turn off RUN_TO_POSITION
            robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

        //opmode finished, close log file
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
    public void initStep (LibraryStateSegAuto[] step) {

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();
        mStateSegAuto = step;
        mStepTimeout = mStateSegAuto[mCurrentStep].mRobotTimeOut;
        mStepSpeed = mStateSegAuto[mCurrentStep].mRobotSpeed;
        mRobotCommand = mStateSegAuto[mCurrentStep].mRobotCommand;
        mRobotParm1 = mStateSegAuto[mCurrentStep].mRobotParm1;
        mRobotParm2 = mStateSegAuto[mCurrentStep].mRobotParm2;
        mRobotParm3 = mStateSegAuto[mCurrentStep].mRobotParm3;
        String fRobotCommand;

        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 0)    :- " + mRobotDirection.substring(0, 0)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 1)    :- " + mRobotDirection.substring(0, 1)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 2)    :- " + mRobotDirection.substring(0, 2)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 3)    :- " + mRobotDirection.substring(0, 3)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(1)       :- " + mRobotDirection.substring(1)  );

        //get the command
        fRobotCommand = mStateSegAuto[mCurrentStep].mRobotCommand.substring(0, 2);

        switch (fRobotCommand)
        {
            case "LT":
                mStepTurnL = Double.parseDouble(mStateSegAuto[mCurrentStep].mRobotCommand.substring(2));
                mStepTurnR = 0;
                break;
            case "RT":
                mStepTurnL = 0;
                mStepTurnR = Double.parseDouble(mStateSegAuto[mCurrentStep].mRobotCommand.substring(2));
                break;
            case "LP":

                break;
            case "RP":

                break;
            case "LR":  // Left turn with a Radius in Parm 1

                break;
            case "RR":  // Right turn with a Radius in Parm 1

                break;
            case "FW":  // Drive forward a distance in inches and power setting
                mStepDistance = Double.parseDouble(mStateSegAuto[mCurrentStep].mRobotCommand.substring(2));
                break;
            case "RV":  // Drive backward a distance in inches and power setting
                mStepDistance = -Double.parseDouble(mStateSegAuto[mCurrentStep].mRobotCommand.substring(2));
                break;
            case "AS":  // Plot a course using A* algorithm, accuracy in Parm 1

                break;
            case "FN":  //  Run a special Function with Parms

                break;
        }


        mCurrentStepState = stepState.STATE_RUNNING;

        fileLogger.writeEvent("initStep()","Current Step    :- " + mCurrentStep  );
        fileLogger.writeEvent("initStep()","mStepTimeout    :- " + mStepTimeout  );
        fileLogger.writeEvent("initStep()","mStepSpeed      :- " + mStepSpeed  );
        fileLogger.writeEvent("initStep()","mRobotCommand   :- " + mRobotCommand  );
        fileLogger.writeEvent("initStep()","mRobotParm1     :- " + mRobotParm1  );
        fileLogger.writeEvent("initStep()","mRobotParm2     :- " + mRobotParm2  );
        fileLogger.writeEvent("initStep()","mRobotParm3     :- " + mRobotParm3  );

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

    public void DriveStep()
    {
        switch (mCurrentDriveState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningDriveStep()", "mStepDistance   :- " + mStepDistance);
                fileLogger.writeEvent("runningDriveStep()", "mStepDistance   :- " + mStepDistance);

                // Determine new target position
                mStepLeftTarget = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                mStepRightTarget = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget);

                fileLogger.writeEvent("runningDriveStep()", "mStepLeftTarget :- " + mStepLeftTarget);
                fileLogger.writeEvent("runningDriveStep()", "mStepRightTarget:- " + mStepRightTarget);

                // set motor controller to mode, Turn On RUN_TO_POSITION
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mCurrentDriveState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                // ramp up speed - need to write function to ramp up speed
                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(mStepSpeed));

                //determine how close to target we are


                //if getting close ramp down speed


                //if within error marging stop


                if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy()) {
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                } else {
                    // Stop all motion;
                    setDriveMotorPower(0);
                    baseStepComplete = true;
                    fileLogger.writeEvent("runningDriveStep()", "Complete         ");
                    mCurrentDriveState = stepState.STATE_COMPLETE;
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

    //set the drive motors power, both left and right
    void setDriveMotorPower (double power) {
        setDriveRightMotorPower(power);
        setDriveLeftMotorPower(power);
    }

    //set the right drive motors power
    void setDriveRightMotorPower (double power) {
        robotDrive.rightMotor1.setPower(power);
        robotDrive.rightMotor2.setPower(power);
    }

    //set the left motors drive power
    void setDriveLeftMotorPower (double power) {
        robotDrive.leftMotor1.setPower(power);
        robotDrive.leftMotor2.setPower(power);
    }
}
