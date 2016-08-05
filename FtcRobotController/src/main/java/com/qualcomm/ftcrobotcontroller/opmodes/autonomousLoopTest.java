/*
controlling the robot for team 5291

Date Created:- 11/121/2015
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import java.lang.Math;
import java.util.concurrent.locks.Lock;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;


/**
 * TeleOp Mode
 *
 * Autonomous mode TeleOp.
 * Robot should run for 30 seconds
 * Move from Home position, detect beacon light, press button, the climb mountain.
 */


public class autonomousLoopTest extends OpMode {
    // device interface module
    DeviceInterfaceModule cdim;

    boolean debugLevel0 = true;
    boolean debugLevel1 = true;
    boolean debugLevel99 = true;

    // bEnabled and bDisabled represents the state of the output pins.
    boolean bEnabled = true;
    boolean bDisabled = false;

    // bPrevState and bCurrState represent the previous and current state of the button on the GamePad.
    boolean bPrevState = false;
    boolean bCurrState = false;

    int motorDeviceModeRead = 1;
    int motorDeviceModeWrite = 2;
    int countsPerInch = 130;
    int countsPerDegreeCenterPivot = 35;
    int countsPerDegreeSidePivotFor = 44;
    int countsPerDegreeSidePivotRev = 55;

    static final int COLORLED_CHANNEL = 0;
    static final int REDLED_CHANNEL = 1;
    static final int GREENLED_CHANNEL = 2;
    static final int BLUELED_CHANNEL = 3;

    // TETRIX VALUES.
    final static double SERVORIGHT_MIN_RANGE  = 0.50;
    final static double SERVORIGHT_MAX_RANGE  = 1.0;
    final static double SERVOLEFT_MIN_RANGE  = 0.50;
    final static double SERVOLEFT_MAX_RANGE  = 1.0;

    double servoRightPosition;
    double servoLeftPosition;

    // the servos are on the motor controller
    Servo servoRight;
    Servo servoLeft;

    // servo controller device
    ServoController servodevice;

    // the motors are on the motor controller
    DcMotor motorRight;
    DcMotor motorLeft;

    // motor controller device
    DcMotorController motordevice;

    boolean completed = false;
    boolean rotateCompleteCenter = false;
    boolean rotateCompleteSide = false;
    boolean moveComplete = false;
    boolean servoMoveComplete = false;
    boolean specialComplete = false;

    // line sensor
    float whiteLineReferenceValue = 300;
    int lineTrackSensorLeft;
    int lineTrackSensorCenter;
    int lineTrackSensorRight;
    boolean moveRight = false;
    boolean nudgeRight = false;
    boolean nudgeLeft = false;
    boolean moveLeft = false;
    boolean centered = false;
    boolean centerLineDetected = false;
    boolean leftLineDetected = false;
    boolean rightLineDetected = false;


    //gyro
    GyroSensor sensorGyro;
    int xVal, yVal, zVal = 0;
    int heading = 0;


    // the first position in the array is not used, as its position 0 or step
    // turing and moving in one step is allowed, turns happen first, then movement
    // align steps to           { 0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,  11,  12,  13,  14,  15,  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50 };  // used for aligning steps to array position
    int pivotTurnSteps[]      = { 0,    0,    0,   90,   90,  -90,  -90,  -90,  -90,   0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees, a single side will drive depending on the angle (pivoting around side of robot)
    int centerTurnSteps[]     = { 0,    0,    0,    0,    0,    0,    0,    0,    0,    45,  45,  45,  180, 0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees, both tracks will drive (pivoting around the center of the robot)
    int movementSteps[]       = { 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   10,  0,   10,  12,  24,  -12, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in inches
    double servoRightSteps[]  = { 0,    0,    105,  115,  125,  135,  145,  155,  165,  90,  0,   0,   180, 0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees of rotation
    double servoLeftSteps[]   = { 0,    0,    105,  115,  125,  135,  145,  155,  165,  0,   90,  0,   0,   180, 0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees of rotation
    double powerSteps[]       = { 0,    0,    0.2,  0.2,  0.2,  0.2,  0.2,  0.2,  0.2,  0.5, 0.5, 0.5, 0.5, 0.5, 0.7, 0.7, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in percentage, max of 1, min of 0,
    int specialSteps[]        = { 0,    5,    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // special function to call, this way you can easily move them around without changing the case below

    /**
     * Constructor
     */
    public autonomousLoopTest() {

    }

    public boolean specialStep (int function) {

        if (debugLevel0)
            DbgLog.msg("specialStep :- Start");

        switch (function) {

            case 1: {
                LEDStatus(false, false, false);
            }
            break;

            case 2: {
            }
            break;

            case 5: {

                while (true) {
                    lineTrackSensorLeft = cdim.getAnalogInputValue(1);
                    //telemetry.addData("Left Sensor:- ", lineTrackSensorLeft);
                    lineTrackSensorCenter = cdim.getAnalogInputValue(2);
                    //telemetry.addData("Center Sensor:- ", lineTrackSensorCenter);
                    lineTrackSensorRight = cdim.getAnalogInputValue(3);
                    //telemetry.addData("Right Sensor:- ", lineTrackSensorRight);


                    if (debugLevel1)
                        DbgLog.msg("specialStep :- lineTrackSensorLeft = " +lineTrackSensorLeft);

                    if (debugLevel1)
                        DbgLog.msg("specialStep :- lineTrackSensorCenter = " +lineTrackSensorCenter);

                    if (debugLevel1)
                        DbgLog.msg("specialStep :- lineTrackSensorRight = " +lineTrackSensorRight);

                    // Determine if white line has been detected
                    // assume position is not know on every check
                    moveRight = false;
                    nudgeRight = false;
                    nudgeLeft = false;
                    moveLeft = false;
                    centered = false;
                    centerLineDetected = false;
                    leftLineDetected = false;
                    rightLineDetected = false;

                    // detect center line
                    if (lineTrackSensorCenter <= whiteLineReferenceValue) {
                        centerLineDetected = true;
                    }
                    if (lineTrackSensorLeft <= whiteLineReferenceValue) {
                        leftLineDetected = true;
                    }
                    if (lineTrackSensorRight <= whiteLineReferenceValue) {
                        rightLineDetected = true;
                    }

                    if ((leftLineDetected == true) && (leftLineDetected == centerLineDetected)) {
                        // check if left and center is on - nudge to the right only
                        // just off center
                        nudgeRight = true;
                        telemetry.addData("Line Position", "Left Centered");
                    } else if ((leftLineDetected == true) && (leftLineDetected != centerLineDetected)) {
                        // check if left is on - move to the right
                        moveRight = true;
                        telemetry.addData("Line Position", "Left");
                    } else if ((rightLineDetected == true) && (rightLineDetected == centerLineDetected)) {
                        // check if right and center is on - nudge to the left only
                        // just off center
                        nudgeLeft = true;
                        telemetry.addData("Line Position", "Right Centered");
                    } else if ((rightLineDetected == true) && (rightLineDetected != centerLineDetected)) {
                        // check if right is on - move to the left
                        moveLeft = true;
                        telemetry.addData("Line Position", "Right");
                    } else if (centerLineDetected) {
                        // check if we are centered, if this is not true we are not on the line

                        moveRight = false;
                        nudgeRight = false;
                        nudgeLeft = false;
                        moveLeft = false;
                        centered = true;
                        telemetry.addData("Line Position", "Centered");
                    }
    /*
                    throttle = (float) 0.2;  // % max speed
                    direction = (float) 0;  // straight ahead

                    //adjsut direction based on line
                    if (moveRight) {
                        direction = (float) 0.2;
                    }
                    if (moveLeft) {
                        direction = - (float) 0.2;
                    }
                    if (nudgeRight) {
                        direction = (float) 0.1;
                    }
                    if (nudgeLeft) {
                        direction = - (float)  0.1;
                    }
                    if (centered) {
                        direction = (float) 0;
                    }

                    right = throttle - direction;
                    left = throttle + direction;
                    telemetry.addData("Left Motor :- ",  String.format("%.2f", left));
                    telemetry.addData("Right Motor:- ",  String.format("%.2f", right));
                    // write the values to the motors
                    motorRight.setPower(right);
                    motorLeft.setPower(left);
    */

                    if (gamepad1.y) {
                        break;
                    }

                }
            }
            break;

            case 99: {

            }
            break;

            default:
                break;

        }

        if (debugLevel0)
            DbgLog.msg("specialStep :- End");

        return true;
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");

        LEDStatus(false, true, false);

        initServos();

        initMotors();

        initMotorEncoders();

        initLineTrack ();

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {


        while (!completed) {
            for (int step = 1; step < 3; step++) {
                telemetry.addData("Step", " " + step + " START!");
                telemetry.addData("TurnAngleSidePivot", " step " + step + " - " + pivotTurnSteps[step]);
                telemetry.addData("TurnAngleCentPivot", " step " + step + " - " + centerTurnSteps[step]);
                telemetry.addData("MoveDistance", " step " + step + " - " + movementSteps[step]);
                telemetry.addData("MoveServos", " step " + step + " - " + servoRightSteps[step] + " " + servoLeftSteps[step]);
                telemetry.addData("specialStep", " step " + step + " - " + specialSteps[step]);
                rotateCompleteSide = robotTurnAngleSidePivot(powerSteps[step], pivotTurnSteps[step]);
                rotateCompleteCenter = robotTurnAngleCenterPivot(powerSteps[step], centerTurnSteps[step]);
                moveComplete = robotMoveDistance(powerSteps[step], movementSteps[step]);
                servoMoveComplete = moveServos(servoRightSteps[step], servoLeftSteps[step]);
                specialComplete = specialStep(specialSteps[step]);
                if (moveComplete && rotateCompleteSide && rotateCompleteCenter && servoMoveComplete && specialComplete) {
                    telemetry.addData("Step", " " + step + " COMPLETE!");
                } else {
                    telemetry.addData("Step", " " + step + " failed");
                }
            }
            completed = true;
        }

        //autonomous completed, slow the loop down a bit
        telemetry.addData("Step", " Finished");


    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        //Disable encoders and take out of running with encoders
        switchMotorControllerEncoderMode(4);
        motorRight.setPower(0);
        motorLeft.setPower(0);
        //set the red led to indicate the robot STOP command has been executed
        LEDStatus(true, false, false);
    }

    public void LEDStatus (boolean Red, boolean Green, boolean Blue) {
        if (debugLevel0)
            DbgLog.msg("LEDStatus :- Start ");

        if (Red == true)
            cdim.setDigitalChannelState(REDLED_CHANNEL, bEnabled);
        else
            cdim.setDigitalChannelState(REDLED_CHANNEL, bDisabled);
        if (Blue == true)
            cdim.setDigitalChannelState(BLUELED_CHANNEL, bEnabled);
        else
            cdim.setDigitalChannelState(BLUELED_CHANNEL, bDisabled);
        if (Green == true)
            cdim.setDigitalChannelState(GREENLED_CHANNEL, bEnabled);
        else
            cdim.setDigitalChannelState(GREENLED_CHANNEL, bDisabled);
        if (debugLevel0)
            DbgLog.msg("LEDStatus :- End ");
    }

    public void initLineTrack (){
        float avarageReadings;
        if (debugLevel0)
            DbgLog.msg("initLineTrack :- Start ");

        lineTrackSensorLeft = cdim.getAnalogInputValue(1);
        //telemetry.addData("Left Sensor:- ", lineTrackSensorLeft);
        lineTrackSensorCenter = cdim.getAnalogInputValue(2);
        //telemetry.addData("Center Sensor:- ", lineTrackSensorCenter);
        lineTrackSensorRight = cdim.getAnalogInputValue(3);
        //telemetry.addData("Right Sensor:- ", lineTrackSensorRight);

        if (debugLevel1)
            DbgLog.msg("initLineTrack :- lineTrackSensorLeft = " + lineTrackSensorLeft);

        if (debugLevel1)
            DbgLog.msg("initLineTrack :- lineTrackSensorCenter = " + lineTrackSensorCenter);

        if (debugLevel1)
            DbgLog.msg("initLineTrack :- lineTrackSensorRight = " + lineTrackSensorRight);


        // Determine correct reference, reference will change depending on reflectivity and distance above surface
        // Refereance should be lower than the greatest value of the 3 sensors less a nominal difference of approx 50 units
       /* if (lineTrackSensorCenter > lineTrackSensorLeft && lineTrackSensorCenter > lineTrackSensorRight) {
            whiteLineReferenceValue = lineTrackSensorCenter - 50;
        }  else if (lineTrackSensorLeft > lineTrackSensorCenter && lineTrackSensorLeft > lineTrackSensorRight) {
            whiteLineReferenceValue = lineTrackSensorLeft - 50;
        } else if (lineTrackSensorRight > lineTrackSensorCenter && lineTrackSensorRight > lineTrackSensorLeft) {
            whiteLineReferenceValue = lineTrackSensorRight - 50;
        } else  if ((lineTrackSensorCenter >= 350) && (lineTrackSensorLeft >= 350) && (lineTrackSensorRight >= 350)) {
            whiteLineReferenceValue = 350;
        }*/
        avarageReadings = (lineTrackSensorCenter + lineTrackSensorRight + lineTrackSensorLeft) / 3;

        whiteLineReferenceValue = avarageReadings + 50;

        if (debugLevel1)
            DbgLog.msg("initLineTrack :- whiteLineReferenceValue = " + whiteLineReferenceValue);

        if (debugLevel0)
            DbgLog.msg("initLineTrack :- End ");

    }


    public void initServos () {

        if (debugLevel0)
            DbgLog.msg("initServos :- Start ");
        servodevice = hardwareMap.servoController.get("servo");

        // Configuring the servos for control
        servoRight = hardwareMap.servo.get("servo1");
        servoLeft = hardwareMap.servo.get("servo2");

        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRightPosition = 0.5;
        servoLeftPosition = 0.5;
        servoRight.setPosition(servoRightPosition);
        servoLeft.setPosition(servoLeftPosition);

        if (debugLevel0)
            DbgLog.msg("initServos :- End ");
    }

    public boolean moveServos (double rightPosition, double leftPosition) {
        boolean rightOKToMove = true;
        boolean leftOKToMove = true;

        if (debugLevel0)
            DbgLog.msg("moveServos :- Start ");
        //set right position
        if ((Range.scale(rightPosition, 0, 180, 0, 1) < SERVORIGHT_MIN_RANGE ) || (Range.scale(rightPosition, 0, 180, 0, 1) > SERVORIGHT_MAX_RANGE )) {
            if (debugLevel1)
                DbgLog.msg("moveServos :- Right Out Of Range, no move ");
            rightOKToMove = false;
        }
        if ((Range.scale(leftPosition, 0, 180, 0, 1) < SERVOLEFT_MIN_RANGE ) || (Range.scale(leftPosition, 0, 180, 0, 1) > SERVOLEFT_MAX_RANGE )) {
            if (debugLevel1)
                DbgLog.msg("moveServos :- Left Out Of Range, no move ");
            leftOKToMove = false;
        }

        if (rightOKToMove) {
            servoRight.setPosition(Range.scale(rightPosition, 0, 180, 0, 1));
        }
        if (leftOKToMove) {
            servoLeft.setPosition(Range.scale(leftPosition, 0, 180, 0, 1));
        }

        if (debugLevel0)
            DbgLog.msg("moveServos :- End ");
        return true;
    }


    public void initMotors () {

        if (debugLevel0)
            DbgLog.msg("initMotors :- Start ");
        motordevice = hardwareMap.dcMotorController.get("motor");

        // Configuring the motors for control
        motorRight = hardwareMap.dcMotor.get("motor1");
        motorLeft = hardwareMap.dcMotor.get("motor2");

        //motor left runs in reverse
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        if (debugLevel0)
            DbgLog.msg("initMotors :- End ");
    }

    public void initMotorEncoders () {

        boolean stateOK = false;

        if (debugLevel0)
            DbgLog.msg("initMotorEncoders :- Start ");

        // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
        switchMotorControllerEncoderMode(1);

        //wait a bit to let the commands go active
        try {
            if (debugLevel1)
                DbgLog.msg("initMotorEncoders :- Busy? ");
            Thread.sleep(100);
        } catch (Exception ex) {
            if (debugLevel1)
                DbgLog.msg("initMotorEncoders :- Error ");
        }
        // Switch the motor controller to read mode
        //don't call this function too quickly, the controller crashes when changing modes too often
        stateOK = switchMotorControllerMode(motorDeviceModeRead);

        // wait until motor encoders are reset
        while ((motorLeft.getCurrentPosition() !=0) || (motorRight.getCurrentPosition() !=0)) {
            try {
                Thread.sleep(100);
                DbgLog.msg("initMotorEncoders - Waiting for encoder reset - Left " + motorLeft.getCurrentPosition() + " Right " + motorRight.getCurrentPosition());
                if (debugLevel1) {
                    //telemetry.addData("Waiting", "for encoder reset");
                }
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("initMotorEncoders - Error Waiting for encoder reset ");
                telemetry.addData("Error", " Encoders not reset");
            }
        }


        if (debugLevel1)
            DbgLog.msg("initMotorEncoders - Encoders Reset, Setting Mode to RUN TO POSITION ");
        // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
        switchMotorControllerEncoderMode(2);

        // Switch the motor controller to read mode
        //don't call this function too quickly, the controller crashes when changing modes too often
        stateOK = switchMotorControllerMode(motorDeviceModeRead);

        if (debugLevel1)
            DbgLog.msg("initMotorEncoders - Encoders Reset, Complete RUN TO POSITION ");
        if (debugLevel0)
            DbgLog.msg("initMotorEncoders :- End ");
    }

    public void switchMotorControllerEncoderMode (int state) {
        boolean stateOK = false;
        //  states
        // 1 = RESET_ENCODERS
        // 2 = RUN_TO_POSITION
        // 3 = RUN_USING_ENCODERS
        // 4 = RUN_WITHOUT_ENCODERS

        if (debugLevel0)
            DbgLog.msg("switchMotorControllerEncoderMode :- Start ");

        // check we are in write mode, if not switch to write mode
        stateOK = switchMotorControllerMode(motorDeviceModeWrite);

        switch (state) {
            case 1:
                motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                break;
            case 2:
                motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                break;
            case 3:
                motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                break;
            case 4:
                motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                break;
            default:
                break;

        }
        if (debugLevel0)
            DbgLog.msg("switchMotorControllerEncoderMode :- End ");
    }

    public boolean switchMotorControllerMode (int state) {
        if (debugLevel0)
            DbgLog.msg("switchMotorControllerMode :- Start ");

        int currentState = 0;
        // valid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
        // State 1 = READ_ONLY
        // State 2 = WRITE_ONLY
        // State 3 = SWITCHING_TO_READ_MODE
        // State 4 = SWITCHING_TO_WRITE_MODE

        // valid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
        if (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            currentState = 1;
            if (debugLevel1)
                DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice.getMotorControllerDeviceMode());
            if (state == currentState) {
                if (debugLevel1)
                    DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice.getMotorControllerDeviceMode());
                if (debugLevel0)
                    DbgLog.msg("switchMotorControllerMode :- End ");
                return true;  //already in the state we need, do nothing
            }
        }
        if (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY) {
            currentState = 2;
            if (debugLevel1)
                DbgLog.msg("switchMotorControllerMode - Currentstate 2 - device mode: " + motordevice.getMotorControllerDeviceMode());
            if (state == currentState) {
                if (debugLevel1)
                    DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice.getMotorControllerDeviceMode());
                if (debugLevel0)
                    DbgLog.msg("switchMotorControllerMode :- End ");
                return true;  //already in the state we need, do nothing
            }
        }

        //looks like we need to change state.
        // add a little delay, the stupic controller doesn't like changing state too often and goes AWOL
        try {
            Thread.sleep(50);
            if (debugLevel1) {
                DbgLog.msg("switchMotorControllerMode - Waiting for motorcontroller change state ");
                //telemetry.addData("Waiting", " Setting Write_Only");
            }
        } catch (Exception ex) {
            telemetry.addData("Error", " Setting Write_Only");
        }

        if (state != currentState) {
            switch (state) {
                case 1: {
                    if (debugLevel1)
                        DbgLog.msg("switchMotorControllerMode - Setting MotorController to Read ");
                    motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
                        try {
                            Thread.sleep(50);
                            if (debugLevel1){
                                DbgLog.msg("switchMotorControllerMode - Waiting for read mode ");
                                //telemetry.addData("Waiting", " Setting Read_Only");
                            }
                        } catch (Exception ex) {
                            telemetry.addData("Error", " Setting Read_Only");
                        }
                    }
                    if (debugLevel0)
                        DbgLog.msg("switchMotorControllerMode :- End ");
                    return true;
                }
                //break;  //unreachable statement because of the return above
                case 2: {
                    if (debugLevel1)
                        DbgLog.msg("switchMotorControllerMode - Setting MotorController to Write ");
                    motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
                        try {
                            Thread.sleep(50);
                            if (debugLevel1) {
                                DbgLog.msg("switchMotorControllerMode - Waiting for write mode ");
                                //telemetry.addData("Waiting", " Setting Write_Only");
                            }
                        } catch (Exception ex) {
                            telemetry.addData("Error", " Setting Write_Only");
                        }
                    }
                    if (debugLevel0)
                        DbgLog.msg("switchMotorControllerMode :- End ");
                    return true;
                }
                //break;  //unreachable statement because of the return above
            }
        }
        return false;  //should never get here
    }

    public boolean robotMoveDistance (double motorPower, int distance) {
        boolean stateOK = false;

        if (debugLevel0)
            DbgLog.msg("robotMoveDistance :- Start ");

        int countsToMove;           //the calculated counts required to move the distance requested
        int motorRightPos;          //the current position position for the right motor in encoder counts
        int motorLeftPos;           //the current position for the left  motor in encoder counts
        int motorRightTargetPos;    //the calculated target position for the right motor in encoder counts
        int motorLeftTargetPos;     //the calculated target position for the left  motor in encoder counts
        int targetAccuracy = 3;     //actual is double this, as it is plus minus this number
        boolean targetGood = false; //used to end this function, when robot has moved to the position this will go true and the function will exit
        boolean notSameReading = true;
        int motorRightPosCompare [] = { 0, 0, 0};
        int motorLeftPosCompare [] = { 0, 0, 0};

        if (distance == 0) {
            if (debugLevel0)
                DbgLog.msg("robotMoveDistance :- End ");
            return true;
        } else {
            //don't call this function too quickly, the controller crashes when changing modes too often
            stateOK = switchMotorControllerMode(motorDeviceModeRead);

            if (debugLevel1)
                DbgLog.msg("robotMoveDistance :- About to read encoders ");
            // find out the encoder positions now
            motorRightPos = motorRight.getCurrentPosition();
            motorLeftPos = motorLeft.getCurrentPosition();
            // get 3 readings the same before determining if the reading is accurate, have seen strange things without doing this
            while (notSameReading) {
                for (int loop = 0; loop < 3; loop++) {
                    motorRightPosCompare[loop] = motorRight.getCurrentPosition();
                    motorLeftPosCompare[loop] = motorLeft.getCurrentPosition();
                    try {
                        Thread.sleep(50);
                    } catch (Exception ex) {
                    }
                }
                if (((motorRightPosCompare[0] == motorRightPosCompare[1]) && (motorRightPosCompare[1] == motorRightPosCompare[2])) &&
                        ((motorLeftPosCompare[0] == motorLeftPosCompare[1]) && (motorLeftPosCompare[1] == motorLeftPosCompare[2])))
                {
                    motorRightPos = motorRightPosCompare[0];
                    motorLeftPos = motorLeftPosCompare[0];
                    notSameReading = false;
                }
            }

            countsToMove = distance * countsPerInch;
            //don't call this function too quickly, the controller crashes when changing modes too often
            stateOK = switchMotorControllerMode(motorDeviceModeWrite);

            motorRightTargetPos = motorRightPos + countsToMove;
            motorLeftTargetPos = motorLeftPos + countsToMove;
            if (debugLevel1) {
                DbgLog.msg("robotMoveDistance :- motorRightTargetPos " + motorRightTargetPos);
                DbgLog.msg("robotMoveDistance :- motorLeftTargetPos " + motorLeftTargetPos);
                DbgLog.msg("robotMoveDistance :- About to set encoder targets for move ");
            }
            motorRight.setTargetPosition(motorRightTargetPos);
            motorLeft.setTargetPosition(motorLeftTargetPos);

            motorRight.setPower(motorPower);
            motorLeft.setPower(motorPower);
            if (debugLevel1)
                DbgLog.msg("robotMoveDistance :- Power set, should be moving ");

            //wait a bit to let the commands go active
            try {
                if (debugLevel1)
                    DbgLog.msg("robotMoveDistance :- Motors Busy? ");
                Thread.sleep(500);
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("robotMoveDistance :- Error ");
            }
            if (debugLevel1)
                DbgLog.msg("robotMoveDistance :- About to read encoders ");
            //don't call this function too quickly, the controller crashes when changing modes too often
            stateOK = switchMotorControllerMode(motorDeviceModeRead);
            // find out the encoder positions now
            while (!targetGood) {
                motorRightPos = motorRight.getCurrentPosition();
                motorLeftPos = motorLeft.getCurrentPosition();
                if (((motorRightPos >= (motorRightTargetPos - targetAccuracy)) && (motorRightPos <= (motorRightTargetPos + targetAccuracy))) &&
                        ((motorLeftPos >= (motorLeftTargetPos - targetAccuracy)) && (motorLeftPos <= (motorLeftTargetPos + targetAccuracy))))
                {
                    if (debugLevel1)
                        DbgLog.msg("robotMoveDistance :- Distance achieved, powering down motors ");
                    targetGood = true;
                } else {
                    if (debugLevel1) {
                        DbgLog.msg("robotMoveDistance :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos);
                        DbgLog.msg("robotMoveDistance :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos);
                        //telemetry.addData("robotMoveDistance :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos , "");
                        //telemetry.addData("robotMoveDistance :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos, "");
                    }
                }
            }
            //don't call this function too quickly, the controller crashes when changing modes too often
            stateOK = switchMotorControllerMode(motorDeviceModeWrite);
            motorRight.setPower(0);
            motorLeft.setPower(0);
            if (debugLevel0)
                DbgLog.msg("robotMoveDistance :- End ");
            return true;
        }

    }

    public boolean robotTurnAngleCenterPivot (double power, int angle) {
        if (debugLevel0)
            DbgLog.msg("robotTurnAngleCenterPivot :- Start ");

        int countsToMove;            //the calculated counts required to move the distance requested
        int motorRightPos = 0;       //the current position position for the right motor in encoder counts
        int motorLeftPos = 0;        //the current position for the left  motor in encoder counts
        int motorRightTargetPos;     //the calculated target position for the right motor in encoder counts
        int motorLeftTargetPos;      //the calculated target position for the left  motor in encoder counts
        int targetAccuracy = 3;      //actual is double this, as it is plus minus this number
        boolean targetGood = false;  //used to end this function, when robot has moved to the position this will go true and the function will exit
        boolean notSameReading = true;
        int motorRightPosCompare [] = { 0, 0, 0};
        int motorLeftPosCompare [] = { 0, 0, 0};

        if (angle == 0) {
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleCenterPivot :- No Angle to Turn, exiting.");
            if (debugLevel0)
                DbgLog.msg("robotTurnAngleCenterPivot :- End ");
            return true;
        } else {
            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(1);
            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(2);
            // change controller to read mode so we can get the current position - should be zero since we reset the encoders
            // probably don't need to even do half this now we reset the encoders for every step we are going to take.

            switchMotorControllerMode(motorDeviceModeRead);
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleCenterPivot :- About to read encoders");
            // find out the encoder positions now
            motorRightPos = motorRight.getCurrentPosition();
            motorLeftPos = motorLeft.getCurrentPosition();
            // find out the encoder positions now
            // get 3 readings the same before determining if the reading is accurate, have seen strange things without doing this
            while (notSameReading) {
                for (int loop = 0; loop < 3; loop++) {
                    motorRightPosCompare[loop] = motorRight.getCurrentPosition();
                    motorLeftPosCompare[loop] = motorLeft.getCurrentPosition();
                    try {
                        Thread.sleep(50);
                    } catch (Exception ex) {
                    }
                }
                if (((motorRightPosCompare[0] == motorRightPosCompare[1]) && (motorRightPosCompare[1] == motorRightPosCompare[2])) &&
                        ((motorLeftPosCompare[0] == motorLeftPosCompare[1]) && (motorLeftPosCompare[1] == motorLeftPosCompare[2])))
                {
                    motorRightPos = motorRightPosCompare[0];
                    motorLeftPos = motorLeftPosCompare[0];
                    notSameReading = false;
                }
            }

            if (angle > 0) {
                countsToMove = (countsPerDegreeCenterPivot * angle);
                switchMotorControllerMode(motorDeviceModeWrite);
                if (power < 0) {
                    countsToMove = countsToMove * -1;
                }
                motorRightTargetPos = motorRightPos - countsToMove;
                motorLeftTargetPos = motorLeftPos + countsToMove;
            } else {
                countsToMove = (countsPerDegreeCenterPivot * angle);
                switchMotorControllerMode(motorDeviceModeWrite);
                if (power < 0) {
                    countsToMove = countsToMove * -1;
                }
                motorRightTargetPos = motorRightPos + countsToMove;
                motorLeftTargetPos = motorLeftPos - countsToMove;
            }
            if (debugLevel1) {
                DbgLog.msg("robotTurnAngleCenterPivot :- motorRightTargetPos " + motorRightTargetPos);
                DbgLog.msg("robotTurnAngleCenterPivot :- motorLeftTargetPos " + motorLeftTargetPos);
                DbgLog.msg("robotTurnAngleCenterPivot :- About to set encoder targets for move");
            }

            motorRight.setTargetPosition(motorRightTargetPos);
            motorLeft.setTargetPosition(motorLeftTargetPos);

            motorRight.setPower(0.3);
            motorLeft.setPower(0.3);

            if (debugLevel1)
                DbgLog.msg("robotTurnAngleCenterPivot :- Power set, should be moving");

            //wait a bit to let the commands go active
            try {
                if (debugLevel1)
                    DbgLog.msg("robotTurnAngleCenterPivot :- Motors Busy? ");
                Thread.sleep(200);
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("robotTurnAngleCenterPivot :- Error ");
            }
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleCenterPivot :- About to read encoders ");

            switchMotorControllerMode(motorDeviceModeRead);

            while (!targetGood) {
                motorRightPos = motorRight.getCurrentPosition();
                motorLeftPos = motorLeft.getCurrentPosition();
                if (((motorRightPos >= (motorRightTargetPos - targetAccuracy)) && (motorRightPos <= (motorRightTargetPos + targetAccuracy))) &&
                        ((motorLeftPos >= (motorLeftTargetPos - targetAccuracy)) && (motorLeftPos <= (motorLeftTargetPos + targetAccuracy))))
                {
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- Angle achieved, powering down motors ");
                    targetGood = true;
                } else {
                    //if (debugLevel1) {
                    //    DbgLog.msg("robotTurnAngleCenterPivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos);
                    //    DbgLog.msg("robotTurnAngleCenterPivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos);
                    //}
                    //telemetry.addData("robotTurnAngleCenterPivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos , "");
                    //telemetry.addData("robotTurnAngleCenterPivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos, "");
                }
            }

            switchMotorControllerMode(motorDeviceModeWrite);
            motorRight.setPower(0);
            motorLeft.setPower(0);

            if (debugLevel0)
                DbgLog.msg("robotTurnAngleCenterPivot :- End ");
            return true;
        }
    }

    public boolean robotTurnAngleSidePivot (double power, int angle) {
        if (debugLevel0)
            DbgLog.msg("robotTurnAngleSidePivot :- Start ");

        int countsToMove;            //the calculated counts required to move the distance requested
        int motorRightPos = 0;       //the current position position for the right motor in encoder counts
        int motorLeftPos = 0;        //the current position for the left  motor in encoder counts
        int motorRightTargetPos;     //the calculated target position for the right motor in encoder counts
        int motorLeftTargetPos;      //the calculated target position for the left  motor in encoder counts
        int targetAccuracy = 3;      //actual is double this, as it is plus minus this number
        boolean targetGood = false;  //used to end this function, when robot has moved to the position this will go true and the function will exit
        boolean notSameReading = true;
        int motorRightPosCompare [] = { 0, 0, 0};
        int motorLeftPosCompare [] = { 0, 0, 0};

        if (angle == 0) {
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- No Angle to Turn, exiting.");
            if (debugLevel0)
                DbgLog.msg("robotTurnAngleSidePivot :- End ");
            return true;
        } else {

            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(1);
            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(2);

            switchMotorControllerMode(motorDeviceModeRead);

            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- About to read encoders");

            motorRightPos = motorRight.getCurrentPosition();
            motorLeftPos = motorLeft.getCurrentPosition();
            // find out the encoder positions now
            // get 3 readings the same before determining if the reading is accurate, have seen strange things without doing this
            while (notSameReading) {
                for (int loop = 0; loop < 3; loop++) {
                    motorRightPosCompare[loop] = motorRight.getCurrentPosition();
                    motorLeftPosCompare[loop] = motorLeft.getCurrentPosition();
                    try {
                        Thread.sleep(50);
                    } catch (Exception ex) {
                    }
                }
                if (((motorRightPosCompare[0] == motorRightPosCompare[1]) && (motorRightPosCompare[1] == motorRightPosCompare[2])) &&
                        ((motorLeftPosCompare[0] == motorLeftPosCompare[1]) && (motorLeftPosCompare[1] == motorLeftPosCompare[2])))
                {
                    motorRightPos = motorRightPosCompare[0];
                    motorLeftPos = motorLeftPosCompare[0];
                    notSameReading = false;
                }
            }

            if (power > 0) {

                countsToMove = (countsPerDegreeSidePivotFor * angle);
            } else {
                countsToMove = -1 * (countsPerDegreeSidePivotRev * angle);
            }
            if (angle > 0) {  // Pivot on right side of robot
                motorRightTargetPos = motorRightPos;
                motorLeftTargetPos = motorLeftPos + countsToMove;

                switchMotorControllerMode(motorDeviceModeWrite);
                if (debugLevel1) {
                    DbgLog.msg("robotTurnAngleSidePivot :- motorRightTargetPos " + motorRightTargetPos);
                    DbgLog.msg("robotTurnAngleSidePivot :- motorLeftTargetPos " + motorLeftTargetPos);
                    DbgLog.msg("robotTurnAngleSidePivot :- About to set encoder targets for move");
                }

                motorRight.setTargetPosition(motorRightTargetPos);
                motorLeft.setTargetPosition(motorLeftTargetPos);

                motorRight.setPower(Math.abs(power));
                motorLeft.setPower(Math.abs(power));
            } else { // Pivot on left side of robot
                motorRightTargetPos = motorRightPos + countsToMove;
                motorLeftTargetPos = motorLeftPos;
                switchMotorControllerMode(motorDeviceModeWrite);

                if (debugLevel1) {
                    DbgLog.msg("robotTurnAngleSidePivot :- motorRightTargetPos " + motorRightTargetPos);
                    DbgLog.msg("robotTurnAngleSidePivot :- motorLeftTargetPos " + motorLeftTargetPos);
                    DbgLog.msg("robotTurnAngleSidePivot :- About to set encoder targets for move");
                }

                motorRight.setTargetPosition(motorRightTargetPos);
                motorLeft.setTargetPosition(motorLeftTargetPos);

                motorRight.setPower(Math.abs(power));
                motorLeft.setPower(Math.abs(power));
            }

            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- Power set, should be moving");

            //wait a bit to let the commands go active
            try {
                if (debugLevel1)
                    DbgLog.msg("robotTurnAngleSidePivot :- Motors Busy? ");
                Thread.sleep(200);
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("robotTurnAngleSidePivot :- Error ");
            }
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- About to read encoders ");

            switchMotorControllerMode(motorDeviceModeRead);

            while (!targetGood) {
                motorRightPos = motorRight.getCurrentPosition();
                motorLeftPos = motorLeft.getCurrentPosition();
                if (((motorRightPos >= (motorRightTargetPos - targetAccuracy)) && (motorRightPos <= (motorRightTargetPos + targetAccuracy))) &&
                        ((motorLeftPos >= (motorLeftTargetPos - targetAccuracy)) && (motorLeftPos <= (motorLeftTargetPos + targetAccuracy))))
                {
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- Angle achieved, powering down motors ");
                    targetGood = true;
                } else {
                    //if (debugLevel1) {
                    //    DbgLog.msg("robotTurnAngleSidePivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos);
                    //    DbgLog.msg("robotTurnAngleSidePivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos);
                    //}
                    //telemetry.addData("robotTurnAngleSidePivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos , "");
                    //telemetry.addData("robotTurnAngleSidePivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos, "");
                }
            }

            switchMotorControllerMode(motorDeviceModeWrite);

            motorRight.setPower(0);
            motorLeft.setPower(0);

            if (debugLevel0)
                DbgLog.msg("robotTurnAngleSidePivot :- End ");

            return true;
        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
