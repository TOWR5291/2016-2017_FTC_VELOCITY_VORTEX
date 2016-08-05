/*m
This opCode written by Zoe Haden for controlling the robot for team 5291

Date Created:- 11/121/2015
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 *
 * Autonomous mode TeleOp.
 * Robot should run for 30 seconds
 * Move from Home position, detect beacon light, press button, the climb mountain.
 */
public class autonomousredleft extends LinearOpMode {
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

    int motordeviceModeRead = 1;
    int motordeviceModeWrite = 2;
    double countsPerInch = 130;

    // color sensor
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    String TeamColor = "Red"; //what team is it?
    boolean red = false; //is the red left or not
    static final int COLORLED_CHANNEL = 0;

    // TETRIX VALUES.
    final static double SERVORIGHT_MIN_RANGE  = 0;
    final static double SERVORIGHT_MAX_RANGE  = 1.0;
    final static double SERVOLEFT_MIN_RANGE  = 0;
    final static double SERVOLEFT_MAX_RANGE  = 1.0;
    final static double SERVOARM_MIN_RANGE  = 0;
    final static double SERVOARM_MAX_RANGE  = 1.0;

    double servoRightPosition;
    double servoLeftPosition;
    double servoArmPosition;

    // line sensor
    int whiteLineReferenceValue;
    int lineTrackSensorLeft;
    int lineTrackSensorCenter;
    int lineTrackSensorRight;
    int lineSensorLeftOffset = 0;
    int lineSensorRightOffset = 0;
    int lineSensorCenterOffset = 0;
    boolean moveRight = false;
    boolean nudgeRight = false;
    boolean nudgeLeft = false;
    boolean moveLeft = false;
    boolean centered = false;
    boolean centerLineDetected = false;
    boolean leftLineDetected = false;
    boolean rightLineDetected = false;

    // the servos are on the motor controller
    Servo servoRight;
    Servo servoLeft;
    Servo servoArm;

    // servo controller device
    ServoController servodevice;

    // the motors are on the motor controller
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor armraise;                       //Arm mover
    DcMotor armextend;

    // motor controller device
    DcMotorController motordevice1;
    DcMotorController motordevice2;

    int ArmExtendPosition;
    int ArmRaisePosition;
    int maxArmExtension = 5000;
    int maxArmHeight = 5000;
    boolean stateOK;

    //gyro sensor stuff
    GyroSensor sensorGyro;
    int xVal, yVal, zVal = 0;
    int heading = 0;
    int newHeading;

//    public void LEDStatus (boolean Red, boolean Green, boolean Blue) {
//        if (debugLevel0)
//            DbgLog.msg("LEDStatus :- Start ");
//
//        if (Red == true)
//            cdim.setDigitalChannelState(REDLED_CHANNEL, bEnabled);
//        else
//            cdim.setDigitalChannelState(REDLED_CHANNEL, bDisabled);
//        if (Blue == true)
//            cdim.setDigitalChannelState(BLUELED_CHANNEL, bEnabled);
//        else
//            cdim.setDigitalChannelState(BLUELED_CHANNEL, bDisabled);
//        if (Green == true)
//            cdim.setDigitalChannelState(GREENLED_CHANNEL, bEnabled);
//        else
//            cdim.setDigitalChannelState(GREENLED_CHANNEL, bDisabled);
//        if (debugLevel0)
//            DbgLog.msg("LEDStatus :- End ");
//    }

//    public boolean robotTurnAngleCenterPivot (double power, int angle) {
//        if (debugLevel0)
//            DbgLog.msg("robotTurnAngleCenterPivot :- Start ");
//
//        int countsToMove;            //the calculated counts required to move the distance requested
//        int motorRightPos = 0;       //the current position position for the right motor in encoder counts
//        int motorLeftPos = 0;        //the current position for the left  motor in encoder counts
//        int motorRightTargetPos;     //the calculated target position for the right motor in encoder counts
//        int motorLeftTargetPos;      //the calculated target position for the left  motor in encoder counts
//        int targetAccuracy = 3;      //actual is double this, as it is plus minus this number
//        boolean targetGood = false;  //used to end this function, when robot has moved to the position this will go true and the function will exit
//        boolean notSameReading = true;
//        int motorRightPosCompare [] = { 0, 0, 0};
//        int motorLeftPosCompare [] = { 0, 0, 0};
//
//        if (angle == 0) {
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleCenterPivot :- No Angle to Turn, exiting.");
//            if (debugLevel0)
//                DbgLog.msg("robotTurnAngleCenterPivot :- End ");
//            return true;
//        } else {
//            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
//            switchMotorControllerEncoderMode(1,1);
//            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
//            switchMotorControllerEncoderMode(1,2);
//            // change controller to read mode so we can get the current position - should be zero since we reset the encoders
//            // probably don't need to even do half this now we reset the encoders for every step we are going to take.
//
//            switchMotorControllerMode(motordeviceModeRead);
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleCenterPivot :- About to read encoders");
//            // find out the encoder positions now
//            motorRightPos = motorRight.getCurrentPosition();
//            motorLeftPos = motorLeft.getCurrentPosition();
//            // find out the encoder positions now
//            // get 3 readings the same before determining if the reading is accurate, have seen strange things without doing this
//            while (notSameReading) {
//                for (int loop = 0; loop < 3; loop++) {
//                    motorRightPosCompare[loop] = motorRight.getCurrentPosition();
//                    motorLeftPosCompare[loop] = motorLeft.getCurrentPosition();
//                    try {
//                        sleep(50);
//                    } catch (Exception ex) {
//                    }
//                }
//                if (((motorRightPosCompare[0] == motorRightPosCompare[1]) && (motorRightPosCompare[1] == motorRightPosCompare[2])) &&
//                        ((motorLeftPosCompare[0] == motorLeftPosCompare[1]) && (motorLeftPosCompare[1] == motorLeftPosCompare[2])))
//                {
//                    motorRightPos = motorRightPosCompare[0];
//                    motorLeftPos = motorLeftPosCompare[0];
//                    notSameReading = false;
//                }
//            }
//
//            if (angle > 0) {
//                countsToMove = (countsPerDegreeCenterPivot * angle);
//                switchMotorControllerMode(motordeviceModeWrite);
//                if (power < 0) {
//                    countsToMove = countsToMove * -1;
//                }
//                motorRightTargetPos = motorRightPos - countsToMove;
//                motorLeftTargetPos = motorLeftPos + countsToMove;
//            } else {
//                countsToMove = (countsPerDegreeCenterPivot * angle);
//                switchMotorControllerMode(motordeviceModeWrite);
//                if (power < 0) {
//                    countsToMove = countsToMove * -1;
//                }
//                motorRightTargetPos = motorRightPos + countsToMove;
//                motorLeftTargetPos = motorLeftPos - countsToMove;
//            }
//            if (debugLevel1) {
//                DbgLog.msg("robotTurnAngleCenterPivot :- motorRightTargetPos " + motorRightTargetPos);
//                DbgLog.msg("robotTurnAngleCenterPivot :- motorLeftTargetPos " + motorLeftTargetPos);
//                DbgLog.msg("robotTurnAngleCenterPivot :- About to set encoder targets for move");
//            }
//
//            motorRight.setTargetPosition(motorRightTargetPos);
//            motorLeft.setTargetPosition(motorLeftTargetPos);
//
//            motorRight.setPower(0.3);
//            motorLeft.setPower(0.3);
//
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleCenterPivot :- Power set, should be moving");
//
//            //wait a bit to let the commands go active
//            try {
//                if (debugLevel1)
//                    DbgLog.msg("robotTurnAngleCenterPivot :- Motors Busy? ");
//                sleep(200);
//            } catch (Exception ex) {
//                if (debugLevel1)
//                    DbgLog.msg("robotTurnAngleCenterPivot :- Error ");
//            }
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleCenterPivot :- About to read encoders ");
//
//            switchMotorControllerMode(motordeviceModeRead);
//
//            while (!targetGood) {
//                motorRightPos = motorRight.getCurrentPosition();
//                motorLeftPos = motorLeft.getCurrentPosition();
//                if (((motorRightPos >= (motorRightTargetPos - targetAccuracy)) && (motorRightPos <= (motorRightTargetPos + targetAccuracy))) &&
//                        ((motorLeftPos >= (motorLeftTargetPos - targetAccuracy)) && (motorLeftPos <= (motorLeftTargetPos + targetAccuracy))))
//                {
//                    if (debugLevel1)
//                        DbgLog.msg("robotTurnAngleCenterPivot :- Angle achieved, powering down motors ");
//                    targetGood = true;
//                } else {
//                    //if (debugLevel1) {
//                    //    DbgLog.msg("robotTurnAngleCenterPivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos);
//                    //    DbgLog.msg("robotTurnAngleCenterPivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos);
//                    //}
//                    //telemetry.addData("robotTurnAngleCenterPivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos , "");
//                    //telemetry.addData("robotTurnAngleCenterPivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos, "");
//                }
//            }
//
//            switchMotorControllerMode(motordeviceModeWrite);
//            motorRight.setPower(0);
//            motorLeft.setPower(0);
//
//            if (debugLevel0)
//                DbgLog.msg("robotTurnAngleCenterPivot :- End ");
//            return true;
//        }
//    }

    public boolean robotTurnAngleCenterPivot (double power, int angle) {
        if (debugLevel0)
            DbgLog.msg("robotTurnAngleCenterPivot :- Start ");

        double countsToMove;            //the calculated counts required to move the distance requested
        boolean correctDirection = false;
        int headingDifference;

        if (angle == 0) {
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleCenterPivot :- No Angle to Turn, exiting.");
            if (debugLevel0)
                DbgLog.msg("robotTurnAngleCenterPivot :- End ");
            return true;
        } else {
            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(1,4);

            heading = sensorGyro.getHeading();

            double powerSetting = (Math.abs(power)) ;

            if (angle > 0) {
                while (!(correctDirection) && (opModeIsActive()))
                {
                    motorRight.setPower(-powerSetting);
                    motorLeft.setPower(powerSetting);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- Current Heading " + heading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- New Heading " + newHeading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- Requested Change " + angle);
                    telemetry.addData("St  Heading", heading);
                    telemetry.addData("New Heading", newHeading);
                    telemetry.addData("Req Angle  ", angle);
                    headingDifference = heading - angle;
                    telemetry.addData("Difference  ", headingDifference);
                    if ((headingDifference <= ( 2)) && (headingDifference >= (- 2))){
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        telemetry.addData("HitIt  ", "Yeah");
                        correctDirection = true;
                    } else if (((headingDifference <= 10)) && (headingDifference >= (- 10))){
                        motorRight.setPower(0.3 * (powerSetting));
                        motorLeft.setPower(-0.3 * (powerSetting));
                        telemetry.addData("Nearly There ", "Yeah");
                    }
                    try {
                        sleep(75);
                    } catch (Exception ex) {
                    }
                    heading = sensorGyro.getHeading();
                    if (!opModeIsActive()) {
                        correctDirection = true;
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        break;
                    }
                }

            } else {
                while (!(correctDirection) && (opModeIsActive()))
                {
                    motorRight.setPower(powerSetting);
                    motorLeft.setPower(-powerSetting);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- Current Heading " + heading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- New Heading " + newHeading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleCenterPivot :- Requested Change " + angle);
                    telemetry.addData("St  Heading", heading);
                    telemetry.addData("New Heading", newHeading);
                    telemetry.addData("Req Angle  ", angle);
                    headingDifference = heading - angle;
                    telemetry.addData("Difference  ", headingDifference);
                    if ((headingDifference <= ( 2)) && (headingDifference >= (- 2))){
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        telemetry.addData("HitIt  ", "Yeah");
                        correctDirection = true;
                    } else if (((headingDifference <= 10)) && (headingDifference >= (- 10))){
                        motorRight.setPower(0.3 * (powerSetting));
                        motorLeft.setPower(-0.3 * (powerSetting));
                        telemetry.addData("Nearly There ", "Yeah");
                    }
                    try {
                        sleep(75);
                    } catch (Exception ex) {
                    }
                    heading = sensorGyro.getHeading();
                    if (!opModeIsActive()) {
                        correctDirection = true;
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        break;
                    }
                }
            }

            if (debugLevel0)
                DbgLog.msg("robotTurnAngleCenterPivot :- End ");
            return true;
        }
    }

    public boolean robotTurnAngleSidePivot (double power, int angle) {
        if (debugLevel0)
            DbgLog.msg("robotTurnAngleSidePivot :- Start ");

        int motorRightPos = 0;       //the current position position for the right motor in encoder counts
        int motorLeftPos = 0;        //the current position for the left  motor in encoder counts
        int targetAccuracy = 3;      //actual is double this, as it is plus minus this number
        boolean targetGood = false;  //used to end this function, when robot has moved to the position this will go true and the function will exit
        boolean notSameReading = true;
        boolean correctDirection = false;
        //get current direction in degrees
        heading = sensorGyro.getHeading();
        DbgLog.msg("robotTurnAngleSidePivot :- heading " + heading);

        if (angle == 0) {
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- No Angle to Turn, exiting.");
            if (debugLevel0)
                DbgLog.msg("robotTurnAngleSidePivot :- End ");
            return true;
        } else {

            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(1,4);

            newHeading = Math.abs(heading - angle);
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- Start Heading " + heading);
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- New Heading " + newHeading);
            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- Requested Change " + angle);
            telemetry.addData("St  Heading", heading);
            telemetry.addData("New Heading", newHeading);
            telemetry.addData("Req Angle  ", angle);

            if (angle < 0) {  // Pivot on right side of robot
                while (!correctDirection && opModeIsActive())
                {
                    motorRight.setPower(0);
                    motorLeft.setPower(Math.abs(power));
                    heading = sensorGyro.getHeading();
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- Current Heading " + heading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- New Heading " + newHeading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- Requested Change " + angle);
                    telemetry.addData("St  Heading", heading);
                    telemetry.addData("New Heading", newHeading);
                    telemetry.addData("Req Angle  ", angle);
                    if ((heading <= (newHeading + 10)) && (heading <= (newHeading - 10))){
                        motorRight.setPower(0);
                        motorLeft.setPower(0.5 * Math.abs(power));
                    } else if ((heading <= (newHeading + 2)) && (heading <= (newHeading - 2))){
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        correctDirection = true;
                    }

                    try {
                        sleep(75);
                        } catch (Exception ex) {
                    }
                }
            } else { // Pivot on left side of robot
                while (!correctDirection && opModeIsActive())
                {
                    motorRight.setPower(Math.abs(power));
                    motorLeft.setPower(0);
                    heading = sensorGyro.getHeading();
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- Current Heading " + heading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- New Heading " + newHeading);
                    if (debugLevel1)
                        DbgLog.msg("robotTurnAngleSidePivot :- Requested Change " + angle);
                    telemetry.addData("St  Heading", heading);
                    telemetry.addData("New Heading", newHeading);
                    telemetry.addData("Req Angle  ", angle);
                    if ((heading <= (newHeading + 10)) && (heading <= (newHeading - 10))){
                        motorRight.setPower(0.5 * Math.abs(power));
                        motorLeft.setPower(0);
                    } else if ((heading <= (newHeading + 2)) && (heading <= (newHeading - 2))){
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        correctDirection = true;
                    }

                    try {
                        sleep(75);
                        } catch (Exception ex) {
                    }
                }
            }

            if (debugLevel1)
                DbgLog.msg("robotTurnAngleSidePivot :- Power set, should be moving");

            //wait a bit to let the commands go active
            try {
                if (debugLevel1)
                    DbgLog.msg("robotTurnAngleSidePivot :- Motors Busy? ");
                sleep(200);
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("robotTurnAngleSidePivot :- Error ");
            }

            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
            switchMotorControllerEncoderMode(1,2);

            if (debugLevel0)
                DbgLog.msg("robotTurnAngleSidePivot :- End ");

            return true;
        }
    }
//
//    public boolean robotTurnAngleSidePivot (double power, int angle) {
//        if (debugLevel0)
//            DbgLog.msg("robotTurnAngleSidePivot :- Start ");
//
//        int countsToMove;            //the calculated counts required to move the distance requested
//        int motorRightPos = 0;       //the current position position for the right motor in encoder counts
//        int motorLeftPos = 0;        //the current position for the left  motor in encoder counts
//        int motorRightTargetPos;     //the calculated target position for the right motor in encoder counts
//        int motorLeftTargetPos;      //the calculated target position for the left  motor in encoder counts
//        int targetAccuracy = 3;      //actual is double this, as it is plus minus this number
//        boolean targetGood = false;  //used to end this function, when robot has moved to the position this will go true and the function will exit
//        boolean notSameReading = true;
//        int motorRightPosCompare [] = { 0, 0, 0};
//        int motorLeftPosCompare [] = { 0, 0, 0};
//
//        if (angle == 0) {
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleSidePivot :- No Angle to Turn, exiting.");
//            if (debugLevel0)
//                DbgLog.msg("robotTurnAngleSidePivot :- End ");
//            return true;
//        } else {
//
//            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
//            switchMotorControllerEncoderMode(1,1);
//            // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
//            switchMotorControllerEncoderMode(1,2);
//
//            //switchMotorControllerMode(motordeviceModeRead);
//
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleSidePivot :- About to read encoders");
//
//            motorRightPos = motorRight.getCurrentPosition();
//            motorLeftPos = motorLeft.getCurrentPosition();
//            // find out the encoder positions now
//            // get 3 readings the same before determining if the reading is accurate, have seen strange things without doing this
//
//            while (notSameReading) {
//                for (int loop = 0; loop < 3; loop++) {
//                    motorRightPosCompare[loop] = motorRight.getCurrentPosition();
//                    motorLeftPosCompare[loop] = motorLeft.getCurrentPosition();
//                    try {
//                        sleep(50);
//                    } catch (Exception ex) {
//                    }
//                }
//                if (((motorRightPosCompare[0] == motorRightPosCompare[1]) && (motorRightPosCompare[1] == motorRightPosCompare[2])) &&
//                        ((motorLeftPosCompare[0] == motorLeftPosCompare[1]) && (motorLeftPosCompare[1] == motorLeftPosCompare[2])))
//                {
//                    motorRightPos = motorRightPosCompare[0];
//                    motorLeftPos = motorLeftPosCompare[0];
//                    notSameReading = false;
//                }
//            }
//            if (power > 0) {
//                countsToMove = (countsPerDegreeSidePivotFor * angle);
//            } else {
//                countsToMove = -1 * (countsPerDegreeSidePivotRev * angle);
//            }
//
//
//            if (angle > 0) {  // Pivot on right side of robot
//                motorRightTargetPos = motorRightPos;
//                motorLeftTargetPos = motorLeftPos + countsToMove;
//
//                switchMotorControllerMode(motordeviceModeWrite);
//                if (debugLevel1) {
//                    DbgLog.msg("robotTurnAngleSidePivot :- motorRightTargetPos " + motorRightTargetPos);
//                    DbgLog.msg("robotTurnAngleSidePivot :- motorLeftTargetPos " + motorLeftTargetPos);
//                    DbgLog.msg("robotTurnAngleSidePivot :- About to set encoder targets for move");
//                }
//
//                motorRight.setTargetPosition(motorRightTargetPos);
//                motorLeft.setTargetPosition(motorLeftTargetPos);
//
//                motorRight.setPower(Math.abs(power));
//                motorLeft.setPower(Math.abs(power));
//            } else { // Pivot on left side of robot
//                motorRightTargetPos = motorRightPos + countsToMove;
//                motorLeftTargetPos = motorLeftPos;
//                switchMotorControllerMode(motordeviceModeWrite);
//
//                if (debugLevel1) {
//                    DbgLog.msg("robotTurnAngleSidePivot :- motorRightTargetPos " + motorRightTargetPos);
//                    DbgLog.msg("robotTurnAngleSidePivot :- motorLeftTargetPos " + motorLeftTargetPos);
//                    DbgLog.msg("robotTurnAngleSidePivot :- About to set encoder targets for move");
//                }
//
//                motorRight.setTargetPosition(motorRightTargetPos);
//                motorLeft.setTargetPosition(motorLeftTargetPos);
//
//                motorRight.setPower(Math.abs(power));
//                motorLeft.setPower(Math.abs(power));
//            }
//
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleSidePivot :- Power set, should be moving");
//
//            //wait a bit to let the commands go active
//            try {
//                if (debugLevel1)
//                    DbgLog.msg("robotTurnAngleSidePivot :- Motors Busy? ");
//                sleep(200);
//            } catch (Exception ex) {
//                if (debugLevel1)
//                    DbgLog.msg("robotTurnAngleSidePivot :- Error ");
//            }
//            if (debugLevel1)
//                DbgLog.msg("robotTurnAngleSidePivot :- About to read encoders ");
//
//            switchMotorControllerMode(motordeviceModeRead);
//
//            while (!targetGood) {
//                motorRightPos = motorRight.getCurrentPosition();
//                motorLeftPos = motorLeft.getCurrentPosition();
//                if (((motorRightPos >= (motorRightTargetPos - targetAccuracy)) && (motorRightPos <= (motorRightTargetPos + targetAccuracy))) &&
//                        ((motorLeftPos >= (motorLeftTargetPos - targetAccuracy)) && (motorLeftPos <= (motorLeftTargetPos + targetAccuracy))))
//                {
//                    if (debugLevel1)
//                        DbgLog.msg("robotTurnAngleSidePivot :- Angle achieved, powering down motors ");
//                    targetGood = true;
//                } else {
//                    //if (debugLevel1) {
//                    //    DbgLog.msg("robotTurnAngleSidePivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos);
//                    //    DbgLog.msg("robotTurnAngleSidePivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos);
//                    //}
//                    //telemetry.addData("robotTurnAngleSidePivot :- Target Right " + motorRightTargetPos + " Target Left " + motorLeftTargetPos , "");
//                    //telemetry.addData("robotTurnAngleSidePivot :- Actual Right " + motorRightPos + " Actual Left " + motorLeftPos, "");
//                }
//            }
//
//            switchMotorControllerMode(motordeviceModeWrite);
//
//            motorRight.setPower(0);
//            motorLeft.setPower(0);
//
//            if (debugLevel0)
//                DbgLog.msg("robotTurnAngleSidePivot :- End ");
//
//            return true;
//        }
//    }
//

    public boolean specialStep (int function) {

        float left;
        float right;
        float throttle;
        float direction;

        switch (function) {

            case 1: {

            }
            break;

            case 2: {

                System.out.println("ColorSensor code initialized!");

                boolean colorComplete = false;

                final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

                // create
                ColorSensor colorSensorRGB;   //setup variables

                // display color
                float hsvValues[] = {0F, 0F, 0F};

                // get a reference to our ColorSensor object.
                colorSensorRGB = hardwareMap.colorSensor.get("color");
                cdim.setDigitalChannelState(COLORLED_CHANNEL, bEnabled);

                //move servo to left side
                moveServo3(37.3);

                while (!colorComplete && opModeIsActive()) {

                    System.out.println("ColorSensor Loop1 Initalized");

                    telemetry.addData("Clear", colorSensorRGB.alpha());
                    telemetry.addData("Red  ", colorSensorRGB.red());
                    telemetry.addData("Green", colorSensorRGB.green());
                    telemetry.addData("Blue ", colorSensorRGB.blue());
                    telemetry.addData("Hue", hsvValues[0]);

                    final float values[] = hsvValues;

                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                        }
                    });

                    if (colorSensorRGB.alpha() < 10000) {

                        System.out.println("Error, White Detected");
                    } else {
                        if ((colorSensorRGB.blue() > 2000) && (colorSensorRGB.red() > 1000)) {
                            if (colorSensorRGB.blue() < colorSensorRGB.red()) {
                                //color is red

                                telemetry.addData("Color", "- RED");
                                red = true;
                                colorComplete = true;
                            } else if (colorSensorRGB.red() < colorSensorRGB.blue()) {
                                //color is blue

                                telemetry.addData("Color", "- BLUE");
                                red = false;
                                colorComplete = true;
                            }
                        } else {
                            //color is not red or blue

                            telemetry.addData("Color", "- NONE");
                        }
                    }
                }

                colorComplete = false;
                //move servo to right side
                moveServo3((180 - 37.3));
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }


                while (!colorComplete && opModeIsActive()) {

                    System.out.println("ColorSensor code initialized!");

                    telemetry.addData("Clear", colorSensorRGB.alpha());
                    telemetry.addData("Red  ", colorSensorRGB.red());
                    telemetry.addData("Green", colorSensorRGB.green());
                    telemetry.addData("Blue ", colorSensorRGB.blue());
                    telemetry.addData("Hue", hsvValues[0]);

                    final float values[] = hsvValues;

                    relativeLayout.post(new Runnable() {
                        public void run() {
                            relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                        }
                    });

                    if (colorSensorRGB.alpha() < 10000) {

                        System.out.println("Error, White Detected");
                    } else {

                        if ((colorSensorRGB.blue() > 1100) && (colorSensorRGB.red() > 1000)) {
                            if (colorSensorRGB.blue() < colorSensorRGB.red()) {
                                //color is red

                                telemetry.addData("Color", "- RED");
                                colorComplete = true;
                                if (red) {
                                    System.out.println("ERROR IN COLOR SENSOR!!!! BOTH SIDES SCANNED AS RED!!!!");
                                    colorComplete = false;
                                }
                            } else if (colorSensorRGB.red() < colorSensorRGB.blue()) {
                                //color is blue

                                telemetry.addData("Color", "- BLUE");
                                colorComplete = true;
                                if (!red) {
                                    System.out.println("ERROR IN COLOR SENSOR!!!! BOTH SIDES SCANNED AS BLUE!!!!");
                                    colorComplete = false;
                                }
                            }
                        } else {
                            //color is not red or blue

                            telemetry.addData("Color", "- NONE");
                        }

                    }

                }

//which side
                if (red && TeamColor.equals("Red")) { // red left
                    //Button left
                    moveServo3(0);
                    //Move Forward
                    robotMoveDistance(0.3, 1);
                    //Debug

                    telemetry.addData("ColorServo", "- RedLeft");
                    System.out.println("Red Left ColorSensor");
                } else if (!red && TeamColor.equals("Red")) { // red right
                    //Button right
                    moveServo3(180);
                    //Move Forward
                    robotMoveDistance(0.3, 1);
                    //Debug
                    System.out.println("Red Right ColorSensor");
                    telemetry.addData("ColorServo", "- RedRight");
                } else if (red && TeamColor.equals("Blue")) { //blue right
                    //Button right
                    moveServo3(180);
                    //Move Forward
                    robotMoveDistance(0.3, 1);
                    //Debug

                    telemetry.addData("ColorServo", "- BlueRight");
                    System.out.println("Blue Right ColorSensor");
                } else if (!red && TeamColor.equals("Blue")) { //blue left
                    //Button left
                    moveServo3(0);
                    //Move Forward
                    robotMoveDistance(0.3, 1);
                    //Debug

                    telemetry.addData("ColorServo", "- BlueLeft");
                    System.out.println("Blue Left ColorSensor");
                } else { //Error detection
                    //Debug
                    telemetry.addData("ColorServo", "- No If Reached");
                    System.out.println("No If Reached!!!!!!!! ColorSensor");
                }
            }

            cdim.setDigitalChannelState(COLORLED_CHANNEL, bDisabled);
            break;

            case 4: {
                telemetry.addData("Gyro", "Start");
                while (opModeIsActive()) {
                    // get the x, y, and z values (rate of change of angle).
                    xVal = sensorGyro.rawX();
                    yVal = sensorGyro.rawY();
                    zVal = sensorGyro.rawZ();

                    // get the heading info.
                    // the Modern Robotics' gyro sensor keeps
                    // track of the current heading for the Z axis only.
                    heading = sensorGyro.getHeading();

                    telemetry.addData("1. x", String.format("%03d", xVal));
                    telemetry.addData("2. y", String.format("%03d", yVal));
                    telemetry.addData("3. z", String.format("%03d", zVal));
                    telemetry.addData("4. h", String.format("%03d", heading));

                    try {
                        sleep(50);
                        } catch (Exception ex) {
                    }
                }
            }
            break;

            case 5: {
                boolean tracking = true;
                switchMotorControllerMode(motordeviceModeWrite);
                // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
                switchMotorControllerEncoderMode(1,4);

                motorLeft.setDirection(DcMotor.Direction.FORWARD);
                motorRight.setDirection(DcMotor.Direction.REVERSE);

                motorRight.setPower(0);
                motorLeft.setPower(0);

                while (tracking && opModeIsActive()) {
                    lineTrackSensorLeft = cdim.getAnalogInputValue(1) + lineSensorLeftOffset;
                    //telemetry.addData("Left Sensor:- ", lineTrackSensorLeft);
                    lineTrackSensorCenter = cdim.getAnalogInputValue(2) + lineSensorCenterOffset;
                    //telemetry.addData("Center Sensor:- ", lineTrackSensorCenter);
                    lineTrackSensorRight = cdim.getAnalogInputValue(3) + lineSensorRightOffset;
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
                    telemetry.addData("Line Left", lineTrackSensorLeft);
                    telemetry.addData("Line Center", lineTrackSensorCenter);
                    telemetry.addData("Line Right", lineTrackSensorRight);
                    telemetry.addData("Line Ref", whiteLineReferenceValue);
                    telemetry.addData("Off Left", lineSensorLeftOffset);
                    telemetry.addData("Off Center", lineSensorCenterOffset);
                    telemetry.addData("Off Right", lineSensorRightOffset);

                    if ((rightLineDetected == true) && (rightLineDetected == centerLineDetected)) {
                        // check if right and center is on - nudge to the left only
                        // just off center
                        nudgeLeft = true;
                        telemetry.addData("Line Position", "Right Centered");
                    } else if ((rightLineDetected == true) && (rightLineDetected != centerLineDetected)) {
                        // check if right is on - move to the left
                        moveLeft = true;
                        telemetry.addData("Line Position", "Right");
                    } else if ((leftLineDetected == true) && (leftLineDetected == centerLineDetected)) {
                        // check if left and center is on - nudge to the right only
                        // just off center
                        nudgeRight = true;
                        telemetry.addData("Line Position", "Left Centered");
                    } else if ((leftLineDetected == true) && (leftLineDetected != centerLineDetected)) {
                        // check if left is on - move to the right
                        moveRight = true;
                        telemetry.addData("Line Position", "Left");
                    } else if (centerLineDetected) {
                        // check if we are centered, if this is not true we are not on the line

                        moveRight = false;
                        nudgeRight = false;
                        nudgeLeft = false;
                        moveLeft = false;
                        centered = true;
                        telemetry.addData("Line Position", "Centered");
                    }

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

                    right = throttle + direction;
                    left = throttle - direction;

                    telemetry.addData("Left Motor :- ", String.format("%.2f", left));
                    telemetry.addData("Right Motor:- ", String.format("%.2f", right));
                    // write the values to the motors
                    motorRight.setPower(right);
                    motorLeft.setPower(left);

                    if (gamepad1.y) {
                        tracking = false;
                    }
                }
                if (opModeIsActive()) {
                    //remove power from the motors
                    motorRight.setPower(0);
                    motorLeft.setPower(0);
                    //set motors back to the right direction
                    motorLeft.setDirection(DcMotor.Direction.REVERSE);
                    motorRight.setDirection(DcMotor.Direction.FORWARD);
                    // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
                    switchMotorControllerEncoderMode(1,2);
                }
            }
            break;

            case 99: {

            }
            break;

            default:
                break;

        }
        return true;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");

        initGyro();

        initServos();

        initMotors();

        initMotorEncoders();

        initLineTrack();

        // wait for the start button to be pressed.
        waitForStart();

        int step = 1;

        // the first position in the array is not used, as its position 0 or step
        // turing and moving in one step is allowed, turns happen first, then movement
        // align steps to           { 0,           1,    2,    3,    4,    5,    6,    7,    8,    9,   10,  11,  12,  13,  14,  15,  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50 };  // used for aligning steps to array position
        int centerTurnSteps[]     = { 0,           0,   45, -135,  90,     0,    0,    0,    0,    45,  45,  45, 180,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees, both tracks will drive (pivoting around the center of the robot)
        double movementSteps[]    = { 0,          30,-16.9,   12,    0,    0,-18.0,    0,    0,     0,   0,  10,   0,  10,  12,  24, -12,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in inches
        double servoRightSteps[]  = { 0,           0,    0,    0,    0,    0,    0,  155,  165,    90,   0,   0, 180,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees of rotation
        double servoLeftSteps[]   = { 0,           0,    0,    0,    0,    0,    0,  155,  165,     0,  90,   0,   0, 180,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in degrees of rotation
        double powerSteps[]       = { 0,           1,    1,    1,    1,    0,    1,  0.2,  0.2,   0.5, 0.5, 0.5, 0.5, 0.5, 0.7, 0.7,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // in percentage, max of 1, min of 0,
        int specialSteps[]        = { 0,           0,    0,    0,    5,    2,    0,    0,    2,     0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // special function to call, this way you can easily move them around without changing the case below

        boolean completed = false;
        boolean rotateCompleteCenter = false;
        boolean rotateCompleteSide = false;
        boolean moveComplete = false;
        boolean servoMoveComplete = false;
        boolean specialComplete = false;

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        {
            if (!(completed)) {
                for (step = 1; step < 20; step++) {
                    telemetry.addData("Step", " " + step + " START!");
                    telemetry.addData("TurnAngleCentPivot", " step " + step + " - " + centerTurnSteps[step]);
                    telemetry.addData("MoveDistance", " step " + step + " - " + movementSteps[step]);
                    telemetry.addData("MoveServos", " step " + step + " - " + servoRightSteps[step] + " " + servoLeftSteps[step]);
                    //telemetry.addData("MoveArm", " step " + step + " - " + armHeight[step] + " " + armExtension[step]);
                    telemetry.addData("specialStep", " step " + step + " - " + specialSteps[step]);
                    rotateCompleteCenter = robotTurnAngleCenterPivot(powerSteps[step], centerTurnSteps[step]);
                    moveComplete = robotMoveDistance(powerSteps[step], movementSteps[step]);
                    servoMoveComplete = (moveServo1(servoRightSteps[step]) && moveServo2(servoLeftSteps[step]));
//                        armMoveComplete = moveArm(armHeight[step]);
//                        armMoveComplete = moveArmExtension(armHeight[step]);
                    specialComplete = specialStep(specialSteps[step]);
                    if (moveComplete && rotateCompleteSide && rotateCompleteCenter && servoMoveComplete && specialComplete) {
                        telemetry.addData("Step", " " + step + " COMPLETE!");
                    } else {
                        telemetry.addData("Step", " " + step + " failed");
                    }
                    // wait a hardware cycle before iterating.
                    waitOneFullHardwareCycle();
                    if (!opModeIsActive()) {
                        completed = true;
                        step = 100;
                        motorRight.setPower(0);
                        motorLeft.setPower(0);
                        break;
                    }
                }
                completed = true;
                motorRight.setPower(0);
                motorLeft.setPower(0);
            }
        }
    }

    public void initGyro(){
        DbgLog.msg("initGyro :- Start ");
        // get a reference to our GyroSensor object.
        sensorGyro = hardwareMap.gyroSensor.get("gyro");

        // calibrate the gyro.
        sensorGyro.calibrate();

        while (sensorGyro.isCalibrating())  {
            //wait a bit to let the commands go active
            try {
                if (debugLevel1)
                    DbgLog.msg("initGyro :- Calibrating ");
                sleep(50);
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("initGyro :- Error ");
            }
        }

        //reset heading
        sensorGyro.resetZAxisIntegrator();

        DbgLog.msg("initGyro :- End ");
    }


    public void initLineTrack (){
        double averageReadings = 0;

        if (debugLevel0)
            DbgLog.msg("initLineTrack :- Start ");

        for (int loop = 1; loop < 5; loop++) {


            lineTrackSensorLeft = cdim.getAnalogInputValue(1) + lineSensorLeftOffset;
            //telemetry.addData("Left Sensor:- ", lineTrackSensorLeft);
            lineTrackSensorCenter = cdim.getAnalogInputValue(2) + lineSensorCenterOffset;
            //telemetry.addData("Center Sensor:- ", lineTrackSensorCenter);
            lineTrackSensorRight = cdim.getAnalogInputValue(3) + lineSensorRightOffset;
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
            averageReadings = ((double) lineTrackSensorCenter + (double) lineTrackSensorRight + (double) lineTrackSensorLeft) / 3;

            if (debugLevel1)
                DbgLog.msg("initLineTrack :- averageReadings = " + averageReadings);

            whiteLineReferenceValue = (int) averageReadings - 50;

            lineSensorLeftOffset = ((int)averageReadings - lineTrackSensorLeft);
            lineSensorRightOffset = ((int) averageReadings - lineTrackSensorRight);
            lineSensorCenterOffset = ((int)averageReadings - lineTrackSensorCenter);
        }

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
        servoRight = hardwareMap.servo.get("servoright");
        servoLeft = hardwareMap.servo.get("servoleft");
        servoArm = hardwareMap.servo.get("servofront");

        servoRight.setDirection(Servo.Direction.REVERSE);

        servoRightPosition = 0;
        servoLeftPosition = 0;
        servoArmPosition = 0;
        servoRight.setPosition(servoRightPosition);
        servoLeft.setPosition(servoLeftPosition);
        servoArm.setPosition(servoArmPosition);

        if (debugLevel0)
            DbgLog.msg("initServos :- End ");
    }

    public boolean moveServo1 (double Position) {
        boolean OKToMove = true;

        if (debugLevel0)
            DbgLog.msg("moveServos :- Start ");
        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < SERVORIGHT_MIN_RANGE ) || (Range.scale(Position, 0, 180, 0, 1) > SERVORIGHT_MAX_RANGE )) {
            if (debugLevel1)
                DbgLog.msg("moveServo1 :- Right Out Of Range, no move ");
            OKToMove = false;
        }

        if (OKToMove) {
            servoRight.setPosition(Range.scale(Position, 0, 180, 0, 1));
        }

        if (debugLevel0)
            DbgLog.msg("moveServo1 :- End ");
        return true;
    }

    public boolean moveServo2 (double Position) {
        boolean OKToMove = true;

        if (debugLevel0)
            DbgLog.msg("moveServo2 :- Start ");
        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < SERVOLEFT_MIN_RANGE ) || (Range.scale(Position, 0, 180, 0, 1) > SERVOLEFT_MAX_RANGE )) {
            if (debugLevel1)
                DbgLog.msg("moveServo2 :- Left Out Of Range, no move ");
            OKToMove = false;
        }

        if (OKToMove) {
            servoLeft.setPosition(Range.scale(Position, 0, 180, 0, 1));
        }

        if (debugLevel0)
            DbgLog.msg("moveServo2 :- End ");
        return true;
    }

    public boolean moveServo3 (double Position) {
        boolean OKToMove = true;

        if (debugLevel0)
            DbgLog.msg("moveServo3 :- Start ");
        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < SERVOARM_MIN_RANGE ) || (Range.scale(Position, 0, 180, 0, 1) > SERVOARM_MAX_RANGE )) {
            if (debugLevel1)
                DbgLog.msg("moveServo3 :- Right Out Of Range, no move ");
            OKToMove = false;
        }

        if (OKToMove) {
            servoArm.setPosition(Range.scale(Position, 0, 180, 0, 1));
        }

        if (debugLevel0)
            DbgLog.msg("moveServo3 :- End ");
        return true;
    }

    public void initMotors () {

        if (debugLevel0)
            DbgLog.msg("initMotors :- Start ");
        motordevice1 = hardwareMap.dcMotorController.get("motorcon1");
        motordevice2 = hardwareMap.dcMotorController.get("motorcon2");

        // Configuring the motors for control
        motorLeft = hardwareMap.dcMotor.get("motor1");
        motorRight = hardwareMap.dcMotor.get("motor2");
        armextend = hardwareMap.dcMotor.get("motor3");
        armraise = hardwareMap.dcMotor.get("motor4");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        armraise.setDirection(DcMotor.Direction.REVERSE);
        armextend.setDirection(DcMotor.Direction.REVERSE);

        if (debugLevel0)
            DbgLog.msg("initMotors :- End ");
    }

    public void initMotorEncoders () {

        boolean stateOK = false;

        if (debugLevel0)
            DbgLog.msg("initMotorEncoders :- Start ");

        // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
        switchMotorControllerEncoderMode(1, 1);
        switchMotorControllerEncoderMode(2, 1);  //want the arm motors to run with encoders

        //wait a bit to let the commands go active
        try {
            if (debugLevel1)
                DbgLog.msg("initMotorEncoders :- Busy? ");
            sleep(100);
        } catch (Exception ex) {
            if (debugLevel1)
                DbgLog.msg("initMotorEncoders :- Error ");
        }
        // Switch the motor controller to read mode
        //don't call this function too quickly, the controller crashes when changing modes too often
        stateOK = switchMotorControllerMode(motordeviceModeRead);

        // wait until motor encoders are reset
        while ((motorLeft.getCurrentPosition() !=0) || (motorRight.getCurrentPosition() !=0)) {
            try {
                sleep(100);
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
        switchMotorControllerEncoderMode(1 ,2);
        switchMotorControllerEncoderMode(2, 2);

        // Switch the motor controller to read mode
        //don't call this function too quickly, the controller crashes when changing modes too often
        stateOK = switchMotorControllerMode(motordeviceModeRead);

        if (debugLevel1)
            DbgLog.msg("initMotorEncoders - Encoders Reset, Complete RUN TO POSITION ");
        if (debugLevel0)
            DbgLog.msg("initMotorEncoders :- End ");
    }

    public void switchMotorControllerEncoderMode (int controller, int state) {
        boolean stateOK = false;
        //  states
        // 1 = RESET_ENCODERS
        // 2 = RUN_TO_POSITION
        // 3 = RUN_USING_ENCODERS
        // 4 = RUN_WITHOUT_ENCODERS

        if (debugLevel0)
            DbgLog.msg("switchMotorControllerEncoderMode :- Start ");

        // check we are in write mode, if not switch to write mode
        if (controller == 1)
            stateOK = switchMotorControllerMode(motordeviceModeWrite);
        if (controller == 2)
            stateOK = switchArmControllerMode(motordeviceModeWrite);

        if (controller == 1){
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
        }

        if (controller == 2){
            switch (state) {
                case 1:
                    armraise.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    armextend.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    break;
                case 2:
                    armraise.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    armextend.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    break;
                case 3:
                    armraise.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                    armextend.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                    break;
                case 4:
                    armraise.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    armextend.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    break;
                default:
                    break;
            }
        }

        if (debugLevel0)
            DbgLog.msg("switchMotorControllerEncoderMode :- End ");
    }

    public boolean switchArmControllerMode (int state) {
        if (debugLevel0)
            DbgLog.msg("switchArmControllerMode :- Start ");

        int currentState = 0;
        // valid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
        // State 1 = READ_ONLY
        // State 2 = WRITE_ONLY
        // State 3 = SWITCHING_TO_READ_MODE
        // State 4 = SWITCHING_TO_WRITE_MODE

        // valid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
        if (motordevice2.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            currentState = 1;
            if (debugLevel1)
                DbgLog.msg("switchArmControllerMode - Currentstate 1 - device mode: " + motordevice2.getMotorControllerDeviceMode());
            if (state == currentState) {
                if (debugLevel1)
                    DbgLog.msg("switchArmControllerMode - Currentstate 1 - device mode: " + motordevice2.getMotorControllerDeviceMode());
                if (debugLevel0)
                    DbgLog.msg("switchArmControllerMode :- End ");
                return true;  //already in the state we need, do nothing
            }
        }
        if (motordevice2.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY) {
            currentState = 2;
            if (debugLevel1)
                DbgLog.msg("switchArmControllerMode - Currentstate 2 - device mode: " + motordevice2.getMotorControllerDeviceMode());
            if (state == currentState) {
                if (debugLevel1)
                    DbgLog.msg("switchArmControllerMode - Currentstate 1 - device mode: " + motordevice2.getMotorControllerDeviceMode());
                if (debugLevel0)
                    DbgLog.msg("switchArmControllerMode :- End ");
                return true;  //already in the state we need, do nothing
            }
        }

        //looks like we need to change state.
        // add a little delay, the stupic controller doesn't like changing state too often and goes AWOL
        try {
            Thread.sleep(50);
            if (debugLevel1) {
                DbgLog.msg("switchArmControllerMode - Waiting for armcontroller change state ");
                //telemetry.addData("Waiting", " Setting Write_Only");
            }
        } catch (Exception ex) {
            telemetry.addData("Error", " Setting Write_Only");
        }

        if (state != currentState) {
            switch (state) {
                case 1: {
                    if (debugLevel1)
                        DbgLog.msg("switchArmControllerMode - Setting MotorController to Read ");
                    motordevice2.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice2.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
                        try {
                            Thread.sleep(50);
                            if (debugLevel1){
                                DbgLog.msg("switchArmControllerMode - Waiting for read mode ");
                                //telemetry.addData("Waiting", " Setting Read_Only");
                            }
                        } catch (Exception ex) {
                            telemetry.addData("Error", " Setting Read_Only");
                        }
                    }
                    if (debugLevel0)
                        DbgLog.msg("switchArmControllerMode :- End ");
                    return true;
                }
                //break;  //unreachable statement because of the return above
                case 2: {
                    if (debugLevel1)
                        DbgLog.msg("switchArmControllerMode - Setting ArmController to Write ");
                    motordevice2.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice2.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
                        try {
                            Thread.sleep(50);
                            if (debugLevel1) {
                                DbgLog.msg("switchArmControllerMode - Waiting for write mode ");
                                //telemetry.addData("Waiting", " Setting Write_Only");
                            }
                        } catch (Exception ex) {
                            telemetry.addData("Error", " Setting Write_Only");
                        }
                    }
                    if (debugLevel0)
                        DbgLog.msg("switchArmControllerMode :- End ");
                    return true;
                }
                //break;  //unreachable statement because of the return above
            }
        }
        return false;  //should never get here
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
        if (motordevice1.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            currentState = 1;
            if (debugLevel1)
                DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice1.getMotorControllerDeviceMode());
            if (state == currentState) {
                if (debugLevel1)
                    DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice1.getMotorControllerDeviceMode());
                if (debugLevel0)
                    DbgLog.msg("switchMotorControllerMode :- End ");
                return true;  //already in the state we need, do nothing
            }
        }
        if (motordevice1.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY) {
            currentState = 2;
            if (debugLevel1)
                DbgLog.msg("switchMotorControllerMode - Currentstate 2 - device mode: " + motordevice1.getMotorControllerDeviceMode());
            if (state == currentState) {
                if (debugLevel1)
                    DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice1.getMotorControllerDeviceMode());
                if (debugLevel0)
                    DbgLog.msg("switchMotorControllerMode :- End ");
                return true;  //already in the state we need, do nothing
            }
        }

        //looks like we need to change state.
        // add a little delay, the stupic controller doesn't like changing state too often and goes AWOL
        try {
            sleep(50);
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
                    motordevice1.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice1.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
                        try {
                            sleep(50);
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
                    motordevice1.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice1.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
                        try {
                            sleep(50);
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

    public boolean robotMoveDistance (double motorPower, double distance) {
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
            stateOK = switchMotorControllerMode(motordeviceModeRead);

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
                        sleep(50);
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

            countsToMove = (int)(distance * countsPerInch);
            //don't call this function too quickly, the controller crashes when changing modes too often
            stateOK = switchMotorControllerMode(motordeviceModeWrite);

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
                sleep(500);
            } catch (Exception ex) {
                if (debugLevel1)
                    DbgLog.msg("robotMoveDistance :- Error ");
            }
            if (debugLevel1)
                DbgLog.msg("robotMoveDistance :- About to read encoders ");
            //don't call this function too quickly, the controller crashes when changing modes too often
            stateOK = switchMotorControllerMode(motordeviceModeRead);
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
            stateOK = switchMotorControllerMode(motordeviceModeWrite);
            motorRight.setPower(0);
            motorLeft.setPower(0);
            if (debugLevel0)
                DbgLog.msg("robotMoveDistance :- End ");
            return true;
        }

    }
}
