/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


This opCode writtem by Zoe Haden for controlling the robot for team 5291
- controls drive motor left
- controls drive motor right

Date Created:- 11/14/2015


 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class linearOpmodeTest extends LinearOpMode {


    ColorSensor colorsensorRGB;
    DeviceInterfaceModule cdim;
    // The color sensor needs to be connected to the I2C port 0
    // The LED on the color sensor needs to be connected to digital pot D0

    static final int COLORLED_CHANNEL = 0;

    // bEnabled and bDisabled represents the state of the output pins.
    boolean bEnabled = true;
    boolean bDisabled = false;

    // bPrevState and bCurrState represent the previous and current state of the button on the GamePad.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    //
    int whiteLineReferenceValue = 300;
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
    float left;
    float right;
    float throttle;
    float direction;

    // the motors are on the motor controller
    DcMotor motorRight;
    DcMotor motorLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");

        // The color sensor has an LED, set the digital channel to output mode.
        cdim.setDigitalChannelMode(COLORLED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        colorsensorRGB = hardwareMap.colorSensor.get("color");

        //turn the color sensor LED on
        cdim.setDigitalChannelState(COLORLED_CHANNEL, bDisabled);

        // Configuring the motors for control
        motorRight = hardwareMap.dcMotor.get("motor1");
        motorLeft = hardwareMap.dcMotor.get("motor2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);


        // wait for the start button to be pressed.
        waitForStart();

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // Determine correct reference, reference will change depending on reflectivity and distance above surface
        // Refereance should be lower than the greatest value of the 3 sensors less a nominal difference of approx 50 units
        if (lineTrackSensorCenter > lineTrackSensorLeft && lineTrackSensorCenter > lineTrackSensorRight) {
            whiteLineReferenceValue = lineTrackSensorCenter - 50;
        }  else if (lineTrackSensorLeft > lineTrackSensorCenter && lineTrackSensorLeft > lineTrackSensorRight) {
            whiteLineReferenceValue = lineTrackSensorLeft - 50;
        } else if (lineTrackSensorRight > lineTrackSensorCenter && lineTrackSensorRight > lineTrackSensorLeft) {
            whiteLineReferenceValue = lineTrackSensorRight - 50;
        } else  if ((lineTrackSensorCenter >= 350) && (lineTrackSensorLeft >= 350) && (lineTrackSensorRight >= 350)) {
            whiteLineReferenceValue = 350;
        }

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //telemetry.addData("Text", "***Data***");

            lineTrackSensorLeft = cdim.getAnalogInputValue(1);
            //telemetry.addData("Left Sensor:- ", lineTrackSensorLeft);
            lineTrackSensorCenter = cdim.getAnalogInputValue(2);
            //telemetry.addData("Center Sensor:- ", lineTrackSensorCenter);
            lineTrackSensorRight = cdim.getAnalogInputValue(3);
            //telemetry.addData("Right Sensor:- ", lineTrackSensorRight);

            // Determine if white line has been detected
            // assume position is not known on every check
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
            } else if ((rightLineDetected == true) && (rightLineDetected== centerLineDetected)) {
                // check if right and center is on - nudge to the left only
                // just off center
                nudgeLeft = true;
                telemetry.addData("Line Position", "Right Centered");
            }else if ((rightLineDetected == true) && (rightLineDetected != centerLineDetected)) {
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

            telemetry.addData("Text", "***Data***");
            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x || gamepad2.x;

            // check for button state transitions.
            //if (bCurrState == true && bCurrState != bPrevState)  {
            if (bCurrState == true)  {
                // button is transitioning to a pressed state.

                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was pressed!");

                // update previous state variable.
                bPrevState = bCurrState;

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

            //} else if (bCurrState == false && bCurrState != bPrevState)  {
            } else if (bCurrState == false)  {
                // button is transitioning to a released state.

                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was released!");

                // update previous state variable.
                bPrevState = bCurrState;
                // write the values to the motors
                motorRight.setPower(0);
                motorLeft.setPower(0);
            }
            // turn off the LED.
            cdim.setDigitalChannelState(COLORLED_CHANNEL, bDisabled);

            // convert the RGB values to HSV values.
            Color.RGBToHSV((colorsensorRGB.red() * 255) / 800, (colorsensorRGB.green() * 255) / 800, (colorsensorRGB.blue() * 255) / 800, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", colorsensorRGB.alpha());
            telemetry.addData("Red  ", colorsensorRGB.red());
            telemetry.addData("Green", colorsensorRGB.green());
            telemetry.addData("Blue ", colorsensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();

        }

    }
}
