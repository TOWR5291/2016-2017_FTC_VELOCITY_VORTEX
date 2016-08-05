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
public class zoeOpmodeColorTest extends LinearOpMode {

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

        // wait for the start button to be pressed.
        waitForStart();

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x || gamepad2.x;

            // check for button state transitions.
            if (bCurrState == true && bCurrState != bPrevState)  {
                // button is transitioning to a pressed state.

                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was pressed!");

                // update previous state variable.
                bPrevState = bCurrState;

                // turn on the LED.
                cdim.setDigitalChannelState(COLORLED_CHANNEL, bEnabled);

            } else if (bCurrState == false && bCurrState != bPrevState)  {
                // button is transitioning to a released state.

                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was released!");

                // update previous state variable.
                bPrevState = bCurrState;

                // turn off the LED.
                cdim.setDigitalChannelState(COLORLED_CHANNEL, bDisabled);
            }

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

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
//        float throttle = -gamepad1.left_stick_y;
//        float direction = gamepad1.left_stick_x;
//        float right = throttle - direction;
//        float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
//        right = Range.clip(right, -1, 1);
//        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
//        right = (float)scaleInput(right);
//        left =  (float)scaleInput(left);

        // write the values to the motors
//        motorRight.setPower(right);
//        motorLeft.setPower(left);
/*
            // Gamepad 1 Button A -
            if (gamepad1.a) {
                telemetry.addData("Text", "Pad 1 - A Pressed");
                // if the A button is pushed on gamepad1
            }

            // Gamepad 1 Button B -
            if (gamepad1.b) {
                telemetry.addData("Text", "Pad 1 - B Pressed");
                // if the B button is pushed on gamepad1
            }

            // Gamepad 1 Button X -
            if (gamepad1.x) {
                telemetry.addData("Text", "Pad 1 - X Pressed");
                // if the X button is pushed on gamepad1
            }

            // Gamepad 1 Button Y -
            if (gamepad1.y) {
                telemetry.addData("Text", "Pad 1 - Y Pressed");
                // if the Y button is pushed on gamepad1
            }

            // Gamepad 1 Button DPad Up -
            if (gamepad1.dpad_up) {
                telemetry.addData("Text", "Pad 1 - DPad Up");
                // if the DPad Up button is pushed on gamepad1
            }

            // Gamepad 1 Button DPad Down -
            if (gamepad1.dpad_down) {
                telemetry.addData("Text", "Pad 1 - DPad Down");
                // if the DPad Down button is pushed on gamepad1
            }

            // Gamepad 1 Button DPad Left -
            if (gamepad1.dpad_left) {
                telemetry.addData("Text", "Pad 1 - DPad Left");
                // if the DPad Left button is pushed on gamepad1
            }

            // Gamepad 1 Button DPad Right -
            if (gamepad1.dpad_right) {
                telemetry.addData("Text", "Pad 1 - DPad Right");
                // if the DPad Right button is pushed on gamepad1
            }

            // Gamepad 1 Button Left Bump -
            if (gamepad1.left_bumper) {
                telemetry.addData("Text", "Pad 1 - Left Bumper");
                // if the Left Bumper button is pushed on gamepad1
            }

            // Gamepad 1 Button Right Bump -
            if (gamepad1.right_bumper) {
                telemetry.addData("Text", "Pad 1 - Right Bumper");
                // if the right Bumper button is pushed on gamepad1
            }

            // Gamepad 2 Button A -
            if (gamepad2.a) {
                telemetry.addData("Text", "Pad 2 - A Pressed");
                // if the A button is pushed on gamepad2
            }

            // Gamepad 2 Button B -
            if (gamepad2.b) {
                telemetry.addData("Text", "Pad 2 - B Pressed");
                // if the B button is pushed on gamepad2
            }

            // Gamepad 2 Button X -
            if (gamepad2.x) {
                telemetry.addData("Text", "Pad 2 - X Pressed");
                // if the X button is pushed on gamepad2
            }

            // Gamepad 2 Button Y -
            if (gamepad2.y) {
                telemetry.addData("Text", "Pad 2 - Y Pressed");
                // if the Y button is pushed on gamepad2
            }

            // Gamepad 2 Button DPad Up -
            if (gamepad2.dpad_up) {
                telemetry.addData("Text", "Pad 2 - DPad Up");
                // if the DPad Up button is pushed on gamepad2
            }

            // Gamepad 2 Button DPad Down -
            if (gamepad2.dpad_down) {
                telemetry.addData("Text", "Pad 2 - DPad Down");
                // if the DPad Down button is pushed on gamepad2
            }

            // Gamepad 2 Button DPad Left -
            if (gamepad2.dpad_left) {
                telemetry.addData("Text", "Pad 2 - DPad Left");
                // if the DPad Left button is pushed on gamepad2
            }

            // Gamepad 2 Button DPad Right -
            if (gamepad2.dpad_right) {
                telemetry.addData("Text", "Pad 2 - DPad Right");
                // if the DPad Right button is pushed on gamepad2
            }

            // Gamepad 2 Button Left Bump -
            if (gamepad2.left_bumper) {
                telemetry.addData("Text", "Pad 2 - Left Bumper");
                // if the Left Bumper button is pushed on gamepad2
            }

            // Gamepad 2 Button Right Bump -
            if (gamepad2.right_bumper) {
                telemetry.addData("Text", "Pad 2 - Right Bumper");
                // if the right Bumper button is pushed on gamepad2
            }
*/
            /*
             * Send telemetry data back to driver station. Note that if we are using
             * a legacy NXT-compatible motor controller, then the getPower() method
             * will return a null value. The legacy NXT-compatible motor controllers
             * are currently write only.
             */
            //telemetry.addData("Text", "*** Robot Data***");
            //telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
            //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        }

    }
}
