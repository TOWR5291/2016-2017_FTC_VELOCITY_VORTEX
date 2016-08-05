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

Color Sensor - AdaFruit RGB Sensor
- The sensor must be connected to the I2C port of the Core Device Module
- LED on the sensor needs to be connected the Core Device Module

This opCode writtem by Zoe Haden for controlling the robot for team 5291
- Reads color sensor
- reads all buttons from the gamepad
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
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
public class zoeOpmodeDriveTest extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */

    static final int raiseEnd1 = 7;
    static final int raiseEnd2 = 6;
    static final int extendEnd1 = 5;
    static final int extendEnd2 = 4;

    DeviceInterfaceModule cdim;

    DcMotor motorRight;
    DcMotor motorLeft;
    // SERVO
    Servo servoLeft;
    Servo servoRight;
    Servo servoBrake;

    // servo controller device
    ServoController servodevice;

    // TETRIX VALUES.
    final static double SERVORIGHT_MIN_RANGE = 0;
    final static double SERVORIGHT_MAX_RANGE = 1.0;
    final static double SERVOLEFT_MIN_RANGE = 0;
    final static double SERVOLEFT_MAX_RANGE = 1.0;

    double servoRightPosition;
    double servoLeftPosition;
    double servoBrakePosition;

    /**
     * Constructor
     */
    public zoeOpmodeDriveTest() {

    }

    public void initEndStops (){
        // The color sensor has an LED, set the digital channel to output mode.
        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");
        cdim.setDigitalChannelMode(raiseEnd1, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(raiseEnd2, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(extendEnd1, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(extendEnd2, DigitalChannelController.Mode.INPUT);
    }


    public void initServos() {

        servodevice = hardwareMap.servoController.get("servo");

        // Configuring the servos for control
        servoRight = hardwareMap.servo.get("servoright");
        servoLeft = hardwareMap.servo.get("servoleft");
        servoBrake = hardwareMap.servo.get("servofront");

        servoRight.setDirection(Servo.Direction.REVERSE);

        servoRightPosition = 0;
        servoLeftPosition = 0;
        servoBrakePosition = 0.1;  //brake on is 0.1, brake off is 0.27
        servoRight.setPosition(servoRightPosition);
        servoLeft.setPosition(servoLeftPosition);
        servoBrake.setPosition(servoBrakePosition);
    }

    @Override
    public void init() {
        initServos();
        initEndStops();
        motorRight = hardwareMap.dcMotor.get("motor1");
        motorLeft = hardwareMap.dcMotor.get("motor2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

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
        float right = -gamepad1.left_stick_y;
        float left = -gamepad1.right_stick_y;
        float brake = -gamepad2.left_stick_y;
//        float right = throttle - direction;
//        float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        brake = Range.clip(brake, 0, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        boolean stop1 = cdim.getDigitalChannelState(raiseEnd1);
        boolean stop2 = cdim.getDigitalChannelState(raiseEnd2);
        boolean stop3 = cdim.getDigitalChannelState(extendEnd1);
        boolean stop4 = cdim.getDigitalChannelState(extendEnd2);
        telemetry.addData("Stop1", "Stop" + stop1);
        telemetry.addData("Stop2", "Stop" + stop2);
        telemetry.addData("Stop3", "Stop" + stop3);
        telemetry.addData("Stop4", "Stop" + stop4);

        // write the values to the motors
        motorLeft.setPower(right);
        motorRight.setPower(left);

        servoBrake.setPosition(brake);
        telemetry.addData("Brake", "Brake" + brake);


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

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("C1 RIGHT X", "Stick1 X: " + gamepad1.left_stick_x);
        telemetry.addData("C1 RIGHT Y", "Stick1 Y: " + gamepad1.left_stick_y);
        telemetry.addData("C1 LEFT X", "Stick2 X: " + gamepad1.right_stick_x);
        telemetry.addData("C1 LEFT Y", "Stick2 Y: " + gamepad1.right_stick_y);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

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
