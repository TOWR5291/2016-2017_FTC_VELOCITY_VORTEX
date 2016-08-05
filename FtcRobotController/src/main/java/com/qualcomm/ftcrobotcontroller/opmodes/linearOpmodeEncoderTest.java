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

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class linearOpmodeEncoderTest extends LinearOpMode {

    DeviceInterfaceModule cdim;

    // bEnabled and bDisabled represents the state of the output pins.
    boolean bEnabled = true;
    boolean bDisabled = false;

    // bPrevState and bCurrState represent the previous and current state of the button on the GamePad.
    boolean bPrevState = false;
    boolean bCurrState = false;

    float driveLeft;
    float driveRight;
    float throttle;
    float direction;
    int motorLeftPos;
    int motorRightPos;
    int motorRightTargetPos;
    int motorLeftTargetPos;
    int motorDeviceModeRead = 1;
    int motorDeviceModeWrite = 2;
    private static final double countsPerInch = 95;  //2280 measured for 2 feet
    private static final double distanceToMove = 24.0;  // inches

    // To go straight, Left Counts increases, Right Counts decreases
    // To go backward, Left Counts decreases, Right Counts increases
    private static final boolean leftReversed = false;
    private static final boolean rightReversed = true;


    // the motors are on the motor controller
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotorController motordevice;

    public void switchMotorControllerMode (int state) {
        int currentState = 0;
        // valid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
        // State 1 = READ_ONLY
        // State 2 = WRITE_ONLY
        // State 3 = SWITCHING_TO_READ_MODE
        // State 4 = SWITCHING_TO_WRITE_MODE

        // valid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
        if (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            currentState = 1;
            DbgLog.msg("switchMotorControllerMode - Currentstate 1 - device mode: " + motordevice.getMotorControllerDeviceMode());

        } else if (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY) {
            currentState = 2;
            DbgLog.msg("switchMotorControllerMode - Currentstate 2 - device mode: " + motordevice.getMotorControllerDeviceMode());
        }

        if (state != currentState) {

            switch (state) {

                case 1: {
                    DbgLog.msg("switchMotorControllerMode - Setting MotorController to Read ");
                    motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
                        try {
                            sleep(10);
                            DbgLog.msg("switchMotorControllerMode - Waiting for read mode ");
                            telemetry.addData("Waiting ", " Setting Read_Only");
                            motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        } catch (Exception ex) {
                            telemetry.addData("Error ", " Setting Read_Only");
                        }
                    }
                }
                break;
                case 2: {
                    DbgLog.msg("switchMotorControllerMode - Setting MotorController to Write ");
                    motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
                        try {
                            sleep(10);
                            DbgLog.msg("switchMotorControllerMode - Waiting for write mode ");
                            telemetry.addData("Waiting ", " Setting Write_Only");
                            motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        } catch (Exception ex) {
                            telemetry.addData("Error ", " Setting Write_Only");
                        }
                    }
                }
                break;
            }
        }
    }

        @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");
        motordevice = hardwareMap.dcMotorController.get("motor");

        // Configuring the motors for control
        motorRight = hardwareMap.dcMotor.get("motor1");
        motorLeft = hardwareMap.dcMotor.get("motor2");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        switchMotorControllerMode(motorDeviceModeRead);

        waitOneFullHardwareCycle();
        while ((motorLeft.getCurrentPosition() !=0) || (motorLeft.getCurrentPosition() !=0))
            waitOneFullHardwareCycle();

        switchMotorControllerMode(motorDeviceModeWrite);

//        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        switchMotorControllerMode(motorDeviceModeRead);
        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            telemetry.addData("Text", "***Data***");
            // check the status of the x button on either gamepad.


            bCurrState = gamepad1.x || gamepad2.x;

            // check for button state transitions.
            if (bCurrState == true && bCurrState != bPrevState)  {
                //Move the robot 2 feet forward
                switchMotorControllerMode(motorDeviceModeRead);
                /*
                motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                while (motordevice.getMotorControllerDeviceMode() ==  DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
                    waitOneFullHardwareCycle();
                }
                */

                DbgLog.msg("Linear Opmode While Loop 1 - About to read encoders ");
                // find out the encoder positions now
                motorRightPos = motorRight.getCurrentPosition();
                motorLeftPos = motorLeft.getCurrentPosition();

                //put controller into write mode
                switchMotorControllerMode(motorDeviceModeWrite);

                /*
                motordevice.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                while (motordevice.getMotorControllerDeviceMode() ==  DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
                    waitOneFullHardwareCycle();
                }
                */
                // 2 feet is 2280 encoder counts

                motorRightTargetPos = motorRightPos - 2280;
                motorLeftTargetPos = motorLeftPos - 2280;
                DbgLog.msg("motorRightTargetPos " + motorRightTargetPos);
                DbgLog.msg("motorLeftTargetPos " + motorLeftTargetPos);

                DbgLog.msg("Linear Opmode While Loop 2 - About to set encoders ");

                motorRight.setTargetPosition(motorRightTargetPos);
                motorLeft.setTargetPosition(motorLeftTargetPos);


                motorRight.setPower(0.2);
                motorLeft.setPower(0.2);

                // button is transitioning to a pressed state.

                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was pressed!");

                // update previous state variable.
                bPrevState = bCurrState;

//                throttle = (float) 0.2;  // % max speed
//                direction = (float) 0;  // straight ahead

                // write the values to the motors
//                motorRight.setPower(driveRight);
//                motorLeft.setPower(driveLeft);

            } else if (bCurrState == false && bCurrState != bPrevState)  {
                switchMotorControllerMode(motorDeviceModeRead);
                // button is transitioning to a released state.

                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was released!");

                // update previous state variable.
                bPrevState = bCurrState;
                // write the values to the motors
                //motorRight.setPower(0);
                //motorLeft.setPower(0);

                driveRight = throttle - direction;
                driveLeft = throttle + direction;
            }
            // get the current position of the robot
            //DbgLog.msg("Linear Opmode While Loop 3 - Setting read encoders ");
            switchMotorControllerMode(motorDeviceModeRead);
            //DbgLog.msg("Linear Opmode While Loop 4 - About to read encoders ");
            motorRightPos = motorRight.getCurrentPosition();
            motorLeftPos = motorLeft.getCurrentPosition();
            telemetry.addData("Right Motor:- ",  motorRightPos);
            telemetry.addData("Left Motor :- ",  motorLeftPos);
            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();
        }

    }

}



