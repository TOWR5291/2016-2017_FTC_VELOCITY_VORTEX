/* 2015/2016 Oxford Middle SchoolFirst Tech Challenge
Mrs. Watson Faculty Adviser

Change Log:


Variable Convention:



*/
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import java.lang.Math;
import java.util.concurrent.locks.Lock;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;


public class Team5291DriverStation_Rev2_5 extends OpMode {

    //List all of our variables
    //DEBUG
    boolean debugLevel0 = true;
    boolean debugLevel1 = true;
    boolean debugLevel99 = true;
    //GAMEPAD
    //Gamepad A = gamepad 1serv
    boolean Gamepad_A_Button_A;             //
    boolean Gamepad_A_Button_B;             //
    boolean Gamepad_A_Button_X;             //
    boolean Gamepad_A_Button_Y;             //
    float Gamepad_A_LeftStick_X;          //
    float Gamepad_A_LeftStick_Y;          //
    float Gamepad_A_RightStick_X;          //
    float Gamepad_A_RightStick_Y;          //
    //Gamepad B = gamepad 2
    boolean Gamepad_B_Dpad_down;            //
    boolean Gamepad_B_Dpad_up;              //
    boolean Gamepad_B_Dpad_left;            //
    boolean Gamepad_B_Dpad_right;           //
    //boolean Gamepad_B_Trigger_right;        //
    //boolean Gamepad_B_Trigger_left;         //
    boolean Gamepad_B_Bumper_left;          //
    boolean Gamepad_B_Bumper_right;         //
    boolean Gamepad_B_Button_A;             //
    boolean Gamepad_B_Button_B;             //
    boolean Gamepad_B_Button_X;             //
    boolean Gamepad_B_Button_Y;             //
    float Gamepad_B_LeftStick_X;          //
    float Gamepad_B_LeftStick_Y;          //
    float Gamepad_B_RightStick_X;          //
    float Gamepad_B_RightStick_Y;
    float Gamepad_B_Trigger_right;
    float Gamepad_B_Trigger_left;

    //MOTOR
    DcMotor motorRight;                     //Drive Motor Right
    DcMotor motorLeft;                      //Drive Motor Left
    DcMotor armraise;                       //Arm mover
    DcMotor armextend;
    int motordeviceModeRead = 1;
    int motordeviceModeWrite = 2;
    // motor controller device
    DcMotorController motordevice1;
    DcMotorController motordevice2;

    double brakeOn = 0.27;
    double brakeOff = 0.4;
    //brake on is 0.1, brake off is 0.32

    int ArmExtendPosition;
    int ArmRaisePosition;
    int maxArmExtension = 5000;
    int maxArmHeight = 5000;
    boolean stateOK;

    static final int raiseEnd1 = 7;
    static final int raiseEnd2 = 6;
    static final int extendEnd1 = 5;
    static final int extendEnd2 = 4;

    DeviceInterfaceModule cdim;

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

    public void initMotors() {
        //motor config
        motordevice1 = hardwareMap.dcMotorController.get("motorcon1");
        motordevice2 = hardwareMap.dcMotorController.get("motorcon2");

        motorLeft = hardwareMap.dcMotor.get("motor1");
        motorRight = hardwareMap.dcMotor.get("motor2");
        armextend = hardwareMap.dcMotor.get("motor3");
        armraise = hardwareMap.dcMotor.get("motor4");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        armraise.setDirection(DcMotor.Direction.REVERSE);
        armextend.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void initServos() {

        if (debugLevel0)
            DbgLog.msg("initServos :- Start ");
        servodevice = hardwareMap.servoController.get("servo");

        // Configuring the servos for control
        servoRight = hardwareMap.servo.get("servoright");
        servoLeft = hardwareMap.servo.get("servoleft");
        servoBrake = hardwareMap.servo.get("servofront");

        servoRight.setDirection(Servo.Direction.REVERSE);

        servoRightPosition = 0;
        servoLeftPosition = 0;
        servoBrakePosition = brakeOn;
        servoRight.setPosition(servoRightPosition);
        servoLeft.setPosition(servoLeftPosition);
        servoBrake.setPosition(servoBrakePosition);

        if (debugLevel0)
            DbgLog.msg("initServos :- End ");
    }

    public boolean moveServo1(double Position) {
        boolean rightOKToMove = true;

        if (debugLevel0)
            telemetry.addData("moveServo1", ":- Start");
        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < SERVORIGHT_MIN_RANGE) || (Range.scale(Position, 0, 180, 0, 1) > SERVORIGHT_MAX_RANGE)) {
            if (debugLevel1)
                telemetry.addData("moveServo1", "moveServosright :- Right Out Of Range, no move ");
            rightOKToMove = false;
        }


        if (rightOKToMove) {
            servoRight.setPosition(Range.scale(Position, 0, 180, 0, 1));
        }

        if (debugLevel0)
            telemetry.addData("moveServo1", ":- End");
        return true;
    }

    public boolean moveServo2(double Position) {
        boolean leftOKToMove = true;

        if (debugLevel0)
            telemetry.addData("moveServo2", ":- Start");
        //set right position
        if ((Range.scale(Position, 0, 180, 0, 1) < SERVOLEFT_MIN_RANGE) || (Range.scale(Position, 0, 180, 0, 1) > SERVOLEFT_MAX_RANGE)) {
            if (debugLevel1)
                telemetry.addData("moveServo2", ":- Left Out Of Range, no move");
            leftOKToMove = false;
        }

        if (leftOKToMove) {
            servoLeft.setPosition(Range.scale(Position, 0, 180, 0, 1));
        }

        if (debugLevel0)
            telemetry.addData("moveServo2", ":- End");
        return true;
    }

    public Team5291DriverStation_Rev2_5() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        initServos();
        initMotors();
        initMotorEncoders();  //encoders are too problematic, don't use, just make sure they are in write mode
        initEndStops();
    }

    @Override
    public void loop() {

        boolean stop1 = cdim.getDigitalChannelState(raiseEnd1);
        boolean stop2 = cdim.getDigitalChannelState(raiseEnd2);
        boolean stop3 = cdim.getDigitalChannelState(extendEnd1);
        boolean stop4 = cdim.getDigitalChannelState(extendEnd2);

        Gamepad_A_RightStick_Y = gamepad1.right_stick_y;
        telemetry.addData("Right stick Y", "Right stick Y: PUSHED, movement in progress..." + String.format("%.2f", Gamepad_A_RightStick_Y));
        Gamepad_A_LeftStick_Y = gamepad1.left_stick_y;
        telemetry.addData("Left stick Y", "Left stick Y: PUSHED, movement in progress..." + String.format("%.2f", Gamepad_A_LeftStick_Y));
        //Gamepad_A_Bumper_left = gamepad1.left_bumper;
        Gamepad_B_Trigger_left = gamepad2.left_trigger;
        telemetry.addData("servo", "trigger: PUSHED, movement in progress..." + String.format("%.2f", gamepad2.left_trigger));
        Gamepad_B_Trigger_right = gamepad2.right_trigger;
        telemetry.addData("servo", "trigger: PUSHED, movement in progress..." + String.format("%.2f", gamepad2.right_trigger));
        Gamepad_B_Bumper_left = gamepad2.left_bumper;
        Gamepad_B_Bumper_right = gamepad2.right_bumper;
        Gamepad_A_Button_X = gamepad1.x;
        Gamepad_A_Button_Y = gamepad1.y;
        Gamepad_A_Button_A = gamepad1.a;
        Gamepad_B_Button_X = gamepad2.x;
        Gamepad_B_Button_A = gamepad2.a;
        Gamepad_B_RightStick_Y = gamepad2.right_stick_y;
        Gamepad_B_LeftStick_Y = gamepad2.left_stick_y;
        //telemetry.addData("Left bumper", "Left bumper: PUSHED, servo movement in progress..." + String.format("%.2f", Gamepad_A_Bumper_left));
        //Gamepad_A_Bumper_right = gamepad1.right_bumper;
        //telemetry.addData("Right bumper Y", "Right bumper: PUSHED, servo movement in progress..." + String.format("%.2f", Gamepad_A_Bumper_right));
        //Gamepad_A_LeftStick_Y = -gamepad1.left_stick_y;
        //Gamepad_A_LeftStick_Y = gamepad1.left_stick_Y;
        //float DriveRight = Gamepad_A_LeftStick_Y - Gamepad_A_LeftStick_X;
        //float DriveLeft = Gamepad_A_LeftStick_Y + Gamepad_A_LeftStick_X;

        Gamepad_A_RightStick_Y = Range.clip(Gamepad_A_RightStick_Y, -1, 1);
        Gamepad_A_LeftStick_Y = Range.clip(Gamepad_A_LeftStick_Y, -1, 1);
        //DriveRight = (float)scaleInput(DriveRight);
        //DriveLeft =  (float)scaleInput(DriveLeft);
        // update the position of the arm.


        // clip the right/left values so that the values never exceed +/- 1
        //Gamepad_A_RightStick_Y = Range.clip(Gamepad_A_RightStick_Y, -1, 1);
        // Gamepad_A_LeftStick_Y = Range.clip(Gamepad_A_LeftStick_Y, -1, 1);

        // scale the joy stick value to make it easier to control
        // the robot more precisely at slower speeds.

        // telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        // telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        //telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

        if (Gamepad_B_Bumper_left) {
            moveServo1(90);
            telemetry.addData("servo:left", "left bumper: PUSHED, movement in progress...");
        }
        if (!Gamepad_B_Bumper_left) {
            moveServo1(0);
            telemetry.addData("servo:left", "left bumper: NOT PUSHED, returning to position...");
        }
        if (Gamepad_B_Bumper_right) {
            moveServo2(90);
            telemetry.addData("servo:right", "right bumper: PUSHED, movement in progress...");
        }
        if (!Gamepad_B_Bumper_right) {
            moveServo2(0);
            telemetry.addData("servo:left", "left bumper: NOT PUSHED, returning to position...");
        }
        if (Gamepad_B_Button_Y) {
            telemetry.addData("Button Y", "Button Y: PUSHED, holding motor...");
        }
        if (!Gamepad_B_Button_Y) {
            telemetry.addData("Button Y", "Button Y: NOT PUSHED, releasing motor...");
        }

        if (Gamepad_A_Button_X) {
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            motorRight.setDirection(DcMotor.Direction.FORWARD);
        }
        if (Gamepad_A_Button_A) {
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
            motorRight.setDirection(DcMotor.Direction.REVERSE);
        }

        // write the values to the motors
        DbgLog.msg("MAIN LOOP : Setting Main Drive power");
        motorRight.setPower(Gamepad_A_RightStick_Y);
        telemetry.addData("Left Stick", "Left Stick: " + String.format("%.2f", Gamepad_A_LeftStick_Y));
        motorLeft.setPower(Gamepad_A_LeftStick_Y);
        telemetry.addData("Right Stick", "Right Stick: " + String.format("%.2f", Gamepad_A_RightStick_Y));
        DbgLog.msg("MAIN LOOP : Finished Setting Main ");

        //this is a really stupid controller having to switch back and forth like this, cause lockups....  Need a better solution
//            stateOK = switchArmControllerMode(motordeviceModeRead);
//            DbgLog.msg("MAIN LOOP : Arm should be in READ");
//            ArmRaisePosition = armraise.getCurrentPosition();
//            ArmExtendPosition = armextend.getCurrentPosition();
//            stateOK = switchArmControllerMode(motordeviceModeWrite);
//            DbgLog.msg("MAIN LOOP : Arm should be in WRITE");
//
//            if (ArmRaisePosition <= maxArmHeight) {
//                DbgLog.msg("MAIN LOOP : Setting Power extend");
//                armraise.setPower(0.5*Gamepad_B_RightStick_Y);
//            } else {
//                DbgLog.msg("MAIN LOOP : Setting Power - retract");
//                armraise.setPower(-.01); //back off the max extension a little
//            }
//
//            if (ArmExtendPosition <= maxArmExtension) {
//                DbgLog.msg("MAIN LOOP : Setting Power extend");
//                armextend.setPower(0.5*Gamepad_B_LeftStick_Y);
//            } else {
//                DbgLog.msg("MAIN LOOP : Setting Power - retract");
//                armextend.setPower(-0.01);
//            }
        Gamepad_B_RightStick_Y = Range.clip(Gamepad_B_RightStick_Y, -1, 1);
        Gamepad_B_LeftStick_Y = Range.clip(Gamepad_B_LeftStick_Y, -1, 1);

        if (Gamepad_B_LeftStick_Y == 0)  {
            servoBrake.setPosition(brakeOn);
            armraise.setPower(0);
        } else {
            if (Gamepad_B_LeftStick_Y > 0) {
                if ((cdim.getDigitalChannelState(raiseEnd1))) {
                    servoBrake.setPosition(brakeOff);
                    armraise.setPower(Gamepad_B_LeftStick_Y);
                }
            }
            if (Gamepad_B_LeftStick_Y < 0) {
                if ((cdim.getDigitalChannelState(raiseEnd2))) {
                    servoBrake.setPosition(brakeOff);
                    armraise.setPower(Gamepad_B_LeftStick_Y);
                }
            }
        }

        if (Gamepad_B_RightStick_Y == 0)  {
            armextend.setPower(0);
        } else {
            if (Gamepad_B_RightStick_Y > 0) {
                if ((cdim.getDigitalChannelState(extendEnd1))) {
                    armextend.setPower(Gamepad_B_RightStick_Y);
                } else {
                    armextend.setPower(0);
                }
            }
            if (Gamepad_B_RightStick_Y < 0) {
                if ((cdim.getDigitalChannelState(extendEnd2))) {
                    armextend.setPower(Gamepad_B_RightStick_Y);
                } else {
                    armextend.setPower(0);
                }
            }
        }


        telemetry.addData("Right StickB", "Right StickB: " + String.format("%.2f", Gamepad_B_RightStick_Y));
        telemetry.addData("Left StickB", "Left StickB: " + String.format("%.2f", Gamepad_B_LeftStick_Y));
    }


    @Override
    public void stop () {

    }

    public void initMotorEncoders () {

        boolean stateOK = false;

        if (debugLevel0)
            DbgLog.msg("initMotorEncoders :- Start ");

        // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
        switchMotorControllerEncoderMode(2, 4);  //want the arm motors to run with encoders
        switchMotorControllerEncoderMode(1, 4);  //want drive motors to run normally

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
//        stateOK = switchMotorControllerMode(motordeviceModeRead);
//        stateOK = switchArmControllerMode(motordeviceModeRead);
//
//        // wait until motor encoders are reset
//        while ((armextend.getCurrentPosition() !=0) || (armraise.getCurrentPosition() !=0)) {
//            try {
//                Thread.sleep(100);
//                DbgLog.msg("initMotorEncoders - Waiting for encoder reset - Left " + motorLeft.getCurrentPosition() + " Right " + motorRight.getCurrentPosition());
//                if (debugLevel1) {
//                    //telemetry.addData("Waiting", "for encoder reset");
//                }
//            } catch (Exception ex) {
//                if (debugLevel1)
//                    DbgLog.msg("initMotorEncoders - Error Waiting for encoder reset ");
//                telemetry.addData("Error", " Encoders not reset");
//            }
//        }

//        if (debugLevel1)
//            DbgLog.msg("initMotorEncoders - Encoders Reset, Setting Mode to RUN TO POSITION ");
//        // Valid states, 1 = RESET_ENCODERS, 2 = RUN_TO_POSITION, 3 = RUN_USING_ENCODERS, 4 = RUN_WITHOUT_ENCODERS
//        switchMotorControllerEncoderMode(2, 3);

        // Switch the motor controller to read mode
        //don't call this function too quickly, the controller crashes when changing modes too often
        stateOK = switchArmControllerMode(motordeviceModeWrite);
        stateOK = switchMotorControllerMode(motordeviceModeWrite);  //motor controller should be in write mode to be able to set power

        if (debugLevel1)
            DbgLog.msg("initMotorEncoders :- Encoders Reset, Complete RUN TO POSITION ");
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
                    motordevice1.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice1.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE) {
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
                    motordevice1.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    // vaid states "READ_ONLY", "WRITE_ONLY", "SWITCHING_TO_READ_MODE", or "SWITCHING_TO_WRITE_MODE"
                    while (motordevice1.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE) {
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
    public void initEndStops (){
        // The color sensor has an LED, set the digital channel to output mode.
        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");
        cdim.setDigitalChannelMode(raiseEnd1, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(raiseEnd2, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(extendEnd1, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(extendEnd2, DigitalChannelController.Mode.INPUT);
    }



}
