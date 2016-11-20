package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import club.towr5291.robotconfig.HardwareArmMotors;
import club.towr5291.robotconfig.HardwareDriveMotors;

/**
 * Created by kids on 11/4/2016 at 7:54 PM.
 */
@TeleOp(name = "Base Drive", group = "5291")
public class BaseDriveWorking extends OpMode {

    // Declare OpMode members.
    private HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareArmMotors armDrive   = new HardwareArmMotors();   // Use a Pushbot's hardware

    double leftPow;
    double rightPow;
    boolean leftNegative;
    boolean rightNegative;
    double max;
    boolean reverse = false;
    boolean slowdown = false;
    boolean intakeOn = true;
    boolean intakeFlip = false;
    boolean launch = false;

    @Override
    public void init() {
        /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robotDrive.init(hardwareMap);
        armDrive.init(hardwareMap);

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotDrive.leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        robotDrive.leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        robotDrive.rightMotor1.setDirection(DcMotor.Direction.FORWARD);
        robotDrive.rightMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setDriveMotorPower(0);

        armDrive.sweeper.setPower(0);
        armDrive.flicker.setPower(0);

    }

    @Override
    public void loop() {

        if(gamepad1.left_bumper) {
            robotDrive.leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            robotDrive.leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            robotDrive.rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            robotDrive.rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            rightPow =  gamepad1.left_stick_y;
            leftPow =  gamepad1.right_stick_y;
            reverse = true;
        } else
        {
            leftPow =  gamepad1.left_stick_y;
            rightPow =  gamepad1.right_stick_y;
            robotDrive.leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            robotDrive.leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            robotDrive.rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            robotDrive.rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            reverse = false;
        }


        if(gamepad1.right_trigger > 0) {
            max = 0.5;
            slowdown = true;
        } else {
            max = 1;
            slowdown = false;
        }

        if (leftPow >= max && leftNegative == false) {
            leftPow = max;
        } else if (leftPow <= -max && leftNegative == true) {
            leftPow = -max;
        }

        if (rightPow >= max && rightNegative == false) {
            rightPow = max;
        } else if (rightPow <= -max && rightNegative == true) {
            rightPow = -max;
        }

        setDriveRightMotorPower(leftPow);
        setDriveLeftMotorPower(rightPow);

        telemetry.addData("Left Speed Raw", -gamepad1.left_stick_y);
        telemetry.addData("Right Speed Raw", -gamepad1.right_stick_y);
        telemetry.addData("Left Speed", -leftPow);
        telemetry.addData("Right Speed", -rightPow);
        telemetry.addData("Reverse", reverse);
        telemetry.addData("Slowdown", slowdown);

        if(gamepad2.right_trigger > 0) {
            intakeFlip = true;
            armDrive.sweeper.setDirection(DcMotor.Direction.FORWARD);
        } else {
            intakeFlip = false;
            armDrive.sweeper.setDirection(DcMotor.Direction.REVERSE);
        }

        if(gamepad2.right_bumper) {
            intakeOn = true;
            armDrive.sweeper.setPower(1);
        } else {
            intakeOn = false;
            armDrive.sweeper.setPower(0);
        }

        if(gamepad2.left_trigger > 0) {
            launch = true;
            armDrive.flicker.setPower(1);
        } else {
            launch = false;
            armDrive.flicker.setPower(0);
        }

        telemetry.addData("Sweeper", intakeOn);
        telemetry.addData("Sweeper Flip", intakeFlip);
        telemetry.addData("Flicker", launch);
    }

    //set the drive motors power, both left and right
    private void setDriveMotorPower (double power) {
        setDriveRightMotorPower(power);
        setDriveLeftMotorPower(power);
    }

    //set the right drive motors power
    private void setDriveRightMotorPower (double power) {
        robotDrive.rightMotor1.setPower(power);
        robotDrive.rightMotor2.setPower(power);
    }

    //set the left motors drive power
    private void setDriveLeftMotorPower (double power) {
        robotDrive.leftMotor1.setPower(power);
        robotDrive.leftMotor2.setPower(power);
    }

}
