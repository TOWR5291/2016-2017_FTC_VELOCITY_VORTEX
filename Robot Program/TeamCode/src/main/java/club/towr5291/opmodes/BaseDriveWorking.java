package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import club.towr5291.robotconfig.HardwareDriveMotors;

/**
 * Created by kids on 11/4/2016 at 7:54 PM.
 */
@TeleOp()
public class BaseDriveWorking extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotor intake;
    DcMotor shooter;

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
        leftFront = hardwareMap.dcMotor.get("leftMotor1");
        leftBack = hardwareMap.dcMotor.get("leftMotor2");
        rightFront = hardwareMap.dcMotor.get("rightMotor1");
        rightBack = hardwareMap.dcMotor.get("rightMotor2");

        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        intake.setPower(0);
        shooter.setPower(0);

    }

    @Override
    public void loop() {

        if(gamepad1.left_bumper) {
            leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

            rightPow =  gamepad1.left_stick_y;
            leftPow =  gamepad1.right_stick_y;

            reverse = true;
        } else {
            leftPow =  gamepad1.left_stick_y;
            rightPow =  gamepad1.right_stick_y;

            leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            leftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            rightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            reverse = false;
        }

        if(gamepad1.right_bumper) {
            max = 0.5;
            slowdown = true;
        } else {
            max = 1;
            slowdown = false;
        }

        if (leftPow < 0) {
            leftNegative = true;
        } else {
            leftNegative = false;
        }

        if (rightPow < 0) {
            rightNegative = true;
        } else {
            rightNegative = false;
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

        leftFront.setPower(-leftPow);
        leftBack.setPower(-leftPow);
        rightFront.setPower(-rightPow);
        rightBack.setPower(-rightPow);

        telemetry.addData("Left Speed Raw", -gamepad1.left_stick_y);
        telemetry.addData("Right Speed Raw", -gamepad1.right_stick_y);
        telemetry.addData("Left Speed", -leftPow);
        telemetry.addData("Right Speed", -rightPow);
        telemetry.addData("Reverse", reverse);
        telemetry.addData("Slowdown", slowdown);

        if(gamepad2.left_trigger != 0) {
            intakeFlip = true;

            intake.setDirection(DcMotor.Direction.REVERSE);
        } else {
            intakeFlip = false;

            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        if(gamepad2.left_bumper) {
            intakeOn = true;

            intake.setPower(1);
        } else {
            intakeOn = false;

            intake.setPower(0);
        }

        if(gamepad2.right_bumper) {
            launch = true;

            shooter.setPower(1);
        } else {
            launch = false;

            shooter.setPower(0);
        }

        telemetry.addData("Intake", intakeOn);
        telemetry.addData("Intake Flip", intakeFlip);
        telemetry.addData("Shooter", launch);
    }
}
