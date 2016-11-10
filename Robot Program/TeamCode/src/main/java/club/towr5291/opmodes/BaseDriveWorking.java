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

    double leftPow;
    double rightPow;
    boolean leftNegative;
    boolean rightNegative;
    double max;
    boolean reverse = false;
    boolean slowdown = false;

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

        leftFront.setPower(-gamepad1.left_stick_y);
        leftBack.setPower(-gamepad1.left_stick_y);
        rightFront.setPower(-gamepad1.right_stick_y);
        rightBack.setPower(-gamepad1.right_stick_y);
    }
}
