package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Local Shark on 9/24/2016.
 */
@TeleOp(name="Fun", group="Michael")
@Disabled
public class MichaelFun extends OpMode {

    DcMotor leftDriveFront;
    DcMotor rightDriveFront;
    DcMotor rightDriveBack;
    DcMotor leftDriveBack;



    double leftDriveFrontPower;
    double rightDriveFrontPower;
    double leftDriveBackPower;
    double rightDriveBackPower;

    @Override
    public void init() {
        leftDriveFront = hardwareMap.dcMotor.get("leftDriveFront");
        rightDriveFront = hardwareMap.dcMotor.get("rightDriveFront");
        leftDriveBack = hardwareMap.dcMotor.get("leftDriveBack");
        rightDriveBack = hardwareMap.dcMotor.get("rightDriveBack");

    }

    @Override
    public void loop() {

        rightDriveFrontPower = -gamepad1.left_stick_y;
        rightDriveBackPower = -gamepad1.left_stick_y;
        leftDriveFrontPower = -gamepad1.left_stick_y;
        leftDriveFrontPower = -gamepad1.left_stick_y;

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        leftDriveFront.setPower(leftDriveFrontPower);
        rightDriveFront.setPower(rightDriveFrontPower);
        leftDriveBack.setPower(leftDriveBackPower);
        rightDriveFront.setPower(rightDriveBackPower);

        telemetry.addData("Left Joystick", "%.2f", leftDriveFrontPower);
        telemetry.addData("Right Joystick", "%.2f", rightDriveFrontPower);
        telemetry.addData("Left Joystick", "%.2f", leftDriveBackPower);
        telemetry.addData("Left Joystick", "%.2f", rightDriveBackPower);
        updateTelemetry(telemetry);
    }

    @Override
    public void stop(){
    }
}
