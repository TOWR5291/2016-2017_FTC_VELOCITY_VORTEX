package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Local Shark on 9/24/2016.
 */
@TeleOp(name="Launch", group="Michael")
@Disabled
public class LaunchingProgramMichaelTeleop extends OpMode {

    DcMotor launch1;
    DcMotor launch2;

    double launch1Power;
    double launch2Power;
    boolean launch;

    // Gamepad inputs


    @Override
    public void init() {
        launch1 = hardwareMap.dcMotor.get("launch1");
        launch2 = hardwareMap.dcMotor.get("launch2");
        launch = false;

        launch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launch1.setDirection(DcMotorSimple.Direction.FORWARD);
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);

        launch1.setPower(launch1Power);
        launch2.setPower(launch2Power);
    }

    @Override
    public void loop() {



        if(gamepad1.left_bumper) { launch = true;}
        if(gamepad1.right_bumper) { launch = false;}


        if (launch) {

            launch2Power = 1;
            launch1Power = 1;




        } else {
            launch2Power = 0;
            launch1Power = 0;
        }

        launch1.setPower(launch1Power);
        launch2.setPower(launch2Power);

        telemetry.addData("", "%.2f", launch1Power);
        telemetry.addData("", "%.2f", launch2Power);
        updateTelemetry(telemetry);
    }

    @Override
    public void stop(){
    }
}
