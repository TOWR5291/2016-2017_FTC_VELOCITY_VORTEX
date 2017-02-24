package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Local Shark on 9/24/2016.
 */
@TeleOp(name="Intake", group="Katie")
@Disabled
public class IntakeProgramKatie extends OpMode {

    DcMotor intake1;
    DcMotor intake2;

    double intake1Power;
    double intake2Power;
    boolean intake;

    // Gamepad inputs


    @Override
    public void init() {
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        intake = false;

        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake1.setPower(intake1Power);
        intake2.setPower(intake2Power);
    }

    @Override
    public void loop() {



        if(gamepad1.left_trigger != 0) { intake = true;}
        if(gamepad1.right_trigger != 0) { intake = false;}

        if (intake) {

            intake2Power = -1;
            intake1Power = -1;




        } else {
            intake2Power = 0;
            intake1Power = 0;
        }

        intake1.setPower(intake1Power);
        intake2.setPower(intake2Power);

        telemetry.addData("", "%.2f", intake1Power);
        telemetry.addData("", "%.2f", intake2Power);
        updateTelemetry(telemetry);
    }

    @Override
    public void stop(){
    }
}
