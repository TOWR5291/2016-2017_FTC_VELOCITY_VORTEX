package org.kettering.ftc.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HelloDriver extends OpMode {

    private int counter = 0;

    public HelloDriver() {
        super();

        boolean b = false;


        DcMotor leftMotor = hardwareMap.dcMotor.get("motor_1");

        leftMotor.setPower(-1.0);

    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        counter++;

        if (counter % 10 == 0)
            telemetry.addData("1","Hello Driver... Counter: "+counter);
    }

    @Override
    public void stop() {
    }

}
