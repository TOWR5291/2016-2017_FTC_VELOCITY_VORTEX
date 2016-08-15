package org.kettering.ftc.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MoveForward extends OpMode {

    private int counter = 0;
    private MyDCMotor leftDriveMotor = new MyDCMotor();
    private MyDCMotor rightDriveMotor = new MyDCMotor();

    public MoveForward() {
        super();
    }

    @Override
    public void init() {

       // DcMotor leftDriveMotor = hardwareMap.dcMotor.get("motor_1");
       // DcMotor rightDriveMotor = hardwareMap.dcMotor.get("motor_2");
    }

    @Override
    public void loop() {

        counter++;

        if (counter < 100) {
            leftDriveMotor.setPower(-0.5);
            rightDriveMotor.setPower(0.5);
        }
        else {
            leftDriveMotor.setPower(0.0);
            rightDriveMotor.setPower(0.0);
        }

        rightDriveMotor.update();

        if (counter % 10 == 0)
            telemetry.addData("1","Right Motor Pos: "+rightDriveMotor.getCurrentPosition());
    }

    @Override
    public void stop() {
    }


    public class MyDCMotor {

        private double power;
        private int currentPosition;

        public MyDCMotor() {

        }

        public void setPower(double p) {
            this.power = p;
        }

        public int getCurrentPosition() {
            return this.currentPosition;
        }

        public void update() {
            this.currentPosition += (int)(2.0 * this.power);
        }
    }
}
