package org.kettering.ftc.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class HaloDrive extends OpMode {

    private int counter = 0;
    private MyDCMotor leftDriveMotor = new MyDCMotor();
    private MyDCMotor rightDriveMotor = new MyDCMotor();
    private MyGyroSensor gyro = new MyGyroSensor();

    public HaloDrive() {
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

        float power = -gamepad1.left_stick_y;
        float direction = gamepad1.right_stick_x;

        // clip the right/left values so that the values never exceed +/- 1

        float left = -power + direction;
        float right = power + direction;

        left = Range.clip(left, -1.0F, 1.0F);
        right = Range.clip(right, -1.0F, 1.0F);

        leftDriveMotor.setPower(left);
        rightDriveMotor.setPower(right);

        rightDriveMotor.update();
        leftDriveMotor.update();
        gyro.update(right-(-left));

        if (counter % 10 == 0) {
            telemetry.addData("1","Right Motor Pos: "+rightDriveMotor.getCurrentPosition());
            telemetry.addData("2","Left Motor Pos: "+leftDriveMotor.getCurrentPosition());
            telemetry.addData("3","Heading: "+gyro.getHeading());
        }
    }

    @Override
    public void stop() {
    }


    public class MyDCMotor {

        private double power = 0.0;
        private int currentPosition = 0;

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

    public class MyGyroSensor {
        private double heading = 0.0;

        public void update(double powerDifference) {
            heading += 1.2 * powerDifference;
            if (heading > 360.0)
                heading -= 360.0;
            if (heading < 0)
                heading += 360.0;
        }

        public int getHeading() {
            return (int)heading;
        }
    }
}
