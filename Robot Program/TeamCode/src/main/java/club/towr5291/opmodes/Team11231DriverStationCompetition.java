package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Team11231DriverStationCompetition", group="Driver_Station")
//@Disabled
public class Team11231DriverStationCompetition  extends LinearOpMode   {

    /*--------------------------------------------------------------------------------------------*/
    /* Declare OpMode members. */
    /* Public OpMode members. */
    public DcMotor  leftMotor1   = null;
    public DcMotor  leftMotor2   = null;
    public DcMotor  rightMotor1  = null;
    public DcMotor  rightMotor2  = null;
    public DcMotor  sweeper  = null;
    public DcMotor flicker = null;
    public Servo myServo = null;

    //private float sweepertarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        /*----------------------------------------------------------------------------------------*/
        //set hardware map
        //
        //
        /*----------------------------------------------------------------------------------------*/
        leftMotor1 = hardwareMap.dcMotor.get("lm1");
        leftMotor2 = hardwareMap.dcMotor.get("lm2");
        rightMotor1 = hardwareMap.dcMotor.get("rm2");
        rightMotor2 = hardwareMap.dcMotor.get("rm1");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        flicker = hardwareMap.dcMotor.get("flicker");
        myServo = hardwareMap.servo.get("servo1");

        /*----------------------------------------------------------------------------------------*/
        //set motor direction
        //
        //
        /*----------------------------------------------------------------------------------------*/
        leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        flicker.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.REVERSE);

        /*----------------------------------------------------------------------------------------*/
        //set run without encoders
        //
        //
        /*----------------------------------------------------------------------------------------*/
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*-----------------------------------------------------------------------------------------*/
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver -- Win!!!");
        telemetry.update();

        double armPosition = 0;
        double armDelta = .02;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            //Halo Drive System
            float forwardSpeed = -gamepad1.right_stick_y;
            //telemetry.addData("left", "%.2f", leftWheelPower);
            //telemetry.update();
            float turnRate = gamepad1.left_stick_x;
            //telemetry.addData("left", "%.2f", leftWheelPower);
            //telemetry.update();

        /*----------------------------------------------------------------------------------------*/
            //
            //
            //
        /*----------------------------------------------------------------------------------------*/
            leftMotor1.setPower(forwardSpeed + turnRate);
            leftMotor2.setPower(forwardSpeed + turnRate);

            rightMotor1.setPower(forwardSpeed - turnRate);
            rightMotor2.setPower(forwardSpeed - turnRate);
        /*----------------------------------------------------------------------------------------*/
            //RUN the SWEEPER with setPower(1), GAME PAD LEFT BUMPER
            //
            //
        /*----------------------------------------------------------------------------------------*/
            if (gamepad2.left_bumper) {
                sweeper.setPower(1);
                armPosition += armDelta;
                myServo.setPosition(armPosition);
            }else{
                armPosition += armDelta;
                myServo.setPosition(armPosition);
            }


        /*----------------------------------------------------------------------------------------*/
            //RUN the flicker with setPower(1), GAME PAD A
            //
            //
        /*----------------------------------------------------------------------------------------*/
            if (gamepad2.a)
                flicker.setPower(.4);

        /*----------------------------------------------------------------------------------------*/
            //REVERSE the SWEEPER with setPower(-1), RIGHT BUMPER
            //STOP the motor before reversing or forward
            //
        /*----------------------------------------------------------------------------------------*/
            if (gamepad2.right_bumper)
                sweeper.setPower(-1);

        /*----------------------------------------------------------------------------------------*/
            //STOP the SWEEPER with setPower(0), GAME PAD X
            //
            //
        /*----------------------------------------------------------------------------------------*/
            if (gamepad2.x)
                sweeper.setPower(0);

        /*----------------------------------------------------------------------------------------*/
            //STOP the flicker with setPower(0), GAME PAD B
            //
            //
        /*----------------------------------------------------------------------------------------*/
            if (gamepad2.b)
                flicker.setPower(0);


            //sweeper.setPower(sweepertarget - sweeper.getPower() * .01);

            //sweeper.setPower(0);

        }//end while

        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        //end run op mode

    }
}//end class
