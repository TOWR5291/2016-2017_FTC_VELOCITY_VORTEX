package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import club.towr5291.functions.FileLogger;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensors;


/**
 * Created by ianhaden on 2/09/16.
 */

@Autonomous(name="Pushbot: Auto Drive AStar Path Finder", group="5291Test")
//@Disabled
public class AutoDriveAStarPathFinder extends OpMode {
    /* Declare OpMode members. */
    HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use base drive hardware configuration

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder  Neverrest 20=140, Neverrest 40=280, Neverrest 60=420
    static final double     DRIVE_GEAR_REDUCTION    = 1.333 ;   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_ACTUAL_FUDGE      = 1;        // Fine tuning amount
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
    static final double     ROBOT_TRACK             = 16.5;     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    static final double     COUNTS_PER_DEGREE       =  ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;

    HardwareSensors robotSensors   = new HardwareSensors();     // Use base drive hardware configuration

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    //define each state for the step.  Each step should go through some of the states below
    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED
    }

    private int mCurrentStep = 0;                               // Current State Machine State.
    private stepState  mCurrentStepState;                       // Current State Machine State.
    private stepState  mCurrentDriveState;                      // Current State Machine State.
    private stepState  mCurrentTurnState;                       // Current State Machine State.
    private LibraryStateSegAuto[] mStateSegAuto;
    double mStepTimeout;
    double mStepDistance;
    double mStepSpeed;
    String mRobotDirection;
    double mStepTurnL;
    double mStepTurnR;
    int mStepLeftTarget;
    int mStepRightTarget;
    boolean baseStepComplete = false;
    boolean armStepComplete = true;
    public boolean activated = false;

    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();           // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();         // Time into current state

    //this is the sequence the state machine will follow
    private LibraryStateSegAuto[] mRobotAutonomous = {
            //                        time, head, dist, powe
            //                        out   ing   ance  r
            //                         s    deg   inch   %
            new LibraryStateSegAuto ( 10,  "L90",  12,  0.5 ),
            new LibraryStateSegAuto ( 10,  "R90",  12,  0.5 )

    };

    //set up Vuforia
    public static final String TAG = "Vison OPMODE";
    OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia;                                   // {@link #vuforia} is the variable we will use to store our instance of the Vuforia localization engine.
    public List<VuforiaTrackable> allTrackables;

    public float mmPerInch        = 25.4f;
    public float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    public float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    /*
    * Code to run ONCE when the driver hits INIT
    */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        robotDrive.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.update();
        robotDrive.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

        robotDrive.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        fileLogger.writeEvent("init()","Init Complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        fileLogger.writeEvent("start()","START PRESSED: ");


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (activated == false)
        {
            //init the Veforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AVATY7T/////AAAAGQJxfNYzLUgGjSx0aOEU0Q0rpcfZO2h2sY1MhUZUr+Bu6RgoUMUP/nERGmD87ybv1/lM2LBFDxcBGRHkXvxtkHel4XEUCsNHFTGWYcVkMIZqctQsIrTe13MnUvSOfQj8ig7xw3iULcwDpY+xAftW61dKTJ0IAOCxx2F0QjJWqRJBxrEUR/DfQi4LyrgnciNMXCiZ8KFyBdC63XMYkQj2joTN579+2u5f8aSCe8jkAFnBLcB1slyaU9lhnlTEMcFjwrLBcWoYIFAZluvFT0LpqZRlS1/XYf45QBSJztFKHIsj1rbCgotAE36novnAQBs74ewnWsJifokJGOYWdFJveWzn3GE9OEH23Y5l7kFDu4wc";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            VuforiaTrackables velocityVortex = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
            VuforiaTrackable wheels = velocityVortex.get(0);
            VuforiaTrackable tools = velocityVortex.get(1);
            VuforiaTrackable legos = velocityVortex.get(2);
            VuforiaTrackable gears = velocityVortex.get(3);

            wheels.setName("wheels");  // wheels target
            tools.setName("tools");  // tools target
            legos.setName("legos");  // legos target
            gears.setName("gears");  // gears target

            /** For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(velocityVortex);

            // RED Targets
            // To Place GEARS Target
            // position is approximately - (-6feet, -1feet)

            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                    .translation(-mmFTCFieldWidth / 2, -1 * 12 * mmPerInch, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            gears.setLocation(gearsTargetLocationOnField);
            RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(gearsTargetLocationOnField));

            // To Place GEARS Target
            // position is approximately - (-6feet, 3feet)
            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the Blue Audience wall.
                    Our translation here is a positive translation in Y.*/
                    .translation(-mmFTCFieldWidth / 2, 3 * 12 * mmPerInch, 0)
                    //.translation(0, mmFTCFieldWidth/2, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            tools.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(toolsTargetLocationOnField));

            //Finsih RED Targets

            // BLUE Targets
            // To Place LEGOS Target
            // position is approximately - (-6feet, -1feet)

            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                    .translation(-3 * 12 * mmPerInch, mmFTCFieldWidth / 2, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            legos.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Gears Target=%s", format(legosTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(legosTargetLocationOnField));

            // To Place WHEELS Target
            // position is approximately - (-6feet, 3feet)
            OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the Blue Audience wall.
                    Our translation here is a positive translation in Y.*/
                    .translation(1 * 12 * mmPerInch, mmFTCFieldWidth / 2, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            wheels.setLocation(wheelsTargetLocationOnField);
            RobotLog.ii(TAG, "Tools Target=%s", format(wheelsTargetLocationOnField));
            fileLogger.writeEvent("init()", "Tools Target=%s" + format(wheelsTargetLocationOnField));

            //Finsih BLUE Targets

            //set up phone location
            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(mmBotWidth / 2, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZY,
                            AngleUnit.DEGREES, -90, 0, 0));
            RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
            fileLogger.writeEvent("init()", "Phone Location=%s" + format(phoneLocationOnRobot));
            //finish phone location

            //trackable listeners
            ((VuforiaTrackableDefaultListener) gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            /** Start tracking the data sets we care about. */
            velocityVortex.activate();
            activated = true;
        }


        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Pos", format(lastLocation));
        } else {
            telemetry.addData("Pos", "Unknown");
        }

        fileLogger.writeEvent("loop()", "STATE: " + String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString() + " Current Turn State:- " + mCurrentTurnState.toString() + " Current Drive State:- " + mCurrentDriveState.toString());
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("STATE", String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString());
        // Execute the current state.  Each STATE's case code does the following:

        switch (mCurrentStepState) {
            case STATE_INIT: {
                initStep(mRobotAutonomous);
            }
            break;
            case STATE_START: {

            }
            break;
            case STATE_RUNNING: {
                runningTurnStep();
                runningDriveStep();
                if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentTurnState == stepState.STATE_COMPLETE) && (armStepComplete)) {
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_COMPLETE;
                }
            }
            break;
            case STATE_PAUSE: {

            }
            break;
            case STATE_COMPLETE: {
                fileLogger.writeEvent("loop()", "Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                if ((mCurrentStep) < (mRobotAutonomous.length - 1)) {
                    fileLogger.writeEvent("loop()", "Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;

                } else {
                    fileLogger.writeEvent("loop()", "STATE_COMPLETE - Setting FINISHED ");
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_FINISHED;
                }
            }
            break;
            case STATE_TIMEOUT: {
                robotDrive.leftMotor.setPower(0);
                robotDrive.rightMotor.setPower(0);
                //  Transition to a new state.
                mCurrentStepState = stepState.STATE_FINISHED;

            }
            break;
            case STATE_ERROR: {
                telemetry.addData("STATE", "ERROR WAITING TO FINISH " + mCurrentStep);
            }
            break;
            case STATE_FINISHED: {
                telemetry.addData("STATE", "FINISHED " + mCurrentStep);
            }
            break;

        }

        //check timeout vale
        if ((mStateTime.seconds() > mStepTimeout) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED))) {
            //  Transition to a new state.
            mCurrentStepState = stepState.STATE_TIMEOUT;
        }

        telemetry.update();
    }


     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        telemetry.addData("FileLogger Op Stop: ", runtime.toString());
        if (fileLogger != null) {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }


    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    public void initStep (LibraryStateSegAuto[] step)
    {

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();
        mStateSegAuto = step;
        mStepTimeout = mStateSegAuto[mCurrentStep].mRobotTimeOut;
        mStepDistance =  mStateSegAuto[mCurrentStep].mRobotDistance;
        mStepSpeed = mStateSegAuto[mCurrentStep].mRobotSpeed;
        mRobotDirection = mStateSegAuto[mCurrentStep].mRobotDirection;

        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 0)    :- " + mRobotDirection.substring(0, 0)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 1)    :- " + mRobotDirection.substring(0, 1)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 2)    :- " + mRobotDirection.substring(0, 2)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(0, 3)    :- " + mRobotDirection.substring(0, 3)  );
//        fileLogger.writeEvent("initStep()","mRobotDirection.substring(1)       :- " + mRobotDirection.substring(1)  );

        if (mRobotDirection.substring(0, 1).equals("L")) {
            mStepTurnL = Double.parseDouble(mRobotDirection.substring(1));
            mStepTurnR = 0;
        } else if (mRobotDirection.substring(0, 1).equals("R" )) {
            mStepTurnL = 0;
            mStepTurnR = Double.parseDouble(mRobotDirection.substring(1));
        } else {
            mStepTurnL = 0;
            mStepTurnR = 0;
            mCurrentTurnState = stepState.STATE_COMPLETE;
        }

        mCurrentStepState = stepState.STATE_RUNNING;

//        fileLogger.writeEvent("initStep()","Current Step    :- " + mCurrentStep  );
//        fileLogger.writeEvent("initStep()","mStepTimeout    :- " + mStepTimeout  );
//        fileLogger.writeEvent("initStep()","mStepDistance   :- " + mStepDistance  );
//        fileLogger.writeEvent("initStep()","mStepSpeed      :- " + mStepSpeed  );
//        fileLogger.writeEvent("initStep()","mRobotDirection :- " + mRobotDirection  );
//        fileLogger.writeEvent("initStep()","mStepTurnL      :- " + mStepTurnL  );
//        fileLogger.writeEvent("initStep()","mStepTurnR      :- " + mStepTurnR  );

    }

    //--------------------------------------------------------------------------
    //  Execute the state.
    //--------------------------------------------------------------------------
    public void runningTurnStepPivot ()
    {

        switch (mCurrentTurnState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningTurnStep()","mStepTurnL      :- " + mStepTurnL  );
                fileLogger.writeEvent("runningTurnStep()","mStepTurnR      :- " + mStepTurnR  );

                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                    // pass target position to motor controller
                    robotDrive.leftMotor.setTargetPosition(mStepLeftTarget);
                    // set motor controller to mode
                    robotDrive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    robotDrive.leftMotor.setPower(Math.abs(.5));
                }
                else {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                    // pass target position to motor controller
                    robotDrive.rightMotor.setTargetPosition(mStepRightTarget);
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    robotDrive.rightMotor.setPower(Math.abs(.5));
                }

                fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );

                mCurrentTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                fileLogger.writeEvent("runningTurnStep()","Running         " );
                fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                if (mStepTurnR == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor.getCurrentPosition(), robotDrive.rightMotor.getCurrentPosition());
                    if (!robotDrive.leftMotor.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor.getCurrentPosition(), robotDrive.rightMotor.getCurrentPosition());
                    if (!robotDrive.rightMotor.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    robotDrive.leftMotor.setPower(0);
                    robotDrive.rightMotor.setPower(0);
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    mCurrentTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    public void runningTurnStep ()
    {
        switch (mCurrentTurnState) {
            case STATE_INIT: {
                fileLogger.writeEvent("runningTurnStep()","mStepTurnL      :- " + mStepTurnL  );
                fileLogger.writeEvent("runningTurnStep()","mStepTurnR      :- " + mStepTurnR  );

                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor.getCurrentPosition() + (int)(0.5 * mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor.getCurrentPosition() - (int)(0.5 * mStepTurnL * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget:-  " + mStepLeftTarget);
                }
                else {
                    // Determine new target position
                    fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor.getCurrentPosition());
                    mStepLeftTarget = robotDrive.leftMotor.getCurrentPosition() - (int)(0.5 * mStepTurnR * COUNTS_PER_DEGREE);
                    mStepRightTarget = robotDrive.rightMotor.getCurrentPosition() + (int)(0.5 * mStepTurnR * COUNTS_PER_DEGREE);
                    fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget);
                }

                // pass target position to motor controller
                robotDrive.leftMotor.setTargetPosition(mStepLeftTarget);
                robotDrive.rightMotor.setTargetPosition(mStepRightTarget);
                // set motor controller to mode
                robotDrive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set power on motor controller to start moving
                robotDrive.leftMotor.setPower(Math.abs(.5));
                robotDrive.rightMotor.setPower(Math.abs(.5));

                fileLogger.writeEvent("runningTurnStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                fileLogger.writeEvent("runningTurnStep()","mStepRightTarget:- " + mStepRightTarget  );

                mCurrentTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                fileLogger.writeEvent("runningTurnStep()","Running         " );
                fileLogger.writeEvent("runningTurnStep()","Current LPosition:-" + robotDrive.leftMotor.getCurrentPosition() + " LTarget:- " + mStepLeftTarget);
                fileLogger.writeEvent("runningTurnStep()","Current RPosition:-" + robotDrive.rightMotor.getCurrentPosition() + " RTarget:- " + mStepRightTarget);
                if (mStepTurnR == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor.getCurrentPosition(), robotDrive.rightMotor.getCurrentPosition());
                    if (!robotDrive.leftMotor.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    fileLogger.writeEvent("runningTurnStep()","Running         " );
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor.getCurrentPosition(), robotDrive.rightMotor.getCurrentPosition());
                    if (!robotDrive.rightMotor.isBusy()) {
                        fileLogger.writeEvent("runningTurnStep()","Complete         " );
                        mCurrentTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    robotDrive.leftMotor.setPower(0);
                    robotDrive.rightMotor.setPower(0);
                    fileLogger.writeEvent("runningTurnStep()","Complete         " );
                    mCurrentTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    public void runningDriveStep()
    {
        if (mCurrentTurnState == stepState.STATE_COMPLETE) {
            switch (mCurrentDriveState) {
                case STATE_INIT: {
                    fileLogger.writeEvent("runningDriveStep()","mStepDistance   :- " + mStepDistance  );
                    fileLogger.writeEvent("runningDriveStep()","mStepDistance   :- " + mStepDistance  );

                    // Determine new target position
                    mStepLeftTarget = robotDrive.leftMotor.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                    mStepRightTarget = robotDrive.rightMotor.getCurrentPosition() + (int) (mStepDistance * COUNTS_PER_INCH);
                    // pass target position to motor controller
                    robotDrive.leftMotor.setTargetPosition(mStepLeftTarget);
                    robotDrive.rightMotor.setTargetPosition(mStepRightTarget);

                    fileLogger.writeEvent("runningDriveStep()","mStepLeftTarget :- " + mStepLeftTarget  );
                    fileLogger.writeEvent("runningDriveStep()","mStepRightTarget:- " + mStepRightTarget  );

                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotDrive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // set power on motor controller to start moving
                    robotDrive.leftMotor.setPower(Math.abs(mStepSpeed));
                    robotDrive.rightMotor.setPower(Math.abs(mStepSpeed));

                    mCurrentDriveState = stepState.STATE_RUNNING;
                }
                break;
                case STATE_RUNNING: {
                    if (robotDrive.leftMotor.isBusy() && robotDrive.rightMotor.isBusy()) {
                        telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget, mStepRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor.getCurrentPosition(), robotDrive.rightMotor.getCurrentPosition());
                    } else {
                        // Stop all motion;
                        robotDrive.leftMotor.setPower(0);
                        robotDrive.rightMotor.setPower(0);
                        baseStepComplete = true;
                        fileLogger.writeEvent("runningDriveStep()","Complete         " );
                        mCurrentDriveState = stepState.STATE_COMPLETE;
                    }
                }
                break;
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
