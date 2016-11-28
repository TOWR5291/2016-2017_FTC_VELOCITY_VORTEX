package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryStateSegAuto;
import club.towr5291.robotconfig.HardwareDriveMotors;


@Autonomous(name="11231 Autonomous Comp", group="5291Test")
public class Autonomous11231ByIan extends LinearOpMode
{
    private static final String TAG = "Autonomous11231ByIan";

    private SharedPreferences sharedPreferences;

    /* Declare OpMode members. */
    private HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //set up the variables for the file logger
    private FileLogger fileLogger;
    //private int debug = 0;
    //private int debug = 1;
    //private int debug = 2;
    private int debug = 3;
    //private int debug = 4;
    //private int debug = 5;
    //private int debug = 6;
    //----------------------------------------------------------------------------------------------
    //set up colour sensor variables
    ColorSensor colorSensor;    // Hardware Device Object
    //----------------------------------------------------------------------------------------------
    private boolean colorError = false;
    /*------------------------------------------------------------------------------------------------*/
    //RANGE SENSOR
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    /*--------------------------------------------------------------------------------------------*/
    //GYRO
    private ModernRoboticsI2cGyro gyro;
    private double gyrozAccumulated = 0;
    private int heading = 0;
    private int gyroXVal,gyroYVal,gyroZVal;
    int mytarget;
    private boolean gyroError = false;
    private boolean gyroLastResetState = false;
    private boolean gyroCurResetState  = false;
    final double GYRO_CORRECTION_MULTIPLIER = .9833;
    /*--------------------------------------------------------------------------------------------*/

    private double     COUNTS_PER_MOTOR_REV    = 1120 ;      // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    private double     DRIVE_GEAR_REDUCTION    = 0.656666666666666 ;     // This is < 1.0 if geared UP, Tilerunner is geared up
    private double     WHEEL_DIAMETER_INCHES   = 4.0 ;      // For figuring circumference
    private double     WHEEL_ACTUAL_FUDGE      = 1;         // Fine tuning amount
    private double     COUNTS_PER_INCH;
    private double     ROBOT_TRACK;                         //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    private double     COUNTS_PER_DEGREE;
    private boolean     BeaconAllianceColorBlue;
    private boolean     BeaconAllianceColorRed;
    private boolean     isBeaconColorBlue;
    private boolean     isBeaconColorRed;
    private double     mRobotTurnAngle;

    //vuforia localisation variables
    OpenGLMatrix lastLocation = null;
    private double localisedRobotX;
    private double localisedRobotY;
    private double localisedRobotBearing;
    private boolean localiseRobotPos;

    private String allianceColor;
    private String alliancePosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;


    //define each state for the step.  Each step should go through some of the states below
    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED,
        STATE_ASTAR_PRE_INIT,
        STATE_ASTAR_INIT,
        STATE_ASTAR_RUNNING,
        STATE_ASTAR_ERROR,
        STATE_ASTAR_COMPLETE
    }

    // set up the variables for the state engine
    private int mCurrentStep = 1;                               // Current Step in State Machine.
    private stepState mCurrentStepState;                        // Current State Machine State.
    private stepState mCurrentDriveState;                       // Current State of Drive.
    private stepState mCurrentTankTurnState;                    // Current State of Tank Turn.
    private stepState mCurrentPivotTurnState;                   // Current State of Pivot Turn.
    private stepState mCurrentDelayState;                       // Current State of Delay (robot doing nothing)
    private stepState mShootParticleState;                       //added 11/23/16Current State of Shoot Particle
    private stepState mBeaconAllianceColorState;
    private stepState mTurnRobotState;
    private int mStartPositionLeft1;
    private int mStartPositionLeft2;
    private int mStartPositionRight1;
    private int mStartPositionRight2;
    private double mRobotParm1;
    private double mRobotParm2;
    private double mRobotParm3;
    private double mRobotParm4;
    private double mRobotParm5;
    private double mRobotParm6;
    private double mStepTimeout;
    private double mStepSpeed;
    private String mRobotCommand;
    private boolean mRobotStepComplete;
    private double mStepTurnL;
    private  double mStepTurnR;
    private double mStepDistance;
    private double mStepDistanceTempSP;
    private int mStepLeftTarget1;
    private int mStepLeftTarget2;
    private int mStepRightTarget1;
    private int mStepRightTarget2;
    private int mStepDelay;
    private boolean baseStepComplete = false;
    boolean armStepComplete = true;
    private double turnAbsolute;
    private double gyroTurnSpeed;
    private float ultaDistance;
    private float OpticalDist;
    private float ultaDistanceTarget;


    private ElapsedTime mStateTime = new ElapsedTime();                                 // Time into current state

    private HashMap<String,LibraryStateSegAuto> autonomousSteps = new HashMap<String,LibraryStateSegAuto>();
    private HashMap<String,String> powerTable = new HashMap<String,String>();

    private int loadStep = 1;

    /*private void loadPowerTableTileRunner ()
    {
        powerTable.put(String.valueOf(0.5), ".1");
        powerTable.put(String.valueOf(1), ".2");
        powerTable.put(String.valueOf(2), ".3");
        powerTable.put(String.valueOf(4), ".4");
        powerTable.put(String.valueOf(6), ".5");
        powerTable.put(String.valueOf(8), ".6");
        powerTable.put(String.valueOf(10), ".7");
        powerTable.put(String.valueOf(12), ".8");
    }

    private void loadPowerTableTankTread ()
    {
        powerTable.put(String.valueOf(0.5), ".3");
        powerTable.put(String.valueOf(1), ".3");
        powerTable.put(String.valueOf(2), ".4");
        powerTable.put(String.valueOf(4), ".5");
        powerTable.put(String.valueOf(6), ".6");
        powerTable.put(String.valueOf(8), "1");
        powerTable.put(String.valueOf(10), "1");
        powerTable.put(String.valueOf(12), "1");
    }*/

    private void loadStaticSteps ()
    {
        //Use Static Steps not used you need to be on alliance color.

        // Valid Commands Angle
        // RT = Right Turn Angle
        // LT = Left Turn Angle
        // LP = Left Pivot Angle
        // RP = Left Pivot Angle
        // LR = Left turn with radius
        // RR = Right turn with radius
        // FW = Drive Forward Distance
        // RV = Drive Backward Distance
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %
        //loadSteps(10, "FW25", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "FW48", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RP90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LP90", 0,    0,    0,    0,    0,    0,  0.5);
    }

    private void loadStaticStepsRedLeft ()
    {

        // Valid Commands Angle
        // RT = Right Turn Angle
        // LT = Left Turn Angle
        // LP = Left Pivot Angle
        // RP = Left Pivot Angle
        // LR = Left turn with radius
        // RR = Right turn with radius
        // FW = Drive Forward Distance
        // RV = Drive Backward Distance
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %
        //loadSteps(10, "FW30", 0,    0,    0,    0,    0,    0,    .1);
        //loadSteps(10, "RT90", 0,    0,    0,    0,    0,    0,    .1);
        //loadSteps(10, "FW30", 0,    0,    0,    0,    0,    0,    .1);
        //(10, "RT90", 0,    0,    0,    0,    0,    0,    .1);
        //loadSteps(10, "FW30", 0,    0,    0,    0,    0,    0,    .1);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LT90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RP90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LP90", 0,    0,    0,    0,    0,    0,  0.5);
        loadSteps(10, "DL0", 0,    0,    0,    0,    0,    0,  0);
    }

    private void loadStaticStepsRedRight ()
    {

        // Valid Commands Angle
        // RT = Right Turn Angle
        // LT = Left Turn Angle
        // LP = Left Pivot Angle
        // RP = Left Pivot Angle
        // LR = Left turn with radius
        // RR = Right turn with radius
        // FW = Drive Forward Distance
        // RV = Drive Backward Distance
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %
        //loadSteps(10, "FW37", 0,    0,    0,    0,    0,    0,    .5);
        //loadSteps(10, "LT90", 0,    0,    0,    0,    0,    0,    .7);
        //loadSteps(10, "FW60", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LT90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RP90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LP90", 0,    0,    0,    0,    0,    0,  0.5);
        loadSteps(10, "DL0", 0,    0,    0,    0,    0,    0,  0);
    }

    private void loadStaticStepsBlueLeft ()
    {
        // Valid Commands Angle
        // RT = Right Turn Angle
        // LT = Left Turn Angle
        // LP = Left Pivot Angle
        // RP = Right Pivot Angle
        // LR = Left turn with radius
        // RR = Right turn with radius
        // FW = Drive Forward Distance
        // RV = Drive Backward Distance
        // GT = Guro Turn at Set Degrees
        // BC = Push beacon color based on distance, 1 = BLUE Alliance and 2 = RED Alliance
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %

        loadSteps(10, "FW10", 0,    0,    0,    0,    0,    0,    0.3);     //
        loadSteps(10, "GT45", 0,    0,    0,    0,    0,    0,    0);     //
        loadSteps(10, "FW10", 0,    0,    0,    0,    0,    0,    0);     //
        loadSteps(5, "SP", 4,    0,    0,    0,    0,    0,  0);
        // loadSteps(10, "FW6", 0,    0,    0,    0,    0,    0,    0.3);     //
        // loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,  0.3);       //
        // loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,  0.3);       //
        //loadSteps(10, "RT40", 0,    0,    0,    0,    0,    0,  0.3);       //
        //loadSteps(10, "RT12.5",    0,  0,  0,  0,  0,  0,  0.2);       //
        // loadSteps(10, "FW21.5",    0,  0,  0,  0,  0,  0,  0.2);      //
        // loadSteps(10, "LT45", 0,    0,    0,    0,    0,    0,  0.3);       //
        //loadSteps(5, "SP", 4,    0,    0,    0,    0,    0,  0);
        //loadSteps(5, "BC2", 1,    0,    0,    0,    0,    0,    0.2);     //
        //loadSteps(10, "RT45", 0,    0,    0,    0,    0,    0,    0.3);     //
        //loadSteps(10, "FW14", 0,    0,    0,    0,    0,    0,    0.3);     //
        //loadSteps(5, "SP", 3,    0,    0,    0,    0,    0,  0);            //
        //loadSteps(10, "LP90", 0,    0,    0,    0,    0,    0,  0.5);     //
        loadSteps(2, "DL0", 0,    0,    0,    0,    0,    0,  0);           //
    }

    private void loadStaticStepsBlueRight ()
    {
        // Valid Commands Angle
        // RT = Right Turn Angle
        // LT = Left Turn Angle
        // LP = Left Pivot Angle
        // RP = Left Pivot Angle
        // LR = Left turn with radius
        // RR = Right turn with radius
        // FW = Drive Forward Distance
        // RV = Drive Backward Distance
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %
        //loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "FW48", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RT90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LT90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RP90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LP90", 0,    0,    0,    0,    0,    0,  0.5);
        loadSteps(10, "DL0", 0,    0,    0,    0,    0,    0,  0);
    }

    private void loadSteps(int timeOut, String command, int parm1, int parm2, int parm3, int parm4, int parm5, int parm6, double power)
    {
        autonomousSteps.put(String.valueOf(loadStep), new LibraryStateSegAuto(loadStep, timeOut, command, parm1, parm2, parm3, parm4, parm5, parm6, power, false));
        loadStep++;
    }

    private void insertSteps(int timeOut, String command, int parm1, int parm2, int parm3, int parm4, int parm5, int parm6, double power, int insertlocation)
    {
        Log.d("insertSteps", "insert location " + insertlocation + " timout " + timeOut + " command " + command + " parm1 " + parm1 + " parm2 " + parm2 + " parm3 " + parm3 + " parm4 " + parm4 + " parm5 " + parm5 + " parm6 " + parm6 + " power " + power);
        HashMap<String,LibraryStateSegAuto> autonomousStepsTemp = new HashMap<String,LibraryStateSegAuto>();
        LibraryStateSegAuto processingStepsTemp;

        //move all the steps from current step to a temp location
        for (int loop = insertlocation; loop < loadStep; loop++)
        {
            processingStepsTemp = autonomousSteps.get(String.valueOf(loop));
            Log.d("insertSteps", "Reading all the next steps " + loop + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand() );
            autonomousStepsTemp.put(String.valueOf(loop), autonomousSteps.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "All steps loaded to a temp hasmap");

        //insert the step we want
        autonomousSteps.put(String.valueOf(insertlocation), new LibraryStateSegAuto (loadStep, timeOut, command, parm1, parm2, parm3, parm4, parm5, parm6, power, false));
        Log.d("insertSteps", "Inserted New step");

        //move all the other steps back into the sequence
        for (int loop = insertlocation; loop < loadStep; loop++)
        {
            processingStepsTemp = autonomousStepsTemp.get(String.valueOf(loop));
            Log.d("insertSteps", "adding these steps back steps " + (loop + 1) + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand() );
            autonomousSteps.put(String.valueOf(loop + 1), autonomousStepsTemp.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "Re added all the previous steps");

        //increment the step counter as we instered a new one
        loadStep++;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*----------------------------------------------------------------------------------------*/
        //GYRO HARDWARE
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro1");
        //CALIBRATE THR GYRO
        gyro.calibrate();//Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID
        sleep(1000);
        while(gyro.isCalibrating()){
        }
        /*----------------------------------------------------------------------------------------*/
        //RANGE HARDWARE
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        /*----------------------------------------------------------------------------------------*/
        //load all the vuforia stuff
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "Aai2GEX/////AAAAGaIIK9GK/E5ZsiRZ/jrJzdg7wYZCIFQ7uzKqQrMx/0Hh212zumzIy4raGwDY6Mf6jABMShH2etZC/BcjIowIHeAG5ShG5lvZIZEplTO+1zK1nFSiGFTPV59iGVqH8KjLbQdgUbsCBqp4f3tI8BWYqAS27wYIPfTK697SuxdQnpEZAOhHpgz+S2VoShgGr+EElzYMBFEaj6kdA/Lq5OwQp31JPet7NWYph6nN+TNHJAxnQBkthYmQg687WlRZhYrvNJepnoEwsDO3NSyeGlFquwuQwgdoGjzq2qn527I9tvM/XVZt7KR1KyWCn3PIS/LFvADSuyoQ2lsiOFtM9C+KCuNWiqQmj7dPPlpvVeUycoDH";
        //parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);                                          //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1);                                                           //tells VuforiaLocalizer to only store one frame at a time
        //ConceptVuforiaGrabImage vuforia = new ConceptVuforiaGrabImage(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables velocityVortex = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels = velocityVortex.get(0);
        wheels.setName("wheels");  // wheels target

        VuforiaTrackable tools  = velocityVortex.get(1);
        tools.setName("tools");  // tools target

        VuforiaTrackable legos = velocityVortex.get(2);
        legos.setName("legos");  // legos target

        VuforiaTrackable gears  = velocityVortex.get(3);
        gears.setName("gears");  // gears target

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(velocityVortex);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Wheels Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         *
         */

        // RED Targets
        // To Place GEARS Target
        // position is approximately - (-6feet, -1feet)

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -1 * 12 * mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        // To Place GEARS Target
        // position is approximately - (-6feet, 3feet)
        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFTCFieldWidth/2, 3 * 12 * mmPerInch, 0)
                //.translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        //Finsih RED Targets

        // BLUE Targets
        // To Place LEGOS Target
        // position is approximately - (-3feet, 6feet)

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-3 * 12 * mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(legosTargetLocationOnField));

        // To Place WHEELS Target
        // position is approximately - (1feet, 6feet)
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(1 * 12 * mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(wheelsTargetLocationOnField));

        //Finsih BLUE Targets

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation((mmBotWidth/2), 50,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Blue");
        alliancePosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "Team11231Autonomous");
        LibraryStateSegAuto processingSteps = new LibraryStateSegAuto(0,0,"",0,0,0,0,0,0,0,false);

        if (debug >= 1)
        {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
            runtime.reset();
            telemetry.addData("FileLogger: ", runtime.toString());
            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        }

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Alliance Colour   " +  allianceColor);
            fileLogger.writeEvent(TAG, "Alliance Position " +  alliancePosition);
            fileLogger.writeEvent(TAG, "Alliance Delay    " +  delay);
            fileLogger.writeEvent(TAG, "Alliance Beacons  " +  numBeacons);
            fileLogger.writeEvent(TAG, "Robot Config      " +  robotConfig);
        }

        //to add more config options edit strings.xml and AutonomousConfiguration.java
        switch (robotConfig) {
            case "Team11231Autonomous": //Team 11231 Autonomous
                COUNTS_PER_MOTOR_REV    = 1120;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION    = .667;                                                   // (.665) UP INCREASES THE DISTANCE This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES   = 4.0 ;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE      = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535)) * WHEEL_ACTUAL_FUDGE ;
                ROBOT_TRACK             = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE       = ((2 * 3.1415926535 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                //loadPowerTableTileRunner();                                                         //load the power table
                break;

            default:  //default for competition TileRunner-2x20
                COUNTS_PER_MOTOR_REV    = 1120;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION    = 0.667;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES   = 4.0 ;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE      = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535)) * WHEEL_ACTUAL_FUDGE ;
                ROBOT_TRACK             = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE       = ((2 * 3.1415926535 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                //loadPowerTableTileRunner();                                                         //load the power table
                break;
        }

        //show options on the driver station phone
        telemetry.addData("Alliance   ",  allianceColor);
        telemetry.addData("Start Pos  ",  alliancePosition);
        telemetry.addData("Start Del  ",  delay);
        telemetry.addData("# Beacons  ",  numBeacons);
        telemetry.addData("Robot      ",  robotConfig);

        //need to load initial step of a delay based on user input
        loadSteps(delay + 2, "DL" + (delay*1000), 0,    0,    0,    0,    0,    0,  0);

        //load the sequence based on alliance colour
        switch (allianceColor) {
            case "Red":
                switch (alliancePosition) {
                    case "Left":
                        loadStaticStepsRedLeft();                                                               //load all the steps into the hashmaps
                        break;
                    case "Right":
                        loadStaticStepsRedRight();                                                               //load all the steps into the hashmaps
                        break;
                }
                break;
            case "Blue":
                switch (alliancePosition) {
                    case "Left":
                        loadStaticStepsBlueLeft();                                                               //load all the steps into the hashmaps
                        break;
                    case "Right":
                        loadStaticStepsBlueRight();                                                               //load all the steps into the hashmaps
                        break;
                }
                break;
            case "Test":
                loadStaticSteps();                                                                  //load all the steps into the hashmaps
                break;
        }

        //don't crash the program if the GRYO is faulty, just bypass it
        //try {
        // get a reference to a Modern Robotics GyroSensor object.
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        // calibrate the gyro, this takes a few seconds
        //gyro.calibrate();
        //} catch (Exception e) {
        //if (debug >= 1)
        //{
        //fileLogger.writeEvent(TAG, "Gyro Error " +  e.getMessage());
        //}
        //gyroError = true;
        //}

        /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robotDrive.init(hardwareMap);

        // get a reference to our ColorSensor object.
        try {
            colorSensor = hardwareMap.colorSensor.get("sensorcolor");
        } catch (Exception e) {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "color Error " +  e.getMessage());
            }
            colorError = true;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotDrive.rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTankTurnState = stepState.STATE_COMPLETE;
        mCurrentDriveState = stepState.STATE_COMPLETE;
        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
        mCurrentDelayState = stepState.STATE_COMPLETE;
        mShootParticleState = stepState.STATE_COMPLETE;
        mBeaconAllianceColorState = stepState.STATE_COMPLETE;

        if (!gyroError) {
            while (!isStopRequested() && gyro.isCalibrating()) {
                sleep(50);
                idle();
            }
        }

        if (debug >= 1)
        {
            fileLogger.writeEvent(TAG, "Init Complete");
        }

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //activate vuforia
        velocityVortex.activate();

        //the main loop.  this is where the action happens
        while (opModeIsActive())
        {
            //use vuforia to get locations informatio
            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                VectorF trans = lastLocation.getTranslation();
                Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                // Robot position is defined by the standard Matrix translation (x and y)
                localisedRobotX = trans.get(0);
                localisedRobotY = trans.get(1);

                // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                localisedRobotBearing = rot.thirdAngle;
                if (localisedRobotBearing < 0)
                {
                    localisedRobotBearing = 360 + localisedRobotBearing;
                }

                telemetry.addData("Pos X ", localisedRobotX);
                telemetry.addData("Pos Y ", localisedRobotY);
                telemetry.addData("Bear  ", localisedRobotBearing);
                telemetry.addData("Pos   ", format(lastLocation));
                localiseRobotPos = true;
            } else {
                localiseRobotPos = false;
                telemetry.addData("Pos   ", "Unknown");
            }

            if (debug >= 3)
            {
                fileLogger.writeEvent(TAG, "mCurrentStepState:- " + mCurrentStepState + " mCurrentStepState " + mCurrentStepState);
            }
            switch (mCurrentStepState)
            {
                case STATE_INIT:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "mCurrentStepState:- " + mCurrentStepState + " mCurrentStepState " + mCurrentStepState);
                        fileLogger.writeEvent(TAG, "About to check if step exists " + mCurrentStep);
                    }
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousSteps.containsKey(String.valueOf(mCurrentStep)))
                    {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Step Exists TRUE " + mCurrentStep + " about to get the values from the step");
                        }
                        processingSteps = autonomousSteps.get(String.valueOf(mCurrentStep));
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Got the values for step " + mCurrentStep + " about to decode");
                        }

                        //decode the step from hashmap
                        initStep(processingSteps);
                    }
                    else  //if no steps left in hashmap then complete
                    {
                        mCurrentStepState = stepState.STATE_FINISHED;
                    }
                }
                break;
                case STATE_START:
                {

                }
                break;
                case STATE_RUNNING:
                {
                    DelayStep();
                    TankTurnStep();
                    PivotTurnStep();
                    DriveStep();
                    setParticleShooter();
                    setBeaconAllianceColor();
                    setTurnRobot();
                    if ((mCurrentDelayState == stepState.STATE_COMPLETE) &&
                            (mCurrentDriveState == stepState.STATE_COMPLETE) &&
                            (mCurrentPivotTurnState == stepState.STATE_COMPLETE) &&
                            (mShootParticleState == stepState.STATE_COMPLETE) &&
                            (mBeaconAllianceColorState == stepState.STATE_COMPLETE) &&
                            (mTurnRobotState == stepState.STATE_COMPLETE) &&
                            (mCurrentTankTurnState == stepState.STATE_COMPLETE))
                    {
                        mCurrentStepState = stepState.STATE_COMPLETE;
                    }
                }
                break;
                case STATE_PAUSE:
                {

                }
                break;
                case STATE_COMPLETE:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "Step Complete - Current Step:- " + mCurrentStep);
                    }

                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;
                }
                break;
                case STATE_TIMEOUT:
                {
                    setDriveMotorPower(0);
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_FINISHED;
                }
                break;
                case STATE_ERROR:
                {
                    telemetry.addData("STATE", "ERROR WAITING TO FINISH " + mCurrentStep);
                }
                break;
                case STATE_FINISHED:
                {
                    setDriveMotorPower(0);

                    //deactivate vuforia
                    velocityVortex.deactivate();

                    telemetry.addData("STATE", "FINISHED " + mCurrentStep);
                    if (debug >= 1)
                    {
                        if (fileLogger != null)
                        {
                            fileLogger.writeEvent(TAG, "Step FINISHED - FINISHED");
                            fileLogger.writeEvent(TAG, "Stopped");
                            fileLogger.close();
                            fileLogger = null;
                        }
                    }
                }
                break;
            }

            //check timeout vale
            if ((mStateTime.seconds() > mStepTimeout  ) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED)))
            {
                //  Transition to a new state.
                mCurrentStepState = stepState.STATE_TIMEOUT;
            }
            telemetry.update();
        }

        //opmode not active anymore

    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Initialise the state.
    //--------------------------------------------------------------------------
    private void initStep (LibraryStateSegAuto mStateSegAuto) {

        mStepDistance = 0;

        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Starting to Decode Step ");
        }

        // Reset the state time, and then change to next state.
        baseStepComplete = false;
        mStateTime.reset();

        mStepTimeout = mStateSegAuto.getmRobotTimeOut();
        mStepSpeed = mStateSegAuto.getmRobotSpeed();
        mRobotCommand = mStateSegAuto.getmRobotCommand();
        mRobotParm1 = mStateSegAuto.getmRobotParm1();
        mRobotParm2 = mStateSegAuto.getmRobotParm2();
        mRobotParm3 = mStateSegAuto.getmRobotParm3();
        mRobotParm4 = mStateSegAuto.getmRobotParm4();
        mRobotParm5 = mStateSegAuto.getmRobotParm5();
        mRobotParm6 = mStateSegAuto.getmRobotParm6();
        mRobotStepComplete = mStateSegAuto.getmRobotStepComplete();

        if (debug >= 3) {
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 2)    :- " + mRobotCommand.substring(0, 2));
        }

        mCurrentStepState = stepState.STATE_RUNNING;

        switch (mRobotCommand.substring(0, 2))
        {
            case "DL":
                mCurrentDelayState = stepState.STATE_INIT;
                break;
            case "LT":
                mCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "RT":
                mCurrentTankTurnState = stepState.STATE_INIT;
                break;
            case "LP":
                mCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "RP":
                mCurrentPivotTurnState = stepState.STATE_INIT;
                break;
            case "FW":  // Drive forward a distance in inches and power setting
                mCurrentDriveState = stepState.STATE_INIT;
                break;
            case "SP": //code added 11/23/16 to add a shoot particle state
                mShootParticleState = stepState.STATE_INIT;
                break;
            case "RV": //code added 11/25/16 to be able to reverse the robot
                mCurrentDriveState = stepState.STATE_INIT;
                break;
            case "BC": //code added 11/27/16 to be able to detect if beacon is alliance color and push button
                mBeaconAllianceColorState =  stepState.STATE_INIT;
                break;
            case "GT": //code added 11/27/16 to be able to make a robot turn based on commanded angle
                mTurnRobotState =  stepState.STATE_INIT;
            case "FN":  //  Run a special Function with Parms
                break;
        }

        if (debug >= 2) {
            fileLogger.writeEvent("initStep()", "Current Step        :- " + mCurrentStep);
            fileLogger.writeEvent("initStep()", "mStepTimeout        :- " + mStepTimeout);
            fileLogger.writeEvent("initStep()", "mStepSpeed          :- " + mStepSpeed);
            fileLogger.writeEvent("initStep()", "mRobotCommand       :- " + mRobotCommand);
            fileLogger.writeEvent("initStep()", "mRobotParm1         :- " + mRobotParm1);
            fileLogger.writeEvent("initStep()", "mRobotParm2         :- " + mRobotParm2);
            fileLogger.writeEvent("initStep()", "mRobotParm3         :- " + mRobotParm3);
            fileLogger.writeEvent("initStep()", "mRobotParm4         :- " + mRobotParm4);
            fileLogger.writeEvent("initStep()", "mRobotParm5         :- " + mRobotParm5);
            fileLogger.writeEvent("initStep()", "mRobotParm6         :- " + mRobotParm6);
            fileLogger.writeEvent("initStep()", "mStepDistance       :- " + mStepDistance);
            fileLogger.writeEvent("initStep()", "mStepTurnL          :- " + mStepTurnL);
            fileLogger.writeEvent("initStep()", "mStepTurnR          :- " + mStepTurnR);
            fileLogger.writeEvent("initStep()", "mRobotStepComplete  :- " + mRobotStepComplete);
        }
    }

    private void DriveStep()
    {
        double mStepSpeedTemp;
        double distanceToEndLeft1;
        double distanceToEndLeft2;
        double distanceToEndRight1;
        double distanceToEndRight2;
        double distanceToEnd;
        double distanceFromStartLeft1;
        double distanceFromStartLeft2;
        double distanceFromStartRight1;
        double distanceFromStartRight2;
        double distanceFromStart;

        switch (mCurrentDriveState)
        {
            case STATE_INIT:
            {
                mStepDistance = 0;

                switch (mRobotCommand.substring(0, 2)) {
                    case "FW":  // Drive forward a distance in inches and power setting
                        mStepDistance = Double.parseDouble(mRobotCommand.substring(2));
                        robotDrive.leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                        robotDrive.leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                        robotDrive.rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                        robotDrive.rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                        break;
                    //code added 11/25/16 to set a reverse distance
                    case "RV":  // Drive forward a distance in inches and power setting
                        mStepDistance = Double.parseDouble(mRobotCommand.substring(2));
                        robotDrive.leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                        robotDrive.leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                        robotDrive.rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
                        robotDrive.rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
                        break;
                }

                if (debug >= 2)
                {
                    fileLogger.writeEvent("runningDriveStep()", "mStepDistance   :- " + mStepDistance);
                    fileLogger.writeEvent("runningDriveStep()", "mStepDistance   :- " + mStepDistance);
                }
                // Determine new target position
                mStartPositionLeft1 = robotDrive.leftMotor1.getCurrentPosition();
                mStartPositionLeft2 = robotDrive.leftMotor2.getCurrentPosition();
                mStartPositionRight1 = robotDrive.rightMotor1.getCurrentPosition();
                mStartPositionRight2 = robotDrive.rightMotor2.getCurrentPosition();

                mStepLeftTarget1 = mStartPositionLeft1 + (int) (mStepDistance * COUNTS_PER_INCH);
                mStepLeftTarget2 = mStartPositionLeft2 + (int) (mStepDistance * COUNTS_PER_INCH);
                mStepRightTarget1 = mStartPositionRight1 + (int) (mStepDistance * COUNTS_PER_INCH);
                mStepRightTarget2 = mStartPositionRight2 + (int) (mStepDistance * COUNTS_PER_INCH);

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget1);
                robotDrive.leftMotor2.setTargetPosition(mStepLeftTarget2);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget1);
                robotDrive.rightMotor2.setTargetPosition(mStepRightTarget2);

                if (debug >= 2)
                {
                    fileLogger.writeEvent("runningDriveStep()", "mStepLeftTarget1 :- " + mStepLeftTarget1 +  " mStepLeftTarget2 :- " + mStepLeftTarget2);
                    fileLogger.writeEvent("runningDriveStep()", "mStepRightTarget1:- " + mStepRightTarget1 + " mStepRightTarget2:- " + mStepRightTarget2);
                }

                // set motor controller to mode, Turn On RUN_TO_POSITION
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mCurrentDriveState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING:
            {
                mStepSpeedTemp = mStepSpeed;

                // ramp up speed - need to write function to ramp up speed
                distanceFromStartLeft1 = Math.abs(mStartPositionLeft1 - robotDrive.leftMotor1.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceFromStartLeft2 = Math.abs(mStartPositionLeft2 - robotDrive.leftMotor2.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceFromStartRight1 = Math.abs(mStartPositionRight1 - robotDrive.rightMotor1.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceFromStartRight2 = Math.abs(mStartPositionRight2 - robotDrive.rightMotor2.getCurrentPosition()) / COUNTS_PER_INCH;

                //if moving ramp up
                distanceFromStart = (distanceFromStartLeft1 + distanceFromStartRight1 + distanceFromStartLeft2 + distanceFromStartRight2) / 4;

                //determine how close to target we are
                distanceToEndLeft1 = (mStepLeftTarget1 - robotDrive.leftMotor1.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceToEndLeft2 = (mStepLeftTarget2 - robotDrive.leftMotor2.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceToEndRight1 = (mStepRightTarget1 - robotDrive.rightMotor1.getCurrentPosition()) / COUNTS_PER_INCH;
                distanceToEndRight2 = (mStepRightTarget2 - robotDrive.rightMotor2.getCurrentPosition()) / COUNTS_PER_INCH;

                //if getting close ramp down speed
                distanceToEnd = (distanceToEndLeft1 + distanceToEndRight1 + distanceToEndLeft2 + distanceToEndRight2) / 4;


                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(mStepSpeedTemp));

                //if within error margin stop
                if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy())
                {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningDriveStep()", "distanceFromStart " + distanceFromStart + " distanceToEnd " + distanceToEnd + " Power Level " + mStepSpeedTemp + " Running to target  L1, L2, R1, R2  " + mStepLeftTarget1 + ", " + mStepLeftTarget2 + ", " + mStepRightTarget1 + mStepLeftTarget2 + ", " + " Running at position " + robotDrive.leftMotor1.getCurrentPosition() + " " + robotDrive.rightMotor1.getCurrentPosition());
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget1, mStepRightTarget1);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    telemetry.addData("Path3", "Running at %7d :%7d", robotDrive.leftMotor2.getCurrentPosition(), robotDrive.rightMotor2.getCurrentPosition());
                }
                else
                {
                    // Stop all motion;
                    setDriveMotorPower(0);
                    baseStepComplete = true;
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent("runningDriveStep()", "Complete         ");
                    }
                    mCurrentDriveState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    //--------------------------------------------------------------------------
    //  Execute the state.
    //--------------------------------------------------------------------------

    private void PivotTurnStep ()  //should be same as radius turn with radius of 1/2 robot width, so this function can be deleted once radius turn is completed
    {
        switch (mCurrentPivotTurnState) {
            case STATE_INIT: {
                mStepTurnL = 0;
                mStepTurnR = 0;

                switch (mRobotCommand.substring(0, 2)) {
                    case "LP":
                        mStepTurnL = Double.parseDouble(mRobotCommand.substring(2));
                        mStepTurnR = 0;
                        break;
                    case "RP":
                        mStepTurnL = 0;
                        mStepTurnR = Double.parseDouble(mRobotCommand.substring(2));
                        break;
                }
                if (debug >= 2)
                {
                    fileLogger.writeEvent("PivotTurnStep()", "mStepTurnL      :- " + mStepTurnL);
                    fileLogger.writeEvent("PivotTurnStep()", "mStepTurnR      :- " + mStepTurnR);
                }
                // Turn On RUN_TO_POSITION
                if(mStepTurnR == 0) {
                    // Determine new target position
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent("PivotTurnStep()", "Current LPosition:-" + robotDrive.leftMotor1.getCurrentPosition());
                    }
                    mStepLeftTarget1 = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepLeftTarget2 = robotDrive.leftMotor2.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget1 = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    mStepRightTarget2 = robotDrive.rightMotor2.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent("PivotTurnStep()", "mStepLeftTarget1:-  " + mStepLeftTarget1 + " mStepLeftTarget2:-  " + mStepLeftTarget2);
                    }
                    // pass target position to motor controller
                    robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget1);
                    robotDrive.leftMotor2.setTargetPosition(mStepLeftTarget2);
                    // set motor controller to mode
                    robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveLeftMotorPower(Math.abs(.3));
                }
                else {
                    // Determine new target position
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent("PivotTurnStep()", "Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition());
                    }
                    mStepLeftTarget1 = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepLeftTarget2 = robotDrive.leftMotor2.getCurrentPosition() + (int) (mStepTurnL * COUNTS_PER_DEGREE);
                    mStepRightTarget1 = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    mStepRightTarget2 = robotDrive.rightMotor2.getCurrentPosition() + (int) (mStepTurnR * COUNTS_PER_DEGREE);
                    if (debug >= 2)
                    {
                        fileLogger.writeEvent("PivotTurnStep()", "mStepRightTarget1:- " + mStepRightTarget1 + " mStepRightTarget2:- " + mStepRightTarget2);
                    }
                    // pass target position to motor controller
                    robotDrive.rightMotor1.setTargetPosition(mStepRightTarget1);
                    robotDrive.rightMotor2.setTargetPosition(mStepRightTarget2);
                    // set motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    setDriveRightMotorPower(Math.abs(.3));
                }
                if (debug >= 3)
                {
                    fileLogger.writeEvent("PivotTurnStep()", "mStepLeftTarget :- " + mStepLeftTarget1 +  " mStepLeftTarget2 :- " + mStepLeftTarget2);
                    fileLogger.writeEvent("PivotTurnStep()", "mStepRightTarget:- " + mStepRightTarget1 + " mStepRightTarget2:- " + mStepRightTarget2);
                }
                mCurrentPivotTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {
                if (debug >= 3)
                {
                    fileLogger.writeEvent("PivotTurnStep()", "Running         ");
                    fileLogger.writeEvent("PivotTurnStep()", "Current LPosition1:-" + robotDrive.leftMotor1.getCurrentPosition() + " LTarget:- " + mStepLeftTarget1 + " LPosition2:-" + robotDrive.leftMotor2.getCurrentPosition() + " LTarget2:- " + mStepLeftTarget2);
                    fileLogger.writeEvent("PivotTurnStep()", "Current RPosition:-" + robotDrive.rightMotor1.getCurrentPosition() + " RTarget:- " + mStepRightTarget1 + " RPosition2:-" + robotDrive.rightMotor2.getCurrentPosition() + " RTarget2:- " + mStepRightTarget2);
                }
                if (mStepTurnR == 0) {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("PivotTurnStep()", "Running         ");
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget1, mStepRightTarget1);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    telemetry.addData("Path3", "Running at %7d :%7d", robotDrive.leftMotor2.getCurrentPosition(), robotDrive.rightMotor2.getCurrentPosition());
                    if (!robotDrive.leftMotor1.isBusy()) {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent("PivotTurnStep()","Complete         " );
                        }
                        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                    }
                } else if (mStepTurnL == 0) {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("PivotTurnStep()","Running         " );
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget1, mStepRightTarget1);
                    telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                    telemetry.addData("Path3", "Running at %7d :%7d", robotDrive.leftMotor2.getCurrentPosition(), robotDrive.rightMotor2.getCurrentPosition());
                    if (!robotDrive.rightMotor1.isBusy()) {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent("PivotTurnStep()","Complete         " );
                        }
                        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(0);
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent("PivotTurnStep()","Complete         " );
                    }
                    mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    private void TankTurnStep ()
    {
        switch (mCurrentTankTurnState) {
            case STATE_INIT: {

                mStepTurnL = 0;
                mStepTurnR = 0;

                switch (mRobotCommand.substring(0, 2)) {
                    case "LT":
                        mStepLeftTarget1 = robotDrive.leftMotor1.getCurrentPosition() - (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepLeftTarget2 = robotDrive.leftMotor2.getCurrentPosition() - (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepRightTarget1 = robotDrive.rightMotor1.getCurrentPosition() + (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepRightTarget2 = robotDrive.rightMotor2.getCurrentPosition() + (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        break;
                    case "RT":
                        mStepLeftTarget1 = robotDrive.leftMotor1.getCurrentPosition() + (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepLeftTarget2 = robotDrive.leftMotor2.getCurrentPosition() + (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepRightTarget1 = robotDrive.rightMotor1.getCurrentPosition() - (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        mStepRightTarget2 = robotDrive.rightMotor2.getCurrentPosition() - (int)(0.5 * Double.parseDouble(mRobotCommand.substring(2)) * COUNTS_PER_DEGREE);
                        break;
                }
                if (debug >= 3)
                {
                    fileLogger.writeEvent("TankTurnStep()","Current LPosition1:- " + robotDrive.leftMotor1.getCurrentPosition() + "mStepLeftTarget1:-   " +mStepLeftTarget1 );
                    fileLogger.writeEvent("TankTurnStep()","Current LPosition2:- " + robotDrive.leftMotor2.getCurrentPosition() + "mStepLeftTarget2:-   " +mStepLeftTarget2 );
                    fileLogger.writeEvent("TankTurnStep()","Current RPosition1:- " + robotDrive.rightMotor1.getCurrentPosition() + "mStepRightTarget1:- " + mStepRightTarget1 );
                    fileLogger.writeEvent("TankTurnStep()","Current RPosition2:- " + robotDrive.rightMotor2.getCurrentPosition() + "mStepRightTarget2:- " + mStepRightTarget2 );
                }

                // pass target position to motor controller
                robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget1);
                robotDrive.leftMotor2.setTargetPosition(mStepLeftTarget2);
                robotDrive.rightMotor1.setTargetPosition(mStepRightTarget1);
                robotDrive.rightMotor2.setTargetPosition(mStepRightTarget2);
                // set motor controller to mode
                robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(.5));
                if (debug >= 2)
                {
                    fileLogger.writeEvent("TankTurnStep()","mStepLeftTarget1 :- " + mStepLeftTarget1  );
                    fileLogger.writeEvent("TankTurnStep()","mStepLeftTarget2 :- " + mStepLeftTarget2  );
                    fileLogger.writeEvent("TankTurnStep()","mStepRightTarget1:- " + mStepRightTarget1  );
                    fileLogger.writeEvent("TankTurnStep()","mStepRightTarget2:- " + mStepRightTarget2  );
                }

                mCurrentTankTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                //if (robotDrive.leftMotor.isBusy() || robotDrive.rightMotor.isBusy()) {

                if (debug >= 3)
                {
                    fileLogger.writeEvent("TankTurnStep()","Running         " );
                    fileLogger.writeEvent("TankTurnStep()","Current LPosition1:- " + robotDrive.leftMotor1.getCurrentPosition() + " LTarget1:- " + mStepLeftTarget1);
                    fileLogger.writeEvent("TankTurnStep()","Current LPosition2:- " + robotDrive.leftMotor2.getCurrentPosition() + " LTarget2:- " + mStepLeftTarget2);
                    fileLogger.writeEvent("TankTurnStep()","Current RPosition1:- " + robotDrive.rightMotor1.getCurrentPosition() + " RTarget1:- " + mStepRightTarget1);
                    fileLogger.writeEvent("TankTurnStep()","Current RPosition2:- " + robotDrive.rightMotor2.getCurrentPosition() + " RTarget2:- " + mStepRightTarget2);
                }

                telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget1, mStepRightTarget1);
                telemetry.addData("Path2", "Running at %7d :%7d", robotDrive.leftMotor1.getCurrentPosition(), robotDrive.rightMotor1.getCurrentPosition());
                telemetry.addData("Path3", "Running at %7d :%7d", robotDrive.leftMotor2.getCurrentPosition(), robotDrive.rightMotor2.getCurrentPosition());

                if (!robotDrive.leftMotor1.isBusy() || (!robotDrive.rightMotor1.isBusy()))
                {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("TankTurnStep()","Complete         " );
                    }
                    setDriveMotorPower(0);
                    mCurrentTankTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }
    //----------------------------------------------------------------------------------------------
    //code added 11/27/16 to find color of beacon and press button if the beacon is same as alliance
    //color.
    //
    //----------------------------------------------------------------------------------------------
    private void setBeaconAllianceColor()
    {
        switch(mBeaconAllianceColorState){
            case  STATE_INIT: {
                ultaDistance = 0;
                OpticalDist = 0;
                ultaDistanceTarget = (float) 3;
                mStepDistanceTempSP = Double.parseDouble(mRobotCommand.substring(2));
                BeaconAllianceColorBlue = false;
                BeaconAllianceColorRed = false;
                isBeaconColorRed = false;
                isBeaconColorBlue = false;
                if (debug >= 3) {
                    fileLogger.writeEvent("Beacon Alliance Color State Init()","Beacon Alliance BLUE:- " +  BeaconAllianceColorBlue  );
                    fileLogger.writeEvent("Beacon Alliance Color State Init()","Beacon Alliance RED:- " +  BeaconAllianceColorRed  );
                    fileLogger.writeEvent("Beacon Alliance Color State Init()","Beacon Alliance Blue:- " +  isBeaconColorRed  );
                    fileLogger.writeEvent("Beacon Alliance Color State Init()","Beacon Alliance Blue:- " +  isBeaconColorBlue  );
                }
                mBeaconAllianceColorState = stepState.STATE_RUNNING;
            }//end INIT state
            break;
            case STATE_RUNNING: {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                ultaDistance = (range1Cache[0] & 0xFF);
                OpticalDist = (range1Cache[1] & 0xFF);
                if (mRobotParm1 == 1.0) {
                    BeaconAllianceColorBlue = true;
                }else if (mRobotParm1 == 2.0){
                    BeaconAllianceColorRed = true;
                }
                // RED ON,  BLUE OFF
                if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                    isBeaconColorRed = true;
                    //RED OFF, BLUE ON
                } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                    isBeaconColorBlue = true;
                    //RED OFF and BLUE OFF
                } else {
                    isBeaconColorRed = false;
                    isBeaconColorBlue = false;
                }
                if(BeaconAllianceColorBlue && isBeaconColorBlue ){
                    if (ultaDistance < ultaDistanceTarget){//MOVE ROBOT
                        setDriveMotorPower(Math.abs(.2));
                    }
                    else{
                        setDriveMotorPower(Math.abs(0));
                    }

                    //mStepLeftTarget1 = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepDistanceTempSP * COUNTS_PER_DEGREE);
                    //mStepLeftTarget2 = robotDrive.leftMotor2.getCurrentPosition() + (int) (mStepDistanceTempSP * COUNTS_PER_DEGREE);
                    //mStepRightTarget1 = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepDistanceTempSP * COUNTS_PER_DEGREE);
                    //mStepRightTarget2 = robotDrive.rightMotor2.getCurrentPosition() + (int) (mStepDistanceTempSP * COUNTS_PER_DEGREE);
                    // pass target position to motor controller
                    //robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget1);
                    //robotDrive.leftMotor2.setTargetPosition(mStepLeftTarget2);
                    //robotDrive.rightMotor1.setTargetPosition(mStepRightTarget1);
                    //robotDrive.rightMotor2.setTargetPosition(mStepRightTarget2);

                    // set motor controller to mode
                    //robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    //setDriveMotorPower(Math.abs(.2));
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("Beacon Color is Blue()","Drive Complete         " );
                    }

                } else if (BeaconAllianceColorRed && isBeaconColorRed) {
                    //move robot
                    if (ultaDistance < ultaDistanceTarget){//MOVE ROBOT
                        setDriveMotorPower(Math.abs(.2));
                    }
                    else{
                        setDriveMotorPower(Math.abs(0));
                    }
                    //mStepLeftTarget1 = robotDrive.leftMotor1.getCurrentPosition() + (int) (mStepDistanceTempSP  * COUNTS_PER_DEGREE);
                    //mStepLeftTarget2 = robotDrive.leftMotor2.getCurrentPosition() + (int) (mStepDistanceTempSP  * COUNTS_PER_DEGREE);
                    //mStepRightTarget1 = robotDrive.rightMotor1.getCurrentPosition() + (int) (mStepDistanceTempSP  * COUNTS_PER_DEGREE);
                    //mStepRightTarget2 = robotDrive.rightMotor2.getCurrentPosition() + (int) (mStepDistanceTempSP  * COUNTS_PER_DEGREE);
                    // pass target position to motor controller
                    //robotDrive.leftMotor1.setTargetPosition(mStepLeftTarget1);
                    //robotDrive.leftMotor2.setTargetPosition(mStepLeftTarget2);
                    //robotDrive.rightMotor1.setTargetPosition(mStepRightTarget1);
                    //robotDrive.rightMotor2.setTargetPosition(mStepRightTarget2);
                    // set motor controller to mode
                    //robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // set power on motor controller to start moving
                    //setDriveMotorPower(Math.abs(.2));
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("Beacon Color is Red()","Drive Complete         " );
                    }
                }
                // if (!robotDrive.leftMotor1.isBusy() || (!robotDrive.rightMotor1.isBusy()))
                //{
                //if (debug >= 3)
                //{
                //fileLogger.writeEvent("Beacon Push()","Complete         " );
                //}
                //setDriveMotorPower(0);
                mCurrentTankTurnState = stepState.STATE_COMPLETE;
                //}
                if (debug >= 3) {
                    fileLogger.writeEvent("Beacon Alliance Color State Complete()","Beacon Alliance BLUE:- " +  BeaconAllianceColorBlue  );
                    fileLogger.writeEvent("Beacon Alliance Color State Complete()","Beacon Alliance RED:- " +  BeaconAllianceColorRed  );
                    fileLogger.writeEvent("Beacon Alliance Color State Complete()","Beacon Color is Red:- " +  isBeaconColorRed  );
                    fileLogger.writeEvent("Beacon Alliance Color State Complete()","Beacon Color is Blue:- " +  isBeaconColorBlue  );
                }
            }//end RUNNING state
            break;
        }//end switch
    }//end method

    private void setTurnRobot()
    {
        switch (mTurnRobotState){
            case STATE_INIT:
            {
                mRobotTurnAngle = Double.parseDouble(mRobotCommand.substring(2));
                mTurnRobotState = stepState.STATE_RUNNING;
                turnAbsolute = 0;
                gyroTurnSpeed = 0;
            }
            break;
            case STATE_RUNNING:
            {
                turnAbsolute = mRobotTurnAngle + (gyro.getIntegratedZValue() * GYRO_CORRECTION_MULTIPLIER); //Target
                gyrozAccumulated = gyro.getIntegratedZValue() * GYRO_CORRECTION_MULTIPLIER;  //Set variables to gyro readings
                gyroTurnSpeed = .4;
                while (Math.abs(gyrozAccumulated - turnAbsolute) > 3) {  //Continue while the robot direction is further than three degrees from the target
                    if (gyrozAccumulated > turnAbsolute) {  //if gyro is positive, we will turn right
                        setDriveLeftMotorPower(gyroTurnSpeed);
                        setDriveRightMotorPower(-gyroTurnSpeed);
                    }
                    if (gyrozAccumulated < turnAbsolute) {  //if gyro is positive, we will turn left
                        setDriveLeftMotorPower(-gyroTurnSpeed);
                        setDriveRightMotorPower(gyroTurnSpeed);
                    }
                    //gyrozAccumulated = gyro.getIntegratedZValue() * GYRO_CORRECTION_MULTIPLIER;;  //Set variables to gyro readings
                    //telemetry.addData("1. accu", gyrozAccumulated);
                }
                setDriveMotorPower(0);
                mTurnRobotState = stepState.STATE_COMPLETE;
            }
            break;
        }
    }//end METHOD
    //----------------------------------------------------------------------------------------------
    //code added 11/2/16 to shoot particles
    //
    //
    //----------------------------------------------------------------------------------------------
    private void setParticleShooter()
    {
        switch (mShootParticleState) {
            case STATE_INIT: {
                //robotDrive.sweeper.setPower(0);
                mShootParticleState = stepState.STATE_RUNNING;
                //if (debug >= 3) {
                //fileLogger.writeEvent("DelayStep()", "Init Delay Time    " + mStepDelay);
                //fileLogger.writeEvent("PARTICLE STATE MACHINE",  "INIT");
                // }
            }
            break;
            case STATE_RUNNING: {
                if (mStateTime.seconds() <= mRobotParm1) {
                    //robotDrive.sweeper.setPower(1);
                    //if (debug >= 3 ) {
                    //fileLogger.writeEvent("DelayStep()", "Init Delay Time    " + mStepDelay);
                    // fileLogger.writeEvent("PARTICLE STATE MACHINE", "RUNNING");
                    //}
                } else {
                    //if (debug >= 3) {
                    //fileLogger.writeEvent("Particle Shooter", "Stop Particle State Complete         ");
                    //fileLogger.writeEvent("PARTICLE STATE MACHINE", "COMPLETE");
                    //}
                    //robotDrive.sweeper.setPower(0);
                    mShootParticleState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    private void DelayStep ()
    {
        switch (mCurrentDelayState) {
            case STATE_INIT: {
                mStepDelay = Integer.parseInt(mRobotCommand.substring(2));
                mCurrentDelayState = stepState.STATE_RUNNING;
                if (debug >= 3) {
                    fileLogger.writeEvent("DelayStep()", "Init Delay Time    " + mStepDelay);
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= mStepDelay) {
                    if (debug >= 3) {
                        fileLogger.writeEvent("DelayStep()", "Complete         ");
                    }
                    mCurrentDelayState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    //set the drive motors power, both left and right
    private void setDriveMotorPower (double power) {
        setDriveRightMotorPower(power);
        setDriveLeftMotorPower(power);
    }

    //set the right drive motors power
    private void setDriveRightMotorPower (double power) {
        robotDrive.rightMotor1.setPower(power);
        robotDrive.rightMotor2.setPower(power);
    }

    //set the left motors drive power
    private void setDriveLeftMotorPower (double power) {
        robotDrive.leftMotor1.setPower(power);
        robotDrive.leftMotor2.setPower(power);
    }

}