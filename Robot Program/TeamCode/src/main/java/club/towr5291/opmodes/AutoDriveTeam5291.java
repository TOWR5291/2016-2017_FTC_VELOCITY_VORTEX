/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Image;
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
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.sixValues;
import club.towr5291.functions.AStarGetPathVer2;
import club.towr5291.functions.BeaconAnalysisOCV;
import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryStateSegAuto;
import club.towr5291.robotconfig.HardwareArmMotors;
import club.towr5291.robotconfig.HardwareDriveMotors;
/*
TOWR 5291 Autonomous
Copyright (c) 2016 TOWR5291
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Written by Ian Haden October 2016
*/

@Autonomous(name="Ians Autonomous Drive", group="5291")
public class AutoDriveTeam5291 extends LinearOpMode
{
    //set up TAG for logging prefic, this info will appear first in every log statemend
    private static final String TAG = "AutoDriveTeam5291";

    //variable for pathvales when processing the A*pathfinder
    public AStarGetPathVer2 getPathValues = new AStarGetPathVer2();

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String allianceColor;
    private String alliancePosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;

    // Declare OpMode members.
    private HardwareDriveMotors robotDrive   = new HardwareDriveMotors();   // Use a Pushbot's hardware
    private HardwareArmMotors armDrive   = new HardwareArmMotors();   // Use a Pushbot's hardware


    private ElapsedTime     runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    private FileLogger fileLogger;
    private int debug = 3;

    //set up range sensor variables
    private ModernRoboticsI2cRangeSensor rangeSensorLeft;
    private ModernRoboticsI2cRangeSensor rangeSensorRight;
    private boolean rangeError = false;

    //set up colour sensor variables
    ColorSensor colorSensor;    // Hardware Device Object
    private boolean colorError = false;

    //set up Gyro variables
    private boolean gyroError = false;
    private ModernRoboticsI2cGyro gyro;                 // Hardware Device Object
    private int gyroXVal, gyroYVal, yroZVal = 0;        // Gyro rate Values
    private int gyroHeading = 0;                        // Gyro integrated heading
    private int gyroAngleZ = 0;
    private boolean gyroLastResetState = false;
    private boolean gyroCurResetState  = false;

    //set up robot variables
    private double     COUNTS_PER_MOTOR_REV;            // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
    private double     DRIVE_GEAR_REDUCTION;            // This is < 1.0 if geared UP, Tilerunner is geared up
    private double     WHEEL_DIAMETER_INCHES;           // For figuring circumference
    private double     WHEEL_ACTUAL_FUDGE;              // Fine tuning amount
    private double     COUNTS_PER_INCH;
    private double     ROBOT_TRACK;                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
    private double     COUNTS_PER_DEGREE;
    private double     WHEEL_TURN_FUDGE;
    //vuforia localisation variables
    OpenGLMatrix lastLocation = null;
    private double localisedRobotX;
    private double localisedRobotY;
    private double localisedRobotBearing;
    private boolean localiseRobotPos;

    //define each state for the step.  Each step should go through some of the states below
    // set up the variables for the state engine
    private int mCurrentStep = 1;                                                       // Current Step in State Machine.
    private int mCurrentAStarStep = 1;                                                  // Current Step in AStar State Machine.
    private stepState mCurrentStepState;                                                // Current State Machine State.
    private stepState mCurrentDriveState;                                               // Current State of Drive.
    private stepState mCurrentTankTurnState;                                            // Current State of Tank Turn.
    private stepState mCurrentPivotTurnState;                                           // Current State of Pivot Turn.
    private stepState mCurrentRadiusTurnState;                                          // Current State of Radius Turn.
    private stepState mCurrentDelayState;                                               // Current State of Delay (robot doing nothing)
    private stepState mCurrentVuforiaState;                                             // Current State of Vuforia Localisation
    private stepState mCurrentVuforiaMoveState;                                         // Current State of Vuforia Move
    private stepState mCurrentVuforiaTurnState;                                         // Current State of Vuforia Turn
    private stepState mCurrentBeaconColourState;                                        // Current State of Beacon Colour
    private stepState mCurrentAttackBeaconState;                                        // Current State of Attack Beacon
    private stepState mCurrentShootBallState;                                           // Current State of Shooting Ball in Vortex
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
    private boolean readyToCapture = false;

    //variable for the state engine, decalred here so they are accessible throughout the entire opmode with having to pass them through each function
    private int mStartPositionLeft1;                    //Left Motor 1  - start position of the robot in inches, starts from 0 to the end
    private int mStartPositionLeft2;                    //Left Motor 2  - start position of the robot in inches, starts from 0 to the end
    private int mStartPositionRight1;                   //Right Motor 1 - start position of the robot in inches, starts from 0 to the end
    private int mStartPositionRight2;                   //Right Motor 2 - start position of the robot in inches, starts from 0 to the end
    private int mStepLeftTarget1;                       //Left Motor 1   - encoder target position
    private int mStepLeftTarget2;                       //Left Motor 2   - encoder target position
    private int mStepRightTarget1;                      //Right Motor 1  - encoder target position
    private int mStepRightTarget2;                      //Right Motor 2  - encoder target position
    private double mStepTimeout;                        //Timeout value ofthe step, the step will abort if the timeout is reached
    private double mStepSpeed;                          //When a move command is executed this is the speed the motors will run at
    private String mRobotCommand;                       //The command the robot will execute, such as move forward, turn right etc
    private double mRobotParm1;                         //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mRobotParm2;                         //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mRobotParm3;                         //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mRobotParm4;                         //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mRobotParm5;                         //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mRobotParm6;                         //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private boolean mRobotStepComplete;                 //flag to write back to the step if we finish the step, currently not used
    private double mStepTurnL;                          //used when decoding the step, this will indicate if the robot is turning left
    private double mStepTurnR;                          //used when decoding the step, this will indicate if the robot is turning right
    private double mStepDistance;                       //used when decoding the step, this will indicate how far the robot is to move in inches
    private int mStepDelay;                             //used when decoding the step, this will indicate how long the delay is on ms.
    private boolean blnDisableVisionProcessing = false; //used when moving to disable vision to allow faster speed reading encoders.
    private boolean baseStepComplete = false;
    boolean armStepComplete = true;
    private ElapsedTime mStateTime = new ElapsedTime(); // Time into current state, used for the timeout

    //hashmap for the steps to be stored in.  A Hashmap is like a fancy array
    private HashMap<String,LibraryStateSegAuto> autonomousSteps = new HashMap<String,LibraryStateSegAuto>();
    private HashMap<String,String> powerTable = new HashMap<String,String>();

    //OpenCV Stuff
    private BeaconAnalysisOCV beaconColour = new BeaconAnalysisOCV();
    private Mat tmp = new Mat();
    private int captureLoop = 0;
    private int intNumberColourTries = 0;
    private Constants.BeaconColours mColour;

    //variable for the counter when loading steps into the hashap, each step must have a unique step ID
    private int loadStep = 1;

    //each robot speeds up and slows down at different rates
    //helps reduce over runs and
    //table for the tilerunner from AndyMark.  These values are for the twin 20 motors which makes the robot fast
    private void loadPowerTableTileRunner ()
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

    //table for the custom tanktread robot.  These values are for the twin 40 motors
    private void loadPowerTableTankTread ()
    {
        powerTable.put(String.valueOf(0.5), ".3");
        powerTable.put(String.valueOf(1), ".3");
        powerTable.put(String.valueOf(2), ".4");
        powerTable.put(String.valueOf(4), ".5");
        powerTable.put(String.valueOf(6), ".5");
        powerTable.put(String.valueOf(8), ".6");
        powerTable.put(String.valueOf(10), ".6");
        powerTable.put(String.valueOf(12), ".6");
        powerTable.put(String.valueOf(15), ".8");
    }

    private void loadStaticSteps ()
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
        // AS = AutoStar From Current Pos to X,Y
        // FN = Special Function
        // VL = Vuforia Localise
        // VM = Vuforia Move
        // AB = Attack the Beacon
        // Red Beacon 1 = (32, 36), (-1016, 914)
        // Red Beacon 2 = (32, 84), (-1016, -305)
        // Blue Beacon 1 = (36,32), (-914, 1016) <270
        // Blue Beacon 2 = (84,32), (305, 1016) <270
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %
        //loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "FW48", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "AS",  120,  110,   0,   12,   24,  270,  0.5);
        loadSteps(10, "RT90", 0,    0,    0,    0,    0,    0,  0.5);
        loadSteps(10, "LT90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "FW24", 0,    0,    0,    0,    0,    0,    1);
        //loadSteps(10, "RP90", 0,    0,    0,    0,    0,    0,  0.5);
        //loadSteps(10, "LP90", 0,    0,    0,    0,    0,    0,  0.5);
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
        // AS = AutoStar From Current Pos to X,Y
        // FN = Special Function
        // VL = Vuforia Localise
        // VM = Vuforia Move
        // AB = Attack the Beacon
        // Red Beacon 1 = (32, 36), (-1016, 914)
        // Red Beacon 2 = (32, 84), (-1016, -305)
        // Blue Beacon 1 = (36,32), (-914, 1016) <270
        // Blue Beacon 2 = (84,32), (305, 1016) <270
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %

        //loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        //loadSteps(10,  "BC",  0,    0,    0,    0,    0,    0,    0);
        //loadSteps(10,  "AB",  0,    0,    0,    0,    0,    0,    0);

        loadSteps(7, "DL3000", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW8",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "LT46", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "SB" , 2500,  0,    0,    0,    0,    0,    0  );
        loadSteps(10, "LT16", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW52", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "RT31", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW40", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "RT31", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW5", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "LT90", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VM",   0,    0,    0,  -1250, 914,  180,   0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(10, "LT141", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW60",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "BC",   0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "AB",   0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "LT126", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW48",  0,    0,    0,    0,    0,    0,    0.6);

        //loadSteps(30, "AS",   0,    0,    0,   12,   24,  270,  0.5);
        //loadSteps(10, "VM",   0,    0,    0,  -1016, -305,  180,   0.6);

        loadSteps(2, "DL0",   0,    0,    0,    0,    0,    0,    0);
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
        // AS = AutoStar From Current Pos to X,Y
        // FN = Special Function
        // VL = Vuforia Localise
        // VM = Vuforia Move
        // AB = Attack the Beacon
        // SB = Shoot Ball
        // Red Beacon 1 = (32, 36), (-1016, 914)
        // Red Beacon 2 = (32, 84), (-1016, -305)
        // Blue Beacon 1 = (36,32), (-914, 1016) <270
        // Blue Beacon 2 = (84,32), (305, 1016) <270
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %


        loadSteps(10, "SB" , 2500,  0,    0,    0,    0,    0,    0  );
        loadSteps(15, "FW8",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "LT46", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW37",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "LT36", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VM",   0,    0,    0,  -1150, -350,  180,   0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(10,  "BC",  0,    0,    0,    0,    0,    0,    0);
        loadSteps(10,  "AB",  0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "RT96", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW36",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "LT100", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VM",   0,    0,    0,  -1150, 840,  180,   0.6);
        loadSteps(10, "LT26", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(10, "BC",   0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "AB",   0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "LT141", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW60",  0,    0,    0,    0,    0,    0,    0.6);

        //leave this delay of 0 in there, inserting a step doesn't work if inserting when at the last step
        loadSteps(2, "DL0",   0,    0,    0,    0,    0,    0,    0);
    }

    private void loadStaticStepsBlueLeft ()
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
        // AS = AutoStar From Current Pos to X,Y
        // FN = Special Function
        // VL = Vuforia Localise
        // VM = Vuforia Move
        // AB = Attack the Beacon
        // Red Beacon 1 = (32, 36), (-1016, 914)
        // Red Beacon 2 = (32, 84), (-1016, -305)
        // Blue Beacon 1 = (36,32), (-914, 1016) <270
        // Blue Beacon 2 = (84,32), (305, 1016) <270
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %

        //loadSteps(2, "DL500", 0,    0,    0,    0,    0,    0,    0);
        //loadSteps(10,  "BC",  0,    0,    0,    0,    0,    0,    0);
        //loadSteps(10,  "AB",  0,    0,    0,    0,    0,    0,    0);

        loadSteps(7, "DL3000", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW8",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "RT46", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "SB" , 2500,  0,    0,    0,    0,    0,    0  );
        loadSteps(10, "RT16", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW49", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "LT31", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW40", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "LT31", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "FW5", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(15, "RT91", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VM",   0,    0,    0,  -1250, 914,  180,   0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,   180,   0.7);
        loadSteps(10, "RT140", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW60",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "BC",   0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "AB",   0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "RT126", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(15, "FW48",  0,    0,    0,    0,    0,    0,    0.6);

        //loadSteps(30, "AS",   0,    0,    0,   12,   24,  270,  0.5);
        //loadSteps(10, "VM",   0,    0,    0,  -1016, -305,  180,   0.6);

        loadSteps(2, "DL0",   0,    0,    0,    0,    0,    0,    0);
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
        // AS = AutoStar From Current Pos to X,Y
        // FN = Special Function
        // VL = Vuforia Localise
        // VM = Vuforia Move
        // AB = Attack the Beacon
        // SB = Shoot Ball
        // Red Beacon 1 = (32, 36), (-1016, 914)
        // Red Beacon 2 = (32, 84), (-1016, -305)
        // Blue Beacon 1 = (36,32), (-914, 1016) <270
        // Blue Beacon 2 = (84,32), (305, 1016) <270
        //       time, comm,  parm, parm, parm, parm, parm, parm, powe
        //       out   and    1     2     3     4     5     6     r
        //        s                                               %

        loadSteps(10, "SB" , 2500,  0,    0,    0,    0,    0,    0  );
        loadSteps(15, "FW8",  0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "RT46", 0,    0,    0,    0,    0,    0,    0.5);
        loadSteps(15, "FW45", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "RT41", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VM",   0,    0,    0,   330, 1200,   90,   0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,    90,   0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,    90,   0.7);
        loadSteps(10,  "BC",  0,    0,    0,    0,    0,    0,    0);
        loadSteps(10,  "AB",  0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "LT106", 0,    0,    0,    0,    0,    0,    0.5);
        loadSteps(15, "FW47", 0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(10, "RT96",0,    0,    0,    0,    0,    0,    0.6);
        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VM",   0,    0,    0,  -900, 1200,   90,   0.6);
        loadSteps(10, "RT26", 0,    0,    0,    0,    0,    0,    0.5);
        loadSteps(2, "DL700", 0,    0,    0,    0,    0,    0,    0);
        loadSteps(10, "VT",   0,    0,    0,    0,    0,    90,   0.7);
//        loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
//        loadSteps(10, "VT",   0,    0,    0,    0,    0,    90,   0.7);
        loadSteps(10,  "BC",  0,    0,    0,    0,    0,    0,    0);
        loadSteps(10,  "AB",  0,    0,    0,    0,    0,    0,    0);
        //loadSteps(2, "DL600", 0,    0,    0,    0,    0,    0,    0);
        //loadSteps(10, "VM",   0,    0,    0,    0,    0,    90,   0.6);
        loadSteps(10, "RT126", 0,    0,    0,    0,    0,    0,    0.5);
        loadSteps(10, "FW50", 0,    0,    0,    0,    0,    0,    0.65);

        //leave this delay of 0 in there, inserting a step doesn't work if inserting when at the last step
        loadSteps(2, "DL0",   0,    0,    0,    0,    0,    0,    0);
    }

    private void loadSteps(int timeOut, String command, int parm1, int parm2, int parm3, int parm4, int parm5, int parm6, double power)
    {
        autonomousSteps.put(String.valueOf(loadStep), new LibraryStateSegAuto (loadStep, timeOut, command, parm1, parm2, parm3, parm4, parm5, parm6, power, false));
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
        //load all the vuforia stuff
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AVATY7T/////AAAAGQJxfNYzLUgGjSx0aOEU0Q0rpcfZO2h2sY1MhUZUr+Bu6RgoUMUP/nERGmD87ybv1/lM2LBFDxcBGRHkXvxtkHel4XEUCsNHFTGWYcVkMIZqctQsIrTe13MnUvSOfQj8ig7xw3iULcwDpY+xAftW61dKTJ0IAOCxx2F0QjJWqRJBxrEUR/DfQi4LyrgnciNMXCiZ8KFyBdC63XMYkQj2joTN579+2u5f8aSCe8jkAFnBLcB1slyaU9lhnlTEMcFjwrLBcWoYIFAZluvFT0LpqZRlS1/XYf45QBSJztFKHIsj1rbCgotAE36novnAQBs74ewnWsJifokJGOYWdFJveWzn3GE9OEH23Y5l7kFDu4wc";
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
                .translation((mmBotWidth/2), 0, 300)
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
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        alliancePosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "Right");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");
        LibraryStateSegAuto processingSteps = new LibraryStateSegAuto(0,0,"",0,0,0,0,0,0,0,false);
        sixValues[] pathValues = new sixValues[1000];
        A0Star a0Star = new A0Star();
        String fieldOutput;
        HashMap<String,LibraryStateSegAuto> autonomousStepsAStar = new HashMap<>();

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
            case "TileRunner-2x40":   //Velocity Vortex Competition Base
                COUNTS_PER_MOTOR_REV    = 1120 ;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION    = 0.78 ;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES   = 4.0 ;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE      = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
                ROBOT_TRACK             = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE        = 1.024;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE       = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                loadPowerTableTileRunner();                                                         //load the power table
                break;
            case "Tank Tread-2x40":   //for tank tread base
                COUNTS_PER_MOTOR_REV    = 1120 ;                                                    // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION    = 1.0 ;                                                     // Tank Tread is 1:1 ration
                WHEEL_DIAMETER_INCHES   = 3.75 ;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE      = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
                ROBOT_TRACK             = 18;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                WHEEL_TURN_FUDGE        = 1.12;                                                        // Fine tuning amount
                COUNTS_PER_DEGREE       = (((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360) * WHEEL_TURN_FUDGE;
                loadPowerTableTankTread();                                                          //load the power table
                break;
            default:  //default for competition TileRunner-2x40
                COUNTS_PER_MOTOR_REV    = 1120 ;                                                     // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
                DRIVE_GEAR_REDUCTION    = 1.28 ;                                                    // This is < 1.0 if geared UP, Tilerunner is geared up
                WHEEL_DIAMETER_INCHES   = 4.0 ;                                                     // For figuring circumference
                WHEEL_ACTUAL_FUDGE      = 1;                                                        // Fine tuning amount
                COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) * WHEEL_ACTUAL_FUDGE ;
                ROBOT_TRACK             = 16.5;                                                     //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
                COUNTS_PER_DEGREE       = ((2 * 3.1415 * ROBOT_TRACK) * COUNTS_PER_INCH) / 360;
                loadPowerTableTileRunner();                                                         //load the power table
                break;
        }

        //show options on the driver station phone
        telemetry.addData("Alliance   ",  allianceColor);
        telemetry.addData("Start Pos  ",  alliancePosition);
        telemetry.addData("Start Del  ",  delay);
        telemetry.addData("# Beacons  ",  numBeacons);
        telemetry.addData("Robot      ",  robotConfig);

        //need to load initial step of a delay based on user input
        loadSteps(delay + 2, "DL" + (delay * 1000), 0,    0,    0,    0,    0,    0,  0);

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
                        loadStaticStepsBlueLeft();                                                              //load all the steps into the hashmaps
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
        try {
            // get a reference to a Modern Robotics GyroSensor object.
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
            // calibrate the gyro, this takes a few seconds
            gyro.calibrate();
        } catch (Exception e) {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "Gyro Error " +  e.getMessage());
            }
            gyroError = true;
        }

        /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robotDrive.init(hardwareMap);
        armDrive.init(hardwareMap);

        //get a reference to the range sensors
        try {
            rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensorrangeleft");
            rangeSensorRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensorrangeright");
        } catch (Exception e) {
            if (debug >= 1)
            {
                fileLogger.writeEvent(TAG, "range Error " +  e.getMessage());
            }
            rangeError = true;
        }

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
        idle();

        robotDrive.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotDrive.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTankTurnState = stepState.STATE_COMPLETE;
        mCurrentDriveState = stepState.STATE_COMPLETE;
        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
        mCurrentRadiusTurnState = stepState.STATE_COMPLETE;
        mCurrentDelayState = stepState.STATE_COMPLETE;
        mCurrentVuforiaState = stepState.STATE_COMPLETE;
        mCurrentVuforiaMoveState = stepState.STATE_COMPLETE;
        mCurrentVuforiaTurnState = stepState.STATE_COMPLETE;
        mCurrentAttackBeaconState = stepState.STATE_COMPLETE;
        mCurrentBeaconColourState = stepState.STATE_COMPLETE;
        mCurrentShootBallState = stepState.STATE_COMPLETE;

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

        //set up variable for our capturedimage
        Image rgb = null;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //activate vuforia
        velocityVortex.activate();

        //the main loop.  this is where the action happens
        while (opModeIsActive())
        {

            if (!blnDisableVisionProcessing) {
                //start capturing frames for analysis
                if (readyToCapture) {
                    captureLoop ++;

                    VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
                    long numImages = frame.getNumImages();

                    for (int i = 0; i < numImages; i++) {
                        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            rgb = frame.getImage(i);
                            break;
                        }
                    }
                    /*rgb is now the Image object that we’ve used in the video*/
                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(rgb.getPixels());

                    //put the image into a MAT for OpenCV
                    Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
                    Utils.bitmapToMat(bm, tmp);

                    //close the frame, prevents memory leaks and crashing
                    frame.close();

                    //analyse the beacons
                    //Constants.BeaconColours Colour = beaconColour.beaconAnalysisOCV(tmp, loop));
                    mColour = beaconColour.beaconAnalysisOCV(tmp, captureLoop);
                    Log.d("OPENCV","Returned " + mColour);
                    telemetry.addData("Beacon ", mColour);

                }

                //use vuforia to get locations informatio
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
                    if (localisedRobotBearing < 0) {
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
            }

            //if (debug >= 3)
            //{
            //    if (fileLogger != null)
            //        fileLogger.writeEvent(TAG, "mCurrentStepState:- " + mCurrentStepState + " mCurrentStepState " + mCurrentStepState);
            //}

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
                    //RadiusTurnStep();
                    DriveStep();
                    VuforiaLocalise();
                    VuforiaMove();
                    VuforiaTurn();
                    AttackBeacon();
                    BeaconColour();
                    ShootBallStep();

                    if ((mCurrentDelayState == stepState.STATE_COMPLETE) &&
                            (mCurrentBeaconColourState == stepState.STATE_COMPLETE) &&
                            (mCurrentAttackBeaconState == stepState.STATE_COMPLETE) &&
                            (mCurrentVuforiaTurnState == stepState.STATE_COMPLETE) &&
                            (mCurrentVuforiaState == stepState.STATE_COMPLETE) &&
                            (mCurrentVuforiaMoveState == stepState.STATE_COMPLETE)  &&
                            (mCurrentDriveState == stepState.STATE_COMPLETE) &&
                            (mCurrentPivotTurnState == stepState.STATE_COMPLETE) &&
                            (mCurrentTankTurnState == stepState.STATE_COMPLETE) &&
                            (mCurrentShootBallState == stepState.STATE_COMPLETE) &&
                            (mCurrentRadiusTurnState == stepState.STATE_COMPLETE))
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
                case STATE_ASTAR_PRE_INIT:
                {
                    mCurrentAStarStep = 1;                                          //init the Step for AStar
                    //get start point
                    //get end point
                    int startX = (int)processingSteps.getmRobotParm1();
                    int startY = (int)processingSteps.getmRobotParm2();
                    int startZ = (int)processingSteps.getmRobotParm3();
                    int endX = (int)processingSteps.getmRobotParm4();
                    int endY = (int)processingSteps.getmRobotParm5();
                    int endDir = (int)processingSteps.getmRobotParm6();

                    //before using the path in the command lets check if we can localise
                    if (lastLocation != null) {
                        //lets get locations for AStar, direction is most important
                        //x and y position for Vuforia are in mm, AStar in Inches
                        //counter clockwise rotation (x,y) = (-x, y)
                        //origin is center of field
                        //Astar is top right so need to add in 6 feet to each value
                        startX = (int)(localisedRobotX/25.4) + 72;
                        startY = (int)(localisedRobotY/25.4) + 72;
                        //need to rotate the axis -90 degrees
                        startZ = (int)localisedRobotBearing;
                        if ((startZ > 357) && (startZ < 3))
                            startZ = 90;
                        else
                        if ((startZ > 267) && (startZ < 273))
                            startZ = 0;
                        else
                        if ((startZ > 177) && (startZ < 183))
                            startZ = 270;
                        else
                        if ((startZ > 87) && (startZ < 93))
                            startZ = 180;
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "AStar Init - Localised Values");
                            fileLogger.writeEvent(TAG, "AStar Init - localisedRobotX:        " + localisedRobotX);
                            fileLogger.writeEvent(TAG, "AStar Init - localisedRobotY:        " + localisedRobotY);
                            fileLogger.writeEvent(TAG, "AStar Init - localisedRobotBearing:  " + localisedRobotBearing);
                            fileLogger.writeEvent(TAG, "AStar Init - startX:                 " + startX);
                            fileLogger.writeEvent(TAG, "AStar Init - startY:                 " + startY);
                            fileLogger.writeEvent(TAG, "AStar Init - startZ:                 " + startZ);
                        }
                    }

                    //process path
                    pathValues = getPathValues.findPathAStar(startX, startY, startZ, endX, endY, endDir);  //for enhanced
                    if (debug >= 1)
                    {
                            fileLogger.writeEvent(TAG, "AStar Path - length:                 " + pathValues.length);
                    }


                    String[][] mapComplete = new String[A0Star.FIELDWIDTH][A0Star.FIELDWIDTH];

                    //write path to logfile to verify path
                    for (int y = 0; y < a0Star.fieldLength; y++)
                    {
                        for (int x = 0; x < a0Star.fieldWidth; x++)
                        {
                            switch (allianceColor) {
                                case "Red":
                                    if (a0Star.walkableRed[y][x]) {
                                        mapComplete[y][x] = "1";
                                        if ((x == startX) && (y == startY))
                                            mapComplete[y][x] = "S";
                                        else if ((x == endX) && (y == endY))
                                            mapComplete[y][x] = "E";
                                    } else {
                                            mapComplete[y][x] = "0";
                                    }
                                    break;

                                case "Blue":
                                    if (a0Star.walkableBlue[y][x]) {
                                        mapComplete[y][x] = "1";
                                        if ((x == startX) && (y == startY))
                                            mapComplete[y][x] = "S";
                                        else if ((x == endX) && (y == endY))
                                            mapComplete[y][x] = "E";
                                    } else {
                                        if ((x == startX) && (y == startY)) {
                                            mapComplete[y][x] = "1";
                                        } else {
                                            mapComplete[y][x] = "0";
                                        }
                                    }
                                    break;

                                default:
                                    if (a0Star.walkable[y][x]) {
                                        mapComplete[y][x] = "1";
                                        if ((x == startX) && (y == startY))
                                            mapComplete[y][x] = "S";
                                        else if ((x == endX) && (y == endY))
                                            mapComplete[y][x] = "E";
                                    }
                                    break;
                            }
                        }
                    }

                    //plot out path..
                    for (int i = 0; i < pathValues.length; i++)
                    {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG,"Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                        }
                        if (((int)pathValues[i].val1 == 0) && ((int)pathValues[i].val3 == 0) && ((int)pathValues[i].val2 == 0) && ((int)pathValues[i].val4 == 0))
                            break;
                        mapComplete[(int)pathValues[i].val3][(int)pathValues[i].val2] = "P";
                        if ((pathValues[i].val2 == startX) && (pathValues[i].val3 == startY))
                        {
                            mapComplete[(int) pathValues[i].val3][(int) pathValues[i].val2] = "S";
                        }
                    }
                    mapComplete[endY][endX] = "E";
                    fieldOutput ="";
                    for (int y = 0; y < a0Star.fieldLength; y++)
                    {
                        for (int x = 0; x < a0Star.fieldWidth; x++)
                        {
                            fieldOutput = "" + fieldOutput + mapComplete[y][x];
                        }
                        if (debug >= 2)
                        {
                            fileLogger.writeEvent(TAG, fieldOutput);
                        }
                        fieldOutput = "";
                    }

                    //load path in Hashmap
                    boolean dirChanged;
                    boolean processingAStarSteps = true;
                    int startSegment = 1;
                    int startStraightSection = 0;
                    int numberOfMoves = 0;
                    int key = 0;
                    int lastDirection = 0;
                    int lasti =0;
                    String strAngleChange = "RT00";
                    boolean endOfAStarSequenceFound = false;

                    while (processingAStarSteps)
                    {
                        numberOfMoves = 0;
                        for (int i = startSegment; i < pathValues.length; i++)
                        {
                            numberOfMoves ++;
                            if (debug >= 2)
                            {
                                fileLogger.writeEvent(TAG,"Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                            }
                            if (((int)pathValues[i].val1 == 0) && ((int)pathValues[i].val2 == 0) && ((int)pathValues[i].val3 == 0))
                            {
                                if (debug >= 2)
                                {
                                    fileLogger.writeEvent(TAG,"End Detected" );
                                }
                                //end of the sequence,
                                lastDirection = (int)pathValues[i-1].val4;
                                processingAStarSteps = false;
                                lasti = i;
                            }
                            //need to check if the first step is in a different direction that the start
                            if (i == 1) {
                                if (startZ != pathValues[i].val4) {  //need to turn
                                    strAngleChange = getAngle(startZ, (int) pathValues[i].val4);
                                    if (debug >= 2) {
                                        fileLogger.writeEvent(TAG, "First Step Need to turn Robot " + strAngleChange + " Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4);
                                        fileLogger.writeEvent(TAG, "Adding Command (" + key + ", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                                    }
                                    autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto(key, 10, strAngleChange, 0, 0, 0, 0, 0, 0, 1, false));
                                    key++;
                                    dirChanged = true;
                                } else {
                                    dirChanged = false;    //no change in direction
                                }
                            }
                            else  //work out the sequence not the first step
                            {
                                if (pathValues[i-1].val4 != pathValues[i].val4) {  //need to turn
                                    strAngleChange = getAngle((int)pathValues[i-1].val4, (int)pathValues[i].val4);
                                    dirChanged = true;
                                }
                                else
                                {
                                    dirChanged = false;    //no change in direction
                                }
                            }
                            if ((dirChanged) || (!processingAStarSteps))  //found end of segment
                            {
                                int AStarPathAngle;
                                if (i == 1)
                                {
                                    AStarPathAngle = startZ;
                                }
                                else
                                {
                                    AStarPathAngle = (int)pathValues[i-1].val4;
                                }
                                switch (AStarPathAngle)
                                {
                                    case 0:
                                    case 90:
                                    case 180:
                                    case 270:
                                        if (debug >= 2)
                                        {
                                            fileLogger.writeEvent(TAG,"Heading on a Straight line " + (numberOfMoves) + " Path");
                                            fileLogger.writeEvent(TAG,"Adding Command (" + key +", 10, " + "FW" + (numberOfMoves) + ", 0, 0, 0, 0, 0, 0, 0.8, false) ");
                                        }
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key, 10, "FW" + numberOfMoves , 0, 0, 0, 0, 0, 0, 0.8, false));
                                        numberOfMoves = 0;
                                        key++;
                                        break;
                                    case 45:
                                    case 135:
                                    case 225:
                                    case 315:
                                        if (debug >= 2)
                                        {
                                            fileLogger.writeEvent(TAG,"Heading on a Straight line " + (int) ((numberOfMoves) * 1.4142) + " Path");
                                            fileLogger.writeEvent(TAG, "Adding Command (" + key + ", 10, " + "FW" + (int) ((numberOfMoves) * 1.4142) + ", 0, 0, 0, 0, 0, 0, .8, false) ");
                                        }
                                        autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key, 10, "FW" + (int)(numberOfMoves * 1.4142 ) , 0,    0,    0,    0,    0,    0,    1,    false));
                                        numberOfMoves = 0;
                                        key++;
                                        break;
                                }
                                if (debug >= 2)
                                {
                                    fileLogger.writeEvent(TAG,"Need to turn Robot " + strAngleChange + " Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 + " Dir:= " + pathValues[i].val4 );
                                    fileLogger.writeEvent(TAG,"Adding Command (" + key +", 10, "+ strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                                }
                                autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key, 10, strAngleChange, 0, 0, 0, 0, 0, 0, 0.4, false));
                                key++;
                            }
                            if (!processingAStarSteps)
                                break;

                        }
                        //need to work out the direction we are facing and the required direction
                        if ((lastDirection != endDir) && (processingAStarSteps == false)) {
                            if (debug >= 2)
                            {
                                fileLogger.writeEvent(TAG,"Sraight Moves Robot End Of Sequence - Need to Trun Robot");
                            }
                            strAngleChange = getAngle((int)pathValues[lasti - 1].val4, endDir);
                            fileLogger.writeEvent(TAG,"Adding Command (" + key +", 10, " + strAngleChange + ", 0, 0, 0, 0, 0, 0, 1, false) ");
                            autonomousStepsAStar.put(String.valueOf(key), new LibraryStateSegAuto (key, 10,  strAngleChange, 0, 0, 0, 0, 0, 0, 0.4, false));
                            key++;
                        }
                    }
                    endOfAStarSequenceFound = false;
                    mCurrentStepState = stepState.STATE_ASTAR_INIT;
                }
                break;
                case STATE_ASTAR_INIT:
                {
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "About to check if step exists " + mCurrentAStarStep);
                    }
                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousStepsAStar.containsKey(String.valueOf(mCurrentAStarStep)))
                    {
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Step Exists TRUE " + mCurrentAStarStep + " about to get the values from the step");
                        }
                        processingSteps = autonomousStepsAStar.get(String.valueOf(mCurrentAStarStep));      //read the step from the hashmap
                        autonomousStepsAStar.remove(String.valueOf(mCurrentAStarStep));                     //remove the step from the hashmap
                        if (debug >= 1)
                        {
                            fileLogger.writeEvent(TAG, "Got the values for step " + mCurrentAStarStep + " about to decode and removed them");
                        }

                        //decode the step from hashmap
                        initAStarStep(processingSteps);
                    }
                    else  //if no steps left in hashmap then complete
                    {
                        mCurrentStepState = stepState.STATE_ASTAR_COMPLETE;
                    }
                }
                break;
                case STATE_ASTAR_RUNNING:
                {
                    //move robot according AStar hashmap
                    TankTurnStep();
                    PivotTurnStep();
                    RadiusTurnStep();
                    DriveStep();
                    if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentPivotTurnState == stepState.STATE_COMPLETE) && (mCurrentTankTurnState == stepState.STATE_COMPLETE) && (mCurrentRadiusTurnState == stepState.STATE_COMPLETE))
                    {
                        //increment ASTar Steps Counter
                        mCurrentAStarStep++;
                        mCurrentStepState = stepState.STATE_ASTAR_INIT;
                    }

                }
                break;
                case STATE_ASTAR_ERROR:
                {
                    //do something on error
                }
                break;
                case STATE_ASTAR_COMPLETE:
                {
                    //empty hashmap ready for next AStar processing.
                    //clear AStar step counter ready for next AStar process
                    mCurrentAStarStep = 0;

                    //when complete, keep processing normal step
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent(TAG, "A* Path Completed:- " + mCurrentStep);
                    }

                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;
                }
                break;
            }

            //check timeout vale
            if ((mStateTime.seconds() > mStepTimeout  ) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED)))
            {
                if (debug >= 1)
                {
                    fileLogger.writeEvent(TAG, "Timeout:- ");
                }
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
            //fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 0)    :- " + mRobotCommand.substring(0, 0));
            //fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 1)    :- " + mRobotCommand.substring(0, 1));
            fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 2)    :- " + mRobotCommand.substring(0, 2));
            //fileLogger.writeEvent("initStep()", "mRobotCommand.substring(0, 3)    :- " + mRobotCommand.substring(0, 3));
            //fileLogger.writeEvent("initStep()", "mRobotCommand.substring(1)       :- " + mRobotCommand.substring(1));
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
            case "LR":  // Left turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "RR":  // Right turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "FW":  // Drive forward a distance in inches and power setting
                mCurrentDriveState = stepState.STATE_INIT;
                break;
            case "AS":  // Plot a course using A* algorithm, accuracy in Parm 1
                mCurrentStepState = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "VL":  // Position the robot using vuforia parameters ready fro AStar  RObot should postion pointing to Red wall and Blue wall where targets are located
                mCurrentVuforiaState = stepState.STATE_INIT;
                break;
            case "VM":  // Move the robot using localisation from the targets
                mCurrentVuforiaMoveState = stepState.STATE_INIT;
                break;
            case "VT":  // Turn the Robot using information from Vuforia and Pythag
                mCurrentVuforiaTurnState = stepState.STATE_INIT;
                break;
            case "BC":  // Get the beacon colour and move the robot to press the button
                mCurrentBeaconColourState = stepState.STATE_INIT;
                break;
            case "AB":  // Press the beacon button robot to press the button
                mCurrentAttackBeaconState = stepState.STATE_INIT;
                break;
            case "SB":  // Press the beacon button robot to press the button
                mCurrentShootBallState = stepState.STATE_INIT;
                break;
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

    private void initAStarStep (LibraryStateSegAuto mStateSegAuto) {
        double mRobotParm1;
        double mRobotParm2;
        double mRobotParm3;
        double mRobotParm4;
        double mRobotParm5;
        double mRobotParm6;
        mStepDistance = 0;

        if (debug >= 3)
        {
            fileLogger.writeEvent(TAG, "Starting to Decode AStar Step ");
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

        mCurrentStepState = stepState.STATE_ASTAR_RUNNING;

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
            case "LR":  // Left turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "RR":  // Right turn with a Radius in Parm 1
                mCurrentRadiusTurnState = stepState.STATE_INIT;
                break;
            case "FW":  // Drive forward a distance in inches and power setting
                mCurrentDriveState = stepState.STATE_INIT;
                break;
            case "AS":  // Plot a course using A* algorithm, accuracy in Parm 1
                mCurrentStepState = stepState.STATE_ASTAR_PRE_INIT;
                break;
            case "FN":  //  Run a special Function with Parms

                break;
        }

        if (debug >= 2) {
            fileLogger.writeEvent("initAStarStep()", "Current Step        :- " + mCurrentStep);
            fileLogger.writeEvent("initAStarStep()", "mStepTimeout        :- " + mStepTimeout);
            fileLogger.writeEvent("initAStarStep()", "mStepSpeed          :- " + mStepSpeed);
            fileLogger.writeEvent("initAStarStep()", "mRobotCommand       :- " + mRobotCommand);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm1         :- " + mRobotParm1);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm2         :- " + mRobotParm2);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm3         :- " + mRobotParm3);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm4         :- " + mRobotParm4);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm5         :- " + mRobotParm5);
            fileLogger.writeEvent("initAStarStep()", "mRobotParm6         :- " + mRobotParm6);
            fileLogger.writeEvent("initAStarStep()", "mStepDistance       :- " + mStepDistance);
            fileLogger.writeEvent("initAStarStep()", "mStepTurnL          :- " + mStepTurnL);
            fileLogger.writeEvent("initAStarStep()", "mStepTurnR          :- " + mStepTurnR);
            fileLogger.writeEvent("initAStarStep()", "mRobotStepComplete  :- " + mRobotStepComplete);
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
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;

        switch (mCurrentDriveState)
        {
            case STATE_INIT:
            {
                mStepDistance = 0;
                blnDisableVisionProcessing = true;  //disable vision processing

                switch (mRobotCommand.substring(0, 2)) {
                    case "FW":  // Drive forward a distance in inches and power setting
                        mStepDistance = Double.parseDouble(mRobotCommand.substring(2));
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

                intLeft1MotorEncoderPosition = robotDrive.leftMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.leftMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.rightMotor1.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.rightMotor2.getCurrentPosition();

                // ramp up speed - need to write function to ramp up speed
                distanceFromStartLeft1 = Math.abs(mStartPositionLeft1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                distanceFromStartLeft2 = Math.abs(mStartPositionLeft2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                distanceFromStartRight1 = Math.abs(mStartPositionRight1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                distanceFromStartRight2 = Math.abs(mStartPositionRight2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;

                //if moving ramp up
                distanceFromStart = (distanceFromStartLeft1 + distanceFromStartRight1 + distanceFromStartLeft2 + distanceFromStartRight2) / 4;

                //determine how close to target we are
                distanceToEndLeft1 = (mStepLeftTarget1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
                distanceToEndLeft2 = (mStepLeftTarget2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
                distanceToEndRight1 = (mStepRightTarget1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
                distanceToEndRight2 = (mStepRightTarget2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;

                //if getting close ramp down speed
                distanceToEnd = (distanceToEndLeft1 + distanceToEndRight1 + distanceToEndLeft2 + distanceToEndRight2) / 4;
/*
                if ((distanceFromStart <= 0.5 ) || (distanceToEnd <= 0.5 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(0.5)));
                }
                else if ((distanceFromStart <= 1 ) || (distanceToEnd <= 1 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(1)));
                }
                else if ((distanceFromStart <= 2 ) || (distanceToEnd <= 2 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(2)));
                }
                else if ((distanceFromStart <= 4 ) || (distanceToEnd <= 4 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(4)));
                }
                else if ((distanceFromStart <= 6 ) || (distanceToEnd <= 6 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(6)));
                }
                else if ((distanceFromStart <= 8 ) || (distanceToEnd <= 8 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(8)));
                }
                else if ((distanceFromStart <= 10 ) || (distanceToEnd <= 10 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(10)));
                }
                else if ((distanceFromStart <= 12 ) || (distanceToEnd <= 12 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(12)));
                }
                else if ((distanceFromStart <= 15 ) || (distanceToEnd <= 15 ))
                {
                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(15)));
                }

                if (mStepSpeed < mStepSpeedTemp)
                    mStepSpeedTemp = mStepSpeed;*/

                // set power on motor controller to start moving
                setDriveMotorPower(Math.abs(mStepSpeedTemp));

                //if within error margin stop
                if (robotDrive.leftMotor1.isBusy() && robotDrive.rightMotor1.isBusy())
                {
                    if (debug >= 3)
                    {
                        fileLogger.writeEvent("runningDriveStep()", "Encoder counts per inch = " + COUNTS_PER_INCH + " distanceFromStart " + distanceFromStart + " distanceToEnd " + distanceToEnd + " Power Level " + mStepSpeedTemp + " Running to target  L1, L2, R1, R2  " + mStepLeftTarget1 + ", " + mStepLeftTarget2 + ", " + mStepRightTarget1 + ",  " + mStepRightTarget2 + ", " + " Running at position L1 " + intLeft1MotorEncoderPosition + " L2 " + intLeft2MotorEncoderPosition + " R1 " + intRight1MotorEncoderPosition + " R2 " + intRight2MotorEncoderPosition);
                    }
                    telemetry.addData("Path1", "Running to %7d :%7d", mStepLeftTarget1, mStepRightTarget1);
                    telemetry.addData("Path2", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intRight1MotorEncoderPosition);
                    telemetry.addData("Path3", "Running at %7d :%7d", intLeft2MotorEncoderPosition, intRight2MotorEncoderPosition);
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
                    blnDisableVisionProcessing = false;  //enable vision processing
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

                blnDisableVisionProcessing = true;  //disable vision processing

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
                        blnDisableVisionProcessing = false;  //enable vision processing
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
                        blnDisableVisionProcessing = false;  //enable vision processing
                        mCurrentPivotTurnState = stepState.STATE_COMPLETE;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    setDriveMotorPower(0);
                    if (debug >= 1)
                    {
                        fileLogger.writeEvent("PivotTurnStep()","Complete         " );
                    }
                    blnDisableVisionProcessing = false;  //enable vision processing
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

                blnDisableVisionProcessing = true;  //disable vision processing

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
                    blnDisableVisionProcessing = false;  //enable vision processing
                    mCurrentTankTurnState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    private void RadiusTurnStep ()
    {
        switch (mCurrentRadiusTurnState) {
            case STATE_INIT: {
                blnDisableVisionProcessing = true;  //disable vision processing


                mCurrentRadiusTurnState = stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING:
            {

                setDriveMotorPower(0);
                if (debug >= 1)
                {
                    fileLogger.writeEvent("RadiusTurnStep()","Complete         " );
                }
                blnDisableVisionProcessing = false;  //enable vision processing

                mCurrentRadiusTurnState = stepState.STATE_COMPLETE;

            }
            break;
        }
    }

    private void VuforiaLocalise ()
    {
        switch (mCurrentVuforiaState) {
            case STATE_INIT: {
                //ensure vision processing is enabled
                blnDisableVisionProcessing = false;  //enable vision processing

                mCurrentVuforiaState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("mCurrentVuforiaState()", "Initialised");
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (debug >= 2) {
                    fileLogger.writeEvent("mCurrentVuforiaState()", "Running" );
                }
                String correctionAngle = "";
                if (debug >= 2) {
                    fileLogger.writeEvent("mCurrentVuforiaState()", "localiseRobotPos " + localiseRobotPos );
                }
                if (!localiseRobotPos) {

                    //need to rerun this step as we cannot get localisation and need to adjust robot to see if we can see a target
                    insertSteps(10, "VL", 0,    0,    0,    0,    0,    0,  0.5, mCurrentStep + 1);
                    if (debug >= 2) {
                        fileLogger.writeEvent("mCurrentVuforiaState()", "Not Localised, inserting a new step" );
                    }
                    //need a delay, as Vuforia is slow to update
                    insertSteps(2, "DL1000", 0, 0, 0, 0, 0, 0, 0, mCurrentStep + 1);

                    //need to adjust robot so we can see target, lets turn robot 180 degrees, if we are facing RED drivers we will end up facing BLUE targets,
                    //if we are facing blue drives we will end up facing RED targets.
                    //if we can't localise we need to abort autonomous so lets try a few things to see if we can localise,
                    // first we will try turning around,
                    // second we will move forward 2 feet
                    // third - abort
                    //Parameter 1 - stop turning once localisation is achieved
                    insertSteps(10, "RT180", 1,    0,    0,    0,    0,    0,  0.5, mCurrentStep + 1);
                    mCurrentVuforiaState = stepState.STATE_COMPLETE;
                    break;
                }

                int intlocalisedRobotBearing = (int)localisedRobotBearing;
                if (debug >= 2) {
                    fileLogger.writeEvent("mCurrentVuforiaState()", "Localised, determining angles.... intlocalisedRobotBearing= " + intlocalisedRobotBearing + " Alliancecolour= " + allianceColor);
                }
                //vuforia angles or 0 towards the BLUE drivers, AStar 0 is to the BLUE beacons
                if (allianceColor.equals("Red")) {
                    //double check localisation
                    if ((intlocalisedRobotBearing > 3) && (intlocalisedRobotBearing < 177))
                    {
                        correctionAngle = "LT" + (180 - intlocalisedRobotBearing);
                    }
                    else if ((intlocalisedRobotBearing > 183) && (intlocalisedRobotBearing < 357))
                    {
                        correctionAngle = "RT" + (180 - (360 - intlocalisedRobotBearing));
                    }
                    else
                    {
                        mCurrentVuforiaState = stepState.STATE_COMPLETE;
                        break;
                    }
                    //double check localisation
                    if (debug >= 2) {
                        fileLogger.writeEvent("mCurrentVuforiaState()", "Inserting Steps VL 0 0 0 mcurrentStep " + mCurrentStep );
                    }
                    insertSteps(10, "VL", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                    //need a delay, as Vuforia is slow to update
                    insertSteps(2, "DL1000", 0, 0, 0, 0, 0, 0, 0, mCurrentStep + 1);
                    //load the angle to adjust
                    insertSteps(10, correctionAngle, 0, 0, 0, 0, 0, 0, 0.3, mCurrentStep + 1);
                    mCurrentVuforiaState = stepState.STATE_COMPLETE;
                    break;
                }

                if (allianceColor.equals("Blue")) {
                    if ((intlocalisedRobotBearing > 273) && (intlocalisedRobotBearing < 360))
                    {
                        correctionAngle = "LT" + (intlocalisedRobotBearing - 270);
                    }
                    else if ((intlocalisedRobotBearing > 0) && (intlocalisedRobotBearing < 91))
                    {
                        correctionAngle = "LT" + (90 + intlocalisedRobotBearing);
                    }
                    else if ((intlocalisedRobotBearing > 90) && (intlocalisedRobotBearing < 267))
                    {
                        correctionAngle = "RT" + (270 - intlocalisedRobotBearing);
                    }
                    else
                    {
                        mCurrentVuforiaState = stepState.STATE_COMPLETE;
                        break;
                    }
                    insertSteps(10, "VL", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                    //need a delay, as Vuforia is slow to update
                    insertSteps(2, "DL1000", 0, 0, 0, 0, 0, 0, 0, mCurrentStep + 1);
                    //load the angle to adjust
                    insertSteps(10, correctionAngle, 0, 0, 0, 0, 0, 0, 0.3, mCurrentStep + 1);
                    mCurrentVuforiaState = stepState.STATE_COMPLETE;
                    break;
                }
            }
            break;
        }
    }

    private void VuforiaMove ()
    {
        switch (mCurrentVuforiaMoveState) {
            case STATE_INIT: {
                //ensure vision processing is enable
                blnDisableVisionProcessing = false;  //enable vision processing

                mCurrentVuforiaMoveState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaMove()", "Initialised");
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaMove()", "Running" );
                }
                String correctionAngle = "";
                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaMove()", "localiseRobotPos " + localiseRobotPos );
                }
                if (!localiseRobotPos)
                {
                    //need to do something gere to try and get localise
                    mCurrentVuforiaMoveState = stepState.STATE_COMPLETE;
                    break;
                }

                int currentX = (int)localisedRobotX;
                int currentY = (int)localisedRobotY;
                int intlocalisedRobotBearing = (int)localisedRobotBearing;
                double requiredMoveX = (currentX - (int)mRobotParm4);
                double requiredMoveY = (currentY - (int)mRobotParm5);

                double requiredMoveDistance = ((Math.sqrt(requiredMoveX * requiredMoveX + requiredMoveY * requiredMoveY)) / 25.4);

                double requiredMoveAngletemp1 = requiredMoveX/requiredMoveY;
                double requiredMoveAngletemp2 = Math.atan(requiredMoveAngletemp1);
                double requiredMoveAngletemp3 = Math.toDegrees(requiredMoveAngletemp2);
                //int requiredMoveAngle = (int)Math.toDegrees(Math.asin(requiredMoveX/requiredMoveY));
                int requiredMoveAngle = (int)Math.abs(requiredMoveAngletemp3);

                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaMove()", "Temp Values requiredMoveAngletemp1 " + requiredMoveAngletemp1 + " requiredMoveAngletemp2 " + requiredMoveAngletemp2 + " requiredMoveAngletemp3 " + requiredMoveAngletemp3);
                    fileLogger.writeEvent("VuforiaMove()", "Temp Values currentX " + currentX + " currentY " + currentY);
                    fileLogger.writeEvent("VuforiaMove()", "Localised, determining angles....Alliancecolour= " + allianceColor + " intlocalisedRobotBearing= " + intlocalisedRobotBearing + " CurrentX= " + currentX + " CurrentY= " + currentY);
                    fileLogger.writeEvent("VuforiaMove()", "Localised, determining angles....requiredMoveX " + requiredMoveX + " requiredMoveY " + requiredMoveY);
                    fileLogger.writeEvent("VuforiaMove()", "Localised, determining angles....requiredMoveDistance " + requiredMoveDistance + " requiredMoveAngle " + requiredMoveAngle);
                }

                if ((((int) mRobotParm5) > currentY) && ((int) mRobotParm4 > currentX)) {
                    requiredMoveAngle = 90 - requiredMoveAngle;
                } else if ((((int) mRobotParm5) > currentY) && ((int) mRobotParm4 < currentX)) {
                    requiredMoveAngle =  90 + requiredMoveAngle;
                } else if ((((int) mRobotParm5) < currentY) && ((int) mRobotParm4 > currentX)) {
                    requiredMoveAngle = 270 + requiredMoveAngle;
                } else if ((((int) mRobotParm5) < currentY) && ((int) mRobotParm4 < currentX)) {
                    requiredMoveAngle = 270 - requiredMoveAngle;
                }

                correctionAngle = newAngleDirection (intlocalisedRobotBearing, requiredMoveAngle);

                insertSteps(10, "FW"+requiredMoveDistance, 0, 0, 0, 0, 0, 0, 0.6, mCurrentStep + 1);
                insertSteps(10, correctionAngle, 0, 0, 0, 0, 0, 0, 0.4, mCurrentStep + 1);

                mCurrentVuforiaMoveState = stepState.STATE_COMPLETE;
            }
            break;
        }
    }

    private void VuforiaTurn ()
    {
        switch (mCurrentVuforiaTurnState) {
            case STATE_INIT: {
                //ensure vision processing is enable
                blnDisableVisionProcessing = false;  //enable vision processing

                mCurrentVuforiaTurnState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaTurn()", "Initialised");
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaTurn()", "Running" );
                }
                String correctionAngle = "";
                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaTurn()", "localiseRobotPos " + localiseRobotPos );
                }
                if (!localiseRobotPos)
                {
                    //need to do something gere to try and get localise
                    mCurrentVuforiaTurnState = stepState.STATE_COMPLETE;
                    break;
                }

                int intlocalisedRobotBearing = (int)localisedRobotBearing;
                double requiredMoveAngle = mRobotParm6;
                correctionAngle = newAngleDirection (intlocalisedRobotBearing, (int)requiredMoveAngle);

                if (debug >= 2) {
                    fileLogger.writeEvent("VuforiaTurn()", "Localised, determining angles....Alliancecolour= " + allianceColor + " intlocalisedRobotBearing= " + intlocalisedRobotBearing  + " requiredMoveAngle " + requiredMoveAngle);
                }

                insertSteps(10, correctionAngle, 0, 0, 0, 0, 0, 0, 0.4, mCurrentStep + 1);

                mCurrentVuforiaTurnState = stepState.STATE_COMPLETE;
            }
            break;
        }
    }

    private void BeaconColour() {

        switch (mCurrentBeaconColourState) {
            case STATE_INIT: {
                //ensure vision processing is enable
                blnDisableVisionProcessing = false;  //enable vision processing
                readyToCapture = true;               //let OpenCV start doing its thing
                intNumberColourTries = 0;
                mCurrentBeaconColourState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("BeaconColour()", "Initialised");
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (debug >= 2) {
                    fileLogger.writeEvent("BeaconColour()", "Running" );
                }
                intNumberColourTries ++;
                //mColour = beaconColour.beaconAnalysisOCV(tmp, captureLoop);

                if (debug >= 2) {
                    fileLogger.writeEvent("BeaconColour()", "Returned " + mColour + " intNumberColourTries" + intNumberColourTries);
                }

                if (intNumberColourTries > 1) {
                    readyToCapture = false;
                    mCurrentBeaconColourState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }

    }

    private void AttackBeacon() {

        switch (mCurrentAttackBeaconState) {
            case STATE_INIT: {
                //ensure vision processing is enable
                blnDisableVisionProcessing = false;  //enable vision processing
                readyToCapture = false;               //stop OpenCV start doing its thing
                intNumberColourTries = 0;
                mCurrentAttackBeaconState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("AttackBeacon()", "Initialised");
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (debug >= 2) {
                    fileLogger.writeEvent("AttackBeacon()", "Running" );
                }
                int intlocalisedRobotBearing = (int)localisedRobotBearing;  //get the current angle of the robot.

                //the button is a few degrees to the left or right.

                if (allianceColor.equals("Red")) {
                    if (mColour == Constants.BeaconColours.BEACON_BLUE_RED) {    //means red is to the right
                        insertSteps(10, "FW-24", 0, 0, 0, 0, 0, 0, 0.6, mCurrentStep + 1);
                        insertSteps(10, "FW10", 0, 0, 0, 0, 0, 0, 0.35, mCurrentStep + 1);
                        insertSteps(10, "LT30", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "FW1", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "RT30", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                    } else if (mColour == Constants.BeaconColours.BEACON_RED_BLUE) {
                        insertSteps(10, "FW-24", 0, 0, 0, 0, 0, 0, 0.6, mCurrentStep + 1);
                        insertSteps(10, "FW8", 0, 0, 0, 0, 0, 0, 0.35, mCurrentStep + 1);
                        insertSteps(10, "RT45", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "FW9", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "LT45", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                    }
                } else if (allianceColor.equals("Blue")) {
                    if (mColour == Constants.BeaconColours.BEACON_BLUE_RED) {    //means red is to the right
                        insertSteps(10, "FW-24", 0, 0, 0, 0, 0, 0, 0.6, mCurrentStep + 1);
                        insertSteps(10, "FW8", 0, 0, 0, 0, 0, 0, 0.35, mCurrentStep + 1);
                        insertSteps(10, "RT45", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "FW9", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "LT45", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                    } else if (mColour == Constants.BeaconColours.BEACON_RED_BLUE) {
                        insertSteps(10, "FW-24", 0, 0, 0, 0, 0, 0, 0.6, mCurrentStep + 1);
                        insertSteps(10, "FW10", 0, 0, 0, 0, 0, 0, 0.35, mCurrentStep + 1);
                        insertSteps(10, "LT30", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "FW6", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                        insertSteps(10, "RT30", 0, 0, 0, 0, 0, 0, 0.5, mCurrentStep + 1);
                    }
                }

                if (debug >= 2) {
                    fileLogger.writeEvent("AttackBeacon()", "Returned " + mColour);
                }

                mCurrentAttackBeaconState = stepState.STATE_COMPLETE;
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
                if (debug >= 2) {
                    fileLogger.writeEvent("DelayStep()", "Init Delay Time    " + mStepDelay);
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= mStepDelay) {
                    if (debug >= 1) {
                        fileLogger.writeEvent("DelayStep()", "Complete         ");
                    }
                    mCurrentDelayState = stepState.STATE_COMPLETE;
                }
            }
            break;
        }
    }

    private void ShootBallStep ()
    {
        switch (mCurrentShootBallState) {
            case STATE_INIT: {
                mCurrentShootBallState = stepState.STATE_RUNNING;
                if (debug >= 2) {
                    fileLogger.writeEvent("ShootBallStep()", "Init");
                }
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= (int)mRobotParm1) {
                    //stop shooting we are complete
                    armDrive.sweeper.setPower(0);
                    armDrive.flicker.setPower(0);
                    if (debug >= 1) {
                        fileLogger.writeEvent("ShootBallStep()", "Complete         ");
                    }
                    mCurrentShootBallState = stepState.STATE_COMPLETE;
                } else {
                    //start shooting
                    armDrive.sweeper.setPower(1);
                    armDrive.flicker.setPower(1);

                    if (debug >= 3) {
                        fileLogger.writeEvent("ShootBallStep()", "Running        ");
                    }

                }

            }
            break;
        }
    }


    private String getAngle(int angle1, int angle2)
    {
        if (debug >= 2)
        {
            fileLogger.writeEvent(TAG, "Getangle - Current Angle1:= " + angle1 + " Desired Angle2:= " + angle2);
        }

        switch (angle1)
        {
            case 0:
                switch (angle2)
                {
                    case 45:
                        return "RT45";
                    case 90:
                        return "RT90";
                    case 135:
                        return "RT135";
                    case 180:
                        return "RT180";
                    case 225:
                        return "LT135";
                    case 270:
                        return "LT90";
                    case 315:
                        return "LT45";
                }
                break;
            case 45:
                switch (angle2)
                {
                    case 0:
                        return "LT45";
                    case 90:
                        return "RT45";
                    case 135:
                        return "RT90";
                    case 180:
                        return "RT135";
                    case 225:
                        return "RT180";
                    case 270:
                        return "LT135";
                    case 315:
                        return "LT90";
                }
                break;
            case 90:
                switch (angle2)
                {
                    case 0:
                        return "LT90";
                    case 45:
                        return "LT45";
                    case 135:
                        return "RT45";
                    case 180:
                        return "RT90";
                    case 225:
                        return "RT135";
                    case 270:
                        return "RT180";
                    case 315:
                        return "LT135";
                }
                break;
            case 135:
                switch (angle2)
                {
                    case 0:
                        return "LT135";
                    case 45:
                        return "LT90";
                    case 90:
                        return "LT45";
                    case 180:
                        return "RT45";
                    case 225:
                        return "RT90";
                    case 270:
                        return "RT135";
                    case 315:
                        return "RT180";
                }
                break;
            case 180:
                switch (angle2)
                {
                    case 0:
                        return "LT180";
                    case 45:
                        return "LT135";
                    case 90:
                        return "LT90";
                    case 135:
                        return "LT45";
                    case 225:
                        return "RT45";
                    case 270:
                        return "RT90";
                    case 315:
                        return "RT135";
                }
                break;
            case 225:
                switch (angle2)
                {
                    case 0:
                        return "RT135";
                    case 45:
                        return "LT180";
                    case 90:
                        return "LT135";
                    case 135:
                        return "LT90";
                    case 180:
                        return "LT45";
                    case 270:
                        return "RT45";
                    case 315:
                        return "RT90";
                }
                break;
            case 270:
                switch (angle2)
                {
                    case 0:
                        return "RT90";
                    case 45:
                        return "RT135";
                    case 90:
                        return "LT180";
                    case 135:
                        return "LT135";
                    case 180:
                        return "LT90";
                    case 225:
                        return "LT45";
                    case 315:
                        return "RT45";
                }
                break;
            case 315:
                switch (angle2)
                {
                    case 0:
                        return "RT45";
                    case 45:
                        return "RT90";
                    case 90:
                        return "RT135";
                    case 135:
                        return "LT180";
                    case 180:
                        return "LT135";
                    case 225:
                        return "LT90";
                    case 270:
                        return "LT45";
                }
                break;
        }
        return "ERROR";
    }

    String newAngleDirection (int currentDirection, int newDirection)
    {
        if (currentDirection < newDirection)
            return "LT" + (newDirection - currentDirection);
        else
            return "RT" + (currentDirection - newDirection);
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
