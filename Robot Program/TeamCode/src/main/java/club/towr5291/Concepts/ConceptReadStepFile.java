package club.towr5291.Concepts;

import android.content.SharedPreferences;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

import club.towr5291.functions.FileLogger;
import club.towr5291.libraries.LibraryStateSegAuto;
import club.towr5291.functions.ReadStepFile;


/**
 * Created by ianhaden on 19/01/2017.
 */

@TeleOp(name = "Concept Read Steps", group = "5291Concept")

public class ConceptReadStepFile extends LinearOpMode {

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private String allianceParkPosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;
    private int debug;

    //set up the variables for the logger
    final String TAG = "Concept Read File";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;


    private ReadStepFile autonomousStepsTest = new ReadStepFile();

    private HashMap<String,LibraryStateSegAuto> autonomousStepsMap = new HashMap<String,LibraryStateSegAuto>();

    private void loadSharePreferences()
    {
        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        allianceParkPosition = sharedPreferences.getString("club.towr5291.Autonomous.ParkPosition", "Vortex");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));
        debug = 3;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        loadSharePreferences();

        LibraryStateSegAuto mStateSegAuto;

        if (debug >= 1) {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        allianceColor = "Red";
        allianceStartPosition = "Left";
        allianceParkPosition = "Centre";
        numBeacons = "both";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291RedLeft.csv");
            Log.d("Status", "Reading Steps : 5291RedLeft.csv");

        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291RedLeft.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Red";
        allianceStartPosition = "Left";
        allianceParkPosition = "Corner";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291RedLeft.csv");
            Log.d("Status", "Reading Steps : 5291RedLeft.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291RedLeft.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Red";
        allianceStartPosition = "Right";
        allianceParkPosition = "Centre";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291RedRight.csv");
            Log.d("Status", "Reading Steps : 5291RedRight.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291RedRight.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Red";
        allianceStartPosition = "Right";
        allianceParkPosition = "Corner";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291RedRight.csv");
            Log.d("Status", "Reading Steps : 5291RedRight.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291RedRight.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Blue";
        allianceStartPosition = "Left";
        allianceParkPosition = "Centre";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291BlueLeft.csv");
            Log.d("Status", "Reading Steps : 5291BlueLeft.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291BlueLeft.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Blue";
        allianceStartPosition = "Left";
        allianceParkPosition = "Corner";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291BlueLeft.csv");
            Log.d("Status", "Reading Steps : 5291BlueLeft.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291BlueLeft.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Blue";
        allianceStartPosition = "Right";
        allianceParkPosition = "Centre";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291BlueRight.csv");
            Log.d("Status", "Reading Steps : 5291BlueRight.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291BlueRight.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        allianceColor = "Blue";
        allianceStartPosition = "Right";
        allianceParkPosition = "Corner";
        if (debug >= 1) {
            fileLogger.writeEvent("Status", "Reading Steps : 5291BlueRight.csv");
            Log.d("Status", "Reading Steps : 5291BlueRight.csv");
        }
        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291BlueRight.csv" , allianceParkPosition, numBeacons);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());

        autonomousStepsMap = autonomousStepsTest.ReadStepFile("5291BlueLeft.csv" , allianceParkPosition, "both");

        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        autonomousStepsTest.insertSteps(3, "DEL1000", false, false, 0,    0,    0,    0,    0,    0,  0, 1);

        autonomousStepsTest.insertSteps(2, "IAN1", false, false, 0, 0, 0, 0, 0, 0, 0, 6);

        autonomousStepsTest.insertSteps(2, "IAN2", false, false, 0, 0, 0, 0, 0, 0, 0, 8);

        autonomousStepsTest.insertSteps(2, "IAN3", false, false, 0, 0, 0, 0, 0, 0, 0, 10);

        for (int loop = 1; loop < autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            idle();
        }

        if (debug >= 1) {
            if (fileLogger != null) {
                fileLogger.writeEvent("Status", "Run Time: " + runtime.toString());
                Log.d("Status", "Run Time: " + runtime.toString());
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }
    }
}
