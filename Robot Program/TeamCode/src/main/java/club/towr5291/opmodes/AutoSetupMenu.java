package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import club.towr5291.functions.FileLogger;

import club.towr5291.libraries.LibField;
import club.towr5291.opmodes.OpModeMaster;
import club.towr5291.opmodes.R;
import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

/**
 * Created by Ian Haden on 11/7/2016.
 */

@Autonomous(name = "Auton Config", group = "0")
public class AutoSetupMenu extends OpModeMaster implements FtcMenu.MenuButtons {

    //set up the variables for the logger
    final String TAG = "Auton Menu";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 3;

    private int loop = 0;

    private static HalDashboard dashboard = null;

    public static HalDashboard getDashboard()
    {
        return dashboard;
    }

    //The autonomous menu settings using sharedpreferences
    private SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private String allianceParkPosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;

    private static AutoSetupMenu instance = null;

    public AutoSetupMenu()
    {
        super();
        instance = this;
    }


    public void initRobot()
    {

        dashboard = HalDashboard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");
        setup();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //start the log
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Log Started");

        initRobot();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            //this will log only when debug is at level 3 or above
            if (debug >= 3) {
                fileLogger.writeEvent(TAG, "In Loop # " + loop);
            }
            loop++;
        }

        //stop the log
        if (fileLogger != null) {
            fileLogger.writeEvent(TAG, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }


    private void setup()
    {
        dashboard.displayPrintf(0, "INITIALIZING - Please wait for Menu");
        fileLogger.writeEvent(TAG, "SETUP");

        doMenus();
        dashboard.displayPrintf(0, "COMPLETE - Settings Written");


    }

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up;} //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    } //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    } //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }  //isMenuBackButton

    private void doMenus()
    {

        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        editor = sharedPreferences.edit();
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        allianceParkPosition = sharedPreferences.getString("club.towr5291.Autonomous.ParkPosition", "Centre");
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "Both");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //
        // Create the menus.
        //
        FtcChoiceMenu teamMenu      = new FtcChoiceMenu("Team:", null, this);
        FtcChoiceMenu allianceMenu  = new FtcChoiceMenu("Alliance:", teamMenu, this);
        FtcChoiceMenu startPosMenu  = new FtcChoiceMenu("Start:", allianceMenu, this);
        FtcChoiceMenu parkMenu      = new FtcChoiceMenu("Park:", startPosMenu, this);
        FtcChoiceMenu beaconMenu    = new FtcChoiceMenu("Beacons:", parkMenu, this);
        FtcValueMenu delayMenu      = new FtcValueMenu("Delay:", beaconMenu, this, 0.0, 20.0, 1.0, 0.0, "%5.2f");
        FtcChoiceMenu robotConfigMenu    = new FtcChoiceMenu("Robot:", delayMenu, this);
        FtcChoiceMenu debugConfigMenu    = new FtcChoiceMenu("Debug:", robotConfigMenu, this);

        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //
        if (teamNumber.equals(LibField.Team.TOWR.toString())) {
            teamMenu.addChoice(LibField.Team.TOWR.toString(), LibField.Team.TOWR, allianceMenu);
            teamMenu.addChoice(LibField.Team.CYBORGCATZ.toString(), LibField.Team.CYBORGCATZ, allianceMenu);
            teamMenu.addChoice(LibField.Team.ELECTCATZ.toString(), LibField.Team.ELECTCATZ, allianceMenu);
        } else if (teamNumber.equals(LibField.Team.CYBORGCATZ.toString())) {
            teamMenu.addChoice(LibField.Team.CYBORGCATZ.toString(), LibField.Team.CYBORGCATZ, allianceMenu);
            teamMenu.addChoice(LibField.Team.TOWR.toString(), LibField.Team.TOWR, allianceMenu);
            teamMenu.addChoice(LibField.Team.ELECTCATZ.toString(), LibField.Team.ELECTCATZ, allianceMenu);
        } else {
            teamMenu.addChoice(LibField.Team.ELECTCATZ.toString(), LibField.Team.ELECTCATZ, allianceMenu);
            teamMenu.addChoice(LibField.Team.CYBORGCATZ.toString(), LibField.Team.CYBORGCATZ, allianceMenu);
            teamMenu.addChoice(LibField.Team.TOWR.toString(), LibField.Team.TOWR, allianceMenu);
        }

        if (allianceColor.equals(LibField.Alliance.RED.toString())) {
            allianceMenu.addChoice(LibField.Alliance.RED.toString(),  LibField.Alliance.RED, startPosMenu);
            allianceMenu.addChoice(LibField.Alliance.BLUE.toString(), LibField.Alliance.BLUE, startPosMenu);
        } else  {
            allianceMenu.addChoice(LibField.Alliance.BLUE.toString(), LibField.Alliance.BLUE, startPosMenu);
            allianceMenu.addChoice(LibField.Alliance.RED.toString(),  LibField.Alliance.RED, startPosMenu);
        }

        if (allianceStartPosition.equals(LibField.StartPos.START_LEFT.toString())) {
            startPosMenu.addChoice(LibField.StartPos.START_LEFT.toString(), LibField.StartPos.START_LEFT, beaconMenu);
            startPosMenu.addChoice(LibField.StartPos.START_RIGHT.toString(), LibField.StartPos.START_RIGHT, beaconMenu);
            startPosMenu.addChoice(LibField.StartPos.START_TEST.toString(), LibField.StartPos.START_TEST, beaconMenu);
        } else if (allianceStartPosition.equals(LibField.StartPos.START_RIGHT.toString())) {
            startPosMenu.addChoice(LibField.StartPos.START_RIGHT.toString(), LibField.StartPos.START_RIGHT, beaconMenu);
            startPosMenu.addChoice(LibField.StartPos.START_LEFT.toString(), LibField.StartPos.START_LEFT, beaconMenu);
            startPosMenu.addChoice(LibField.StartPos.START_TEST.toString(), LibField.StartPos.START_TEST, beaconMenu);
        } else {
            startPosMenu.addChoice(LibField.StartPos.START_TEST.toString(), LibField.StartPos.START_TEST, beaconMenu);
            startPosMenu.addChoice(LibField.StartPos.START_LEFT.toString(), LibField.StartPos.START_LEFT, beaconMenu);
            startPosMenu.addChoice(LibField.StartPos.START_RIGHT.toString(), LibField.StartPos.START_RIGHT, beaconMenu);
        }

        if (numBeacons.equals(LibField.BeaconChoice.BOTH.toString())) {
            beaconMenu.addChoice(LibField.BeaconChoice.BOTH.toString(), LibField.BeaconChoice.BOTH, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.NEAR.toString(), LibField.BeaconChoice.NEAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.FAR.toString(), LibField.BeaconChoice.FAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.NONE.toString(), LibField.BeaconChoice.NONE, parkMenu);
        } else if (numBeacons.equals(LibField.BeaconChoice.NEAR.toString())) {
            beaconMenu.addChoice(LibField.BeaconChoice.NEAR.toString(), LibField.BeaconChoice.NEAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.BOTH.toString(), LibField.BeaconChoice.BOTH, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.FAR.toString(), LibField.BeaconChoice.FAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.NONE.toString(), LibField.BeaconChoice.NONE, parkMenu);
        } else if (numBeacons.equals(LibField.BeaconChoice.FAR.toString())) {
            beaconMenu.addChoice(LibField.BeaconChoice.FAR.toString(), LibField.BeaconChoice.FAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.NEAR.toString(), LibField.BeaconChoice.NEAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.BOTH.toString(), LibField.BeaconChoice.BOTH, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.NONE.toString(), LibField.BeaconChoice.NONE, parkMenu);
        } else {
            beaconMenu.addChoice(LibField.BeaconChoice.NONE.toString(), LibField.BeaconChoice.NONE, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.BOTH.toString(), LibField.BeaconChoice.BOTH, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.NEAR.toString(), LibField.BeaconChoice.NEAR, parkMenu);
            beaconMenu.addChoice(LibField.BeaconChoice.FAR.toString(), LibField.BeaconChoice.FAR, parkMenu);
        }

        if (allianceParkPosition.equals(LibField.ParkChoice.CENTER_PARK.toString())) {
            parkMenu.addChoice(LibField.ParkChoice.CENTER_PARK.toString(), LibField.ParkChoice.CENTER_PARK, delayMenu);
            parkMenu.addChoice(LibField.ParkChoice.CORNER_PARK.toString(), LibField.ParkChoice.CORNER_PARK, delayMenu);
            parkMenu.addChoice(LibField.ParkChoice.DEFEND_PARK.toString(), LibField.ParkChoice.DEFEND_PARK, delayMenu);
        } else if (allianceParkPosition.equals(LibField.ParkChoice.CORNER_PARK.toString())) {
            parkMenu.addChoice(LibField.ParkChoice.CORNER_PARK.toString(), LibField.ParkChoice.CORNER_PARK, delayMenu);
            parkMenu.addChoice(LibField.ParkChoice.CENTER_PARK.toString(), LibField.ParkChoice.CENTER_PARK, delayMenu);
            parkMenu.addChoice(LibField.ParkChoice.DEFEND_PARK.toString(), LibField.ParkChoice.DEFEND_PARK, delayMenu);
        } else {
            parkMenu.addChoice(LibField.ParkChoice.DEFEND_PARK.toString(), LibField.ParkChoice.DEFEND_PARK, delayMenu);
            parkMenu.addChoice(LibField.ParkChoice.CENTER_PARK.toString(), LibField.ParkChoice.CENTER_PARK, delayMenu);
            parkMenu.addChoice(LibField.ParkChoice.CORNER_PARK.toString(), LibField.ParkChoice.CORNER_PARK, delayMenu);
        }

        delayMenu.setChildMenu(robotConfigMenu);

        if (robotConfig.equals(LibField.RobotConfigChoice.TileRunner2x60.toString())) {
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x60.toString(), LibField.RobotConfigChoice.TileRunner2x60, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x40.toString(), LibField.RobotConfigChoice.TileRunner2x40, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x20.toString(), LibField.RobotConfigChoice.TileRunner2x20, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.Custom_11231_2016.toString(), LibField.RobotConfigChoice.Custom_11231_2016, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TankTread2x40Custom.toString(), LibField.RobotConfigChoice.TankTread2x40Custom, debugConfigMenu);
        } else if (robotConfig.equals(LibField.RobotConfigChoice.TileRunner2x40.toString())) {
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x40.toString(), LibField.RobotConfigChoice.TileRunner2x40, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x60.toString(), LibField.RobotConfigChoice.TileRunner2x60, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x20.toString(), LibField.RobotConfigChoice.TileRunner2x20, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.Custom_11231_2016.toString(), LibField.RobotConfigChoice.Custom_11231_2016, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TankTread2x40Custom.toString(), LibField.RobotConfigChoice.TankTread2x40Custom, debugConfigMenu);
        } else if (robotConfig.equals(LibField.RobotConfigChoice.TileRunner2x20.toString())) {
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x20.toString(), LibField.RobotConfigChoice.TileRunner2x20, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x40.toString(), LibField.RobotConfigChoice.TileRunner2x40, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x60.toString(), LibField.RobotConfigChoice.TileRunner2x60, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.Custom_11231_2016.toString(), LibField.RobotConfigChoice.Custom_11231_2016, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TankTread2x40Custom.toString(), LibField.RobotConfigChoice.TankTread2x40Custom, debugConfigMenu);
        } else if (robotConfig.equals(LibField.RobotConfigChoice.TankTread2x40Custom.toString())) {
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TankTread2x40Custom.toString(), LibField.RobotConfigChoice.TankTread2x40Custom, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x60.toString(), LibField.RobotConfigChoice.TileRunner2x60, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x40.toString(), LibField.RobotConfigChoice.TileRunner2x40, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x20.toString(), LibField.RobotConfigChoice.TileRunner2x20, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.Custom_11231_2016.toString(), LibField.RobotConfigChoice.Custom_11231_2016, debugConfigMenu);
        }else if (robotConfig.equals(LibField.RobotConfigChoice.Custom_11231_2016.toString())) {
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.Custom_11231_2016.toString(), LibField.RobotConfigChoice.Custom_11231_2016, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TankTread2x40Custom.toString(), LibField.RobotConfigChoice.TankTread2x40Custom, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x60.toString(), LibField.RobotConfigChoice.TileRunner2x60, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x40.toString(), LibField.RobotConfigChoice.TileRunner2x40, debugConfigMenu);
            robotConfigMenu.addChoice(LibField.RobotConfigChoice.TileRunner2x20.toString(), LibField.RobotConfigChoice.TileRunner2x20, debugConfigMenu);
        }

        if (debug == 1) {
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 2) {
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 3) {
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 4) {
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 5) {
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 6) {
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        }  else if (debug == 7) {
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 8) {
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("10", 10);
        } else if (debug == 9) {
            debugConfigMenu.addChoice("9", 9);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("10", 10);
        }  else {
            debugConfigMenu.addChoice("10", 10);
            debugConfigMenu.addChoice("1", 1);
            debugConfigMenu.addChoice("2", 2);
            debugConfigMenu.addChoice("3", 3);
            debugConfigMenu.addChoice("4", 4);
            debugConfigMenu.addChoice("5", 5);
            debugConfigMenu.addChoice("6", 6);
            debugConfigMenu.addChoice("7", 7);
            debugConfigMenu.addChoice("8", 8);
            debugConfigMenu.addChoice("9", 9);
        }

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(teamMenu, this);

        //
        // Set choices variables.
        //
        allianceStartPosition = startPosMenu.getChoiceText(startPosMenu.getCurrentChoice());
        numBeacons = beaconMenu.getChoiceText(beaconMenu.getCurrentChoice());
        allianceParkPosition = parkMenu.getChoiceText(parkMenu.getCurrentChoice());
        allianceColor = allianceMenu.getChoiceText(allianceMenu.getCurrentChoice());
        teamNumber = teamMenu.getChoiceText(teamMenu.getCurrentChoice());
        robotConfig = robotConfigMenu.getChoiceText(robotConfigMenu.getCurrentChoice());
        delay = (int)delayMenu.getCurrentValue();
        debug = Integer.parseInt(debugConfigMenu.getChoiceText(debugConfigMenu.getCurrentChoice()));

        //write the options to sharedpreferences
        editor.putString("club.towr5291.Autonomous.TeamNumber", teamNumber);
        editor.putString("club.towr5291.Autonomous.Color", allianceColor);
        editor.putString("club.towr5291.Autonomous.StartPosition", allianceStartPosition);
        editor.putString("club.towr5291.Autonomous.ParkPosition", allianceParkPosition);
        editor.putString("club.towr5291.Autonomous.Beacons", numBeacons);
        editor.putString("club.towr5291.Autonomous.Delay", String.valueOf(delay));
        editor.putString("club.towr5291.Autonomous.RobotConfig", robotConfig);
        editor.putString("club.towr5291.Autonomous.Debug", String.valueOf(debug));
        editor.commit();

        //read them back to ensure they were written
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", null);
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", null);
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", null);
        allianceParkPosition = sharedPreferences.getString("club.towr5291.Autonomous.ParkPosition", null);
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", null);
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", null));
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", null);
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", null));

        int lnum = 1;
        dashboard.displayPrintf(lnum++, "Team:     " + teamNumber);
        dashboard.displayPrintf(lnum++, "Alliance: " + allianceColor);
        dashboard.displayPrintf(lnum++, "Start:    " + allianceStartPosition);
        dashboard.displayPrintf(lnum++, "Park:     " + allianceParkPosition);
        dashboard.displayPrintf(lnum++, "Beacons:  " + numBeacons);
        dashboard.displayPrintf(lnum++, "Delay:    " + String.valueOf(delay));
        dashboard.displayPrintf(lnum++, "Robot:    " + robotConfig);
        dashboard.displayPrintf(lnum++, "Debug:    " + debug);

        fileLogger.writeEvent("AutonConfig", "Team     " + teamNumber);
        fileLogger.writeEvent("AutonConfig", "Alliance " + allianceColor);
        fileLogger.writeEvent("AutonConfig", "Start    " + allianceStartPosition);
        fileLogger.writeEvent("AutonConfig", "Park     " + allianceParkPosition);
        fileLogger.writeEvent("AutonConfig", "Beacons  " + numBeacons);
        fileLogger.writeEvent("AutonConfig", "Delay    " + String.valueOf(delay));
        fileLogger.writeEvent("AutonConfig", "Robot    " + robotConfig);
        fileLogger.writeEvent("AutonConfig", "Debug:   " + debug);

    }

}


