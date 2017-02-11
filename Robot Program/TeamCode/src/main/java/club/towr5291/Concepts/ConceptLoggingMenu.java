package club.towr5291.Concepts;

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "Concept Logging Menu", group = "5291 Concepts")
public class ConceptLoggingMenu extends OpModeMaster implements FtcMenu.MenuButtons {

    //set up the variables for the logger
    final String TAG = "Concept Logging";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    private int debug = 1;

    private int loop = 0;

    private static HalDashboard dashboard = null;

    private static LibField.StartPos startPos = LibField.StartPos.START_LEFT;
    private static LibField.BeaconChoice beaconChoice = LibField.BeaconChoice.NEAR;
    private static LibField.ParkChoice parkChoice = LibField.ParkChoice.CENTER_PARK;
    private static LibField.Alliance alliance = LibField.Alliance.RED;
    private static LibField.Team team = LibField.Team.TOWR;

    public static HalDashboard getDashboard()
    {
        return dashboard;
    }

    public void initRobot()
    {

        dashboard = HalDashboard.createInstance(telemetry);

        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        setup();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //start the log
        if (debug >= 1) {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
        }
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
        if (debug >= 1) {
            if (fileLogger != null) {
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }
    }


    private void setup()
    {
        dashboard.displayPrintf(2, "STATE: %s", "INITIALIZING - PLEASE WAIT FOR MENU");
        fileLogger.writeEvent(TAG, "SETUP");

        doMenus();



        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");



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
        //
        // Create the menus.
        //

        FtcChoiceMenu<LibField.Team> teamMenu               =
                new FtcChoiceMenu<>("TEAM:", null, this);

        FtcChoiceMenu<LibField.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", teamMenu, this);

        FtcChoiceMenu<LibField.StartPos> startPosMenu =
                new FtcChoiceMenu<>("START:", allianceMenu, this);

        FtcChoiceMenu<LibField.BeaconChoice> beaconMenu =
                new FtcChoiceMenu<>("BEACONS:", startPosMenu, this);

        FtcChoiceMenu<LibField.ParkChoice> parkMenu   =
                new FtcChoiceMenu<>("PARK:", beaconMenu, this);

        FtcValueMenu delayMenu     = new FtcValueMenu("DELAY:", parkMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");


        teamMenu.addChoice(LibField.Team.TOWR.toString(), LibField.Team.TOWR, allianceMenu);
        teamMenu.addChoice(LibField.Team.CYBORGCATZ.toString(), LibField.Team.CYBORGCATZ, allianceMenu);

        allianceMenu.addChoice("RED",  LibField.Alliance.RED, startPosMenu);
        allianceMenu.addChoice("BLUE", LibField.Alliance.BLUE, startPosMenu);

        startPosMenu.addChoice(LibField.StartPos.START_LEFT.toString(), LibField.StartPos.START_LEFT, beaconMenu);
        startPosMenu.addChoice(LibField.StartPos.START_RIGHT.toString(), LibField.StartPos.START_RIGHT, beaconMenu);
        startPosMenu.addChoice(LibField.StartPos.START_TEST.toString(), LibField.StartPos.START_TEST, beaconMenu);

        beaconMenu.addChoice(LibField.BeaconChoice.BOTH.toString(), LibField.BeaconChoice.BOTH, parkMenu);
        beaconMenu.addChoice(LibField.BeaconChoice.NEAR.toString(), LibField.BeaconChoice.NEAR, parkMenu);
        beaconMenu.addChoice(LibField.BeaconChoice.FAR.toString(), LibField.BeaconChoice.FAR, parkMenu);
        beaconMenu.addChoice(LibField.BeaconChoice.NONE.toString(), LibField.BeaconChoice.NONE, parkMenu);

        parkMenu.addChoice(LibField.ParkChoice.CENTER_PARK.toString(), LibField.ParkChoice.CENTER_PARK, delayMenu);
        parkMenu.addChoice(LibField.ParkChoice.CORNER_PARK.toString(), LibField.ParkChoice.CORNER_PARK, delayMenu);
        parkMenu.addChoice(LibField.ParkChoice.DEFEND_PARK.toString(), LibField.ParkChoice.DEFEND_PARK, delayMenu);

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(teamMenu, this);

        //
        // Set choices variables.
        //

        startPos = startPosMenu.getCurrentChoiceObject();
        beaconChoice = beaconMenu.getCurrentChoiceObject();
        parkChoice = parkMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        team = teamMenu.getCurrentChoiceObject();

        int lnum = 3;
        dashboard.displayPrintf(lnum++, "START: %s", startPos);
        dashboard.displayPrintf(lnum++, "PUSH: %s", beaconChoice);
        dashboard.displayPrintf(lnum++, "PARK: %s", parkChoice);
        dashboard.displayPrintf(lnum++, "ALLIANCE: %s", alliance);
        dashboard.displayPrintf(lnum++, "TEAM: %s", team);

        fileLogger.writeEvent("STARTPOS %s", "" + startPos);
        fileLogger.writeEvent("PUSH     %s", "" + beaconChoice);
        fileLogger.writeEvent("PARK     %s", "" + parkChoice);
        fileLogger.writeEvent("ALLIANCE %s", "" + alliance);
        fileLogger.writeEvent("TEAM     %s", "" + team);
    }

}
