package org.firstinspires.ftc.robotcontroller.internal;

/**
 * Created by ianhaden on 30/10/2016.
 */


import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.NumberPicker;
import android.widget.Spinner;
import android.widget.Toast;

import com.qualcomm.ftcrobotcontroller.R;


public class AutonomousConfiguration extends Activity implements Spinner.OnItemSelectedListener {
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    Spinner teamNumber;
    Spinner allianceColor;
    Spinner allianceStartPosition;
    Spinner allianceBeacons;
    Spinner allianceParkPosition;
    Spinner robotConfig;
    Spinner delay;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.autonomous_configuration);

        sharedPreferences =  PreferenceManager.getDefaultSharedPreferences(this);
        editor = sharedPreferences.edit();

        //String savedColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
        //String savedPosition = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Position", "null");
        //int savedDelay = sharedPreferences.getInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", 0);

        String savedTeamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber","0000");
        String savedColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Test");
        String savedStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Test");
        String savedParkPosition = sharedPreferences.getString("club.towr5291.Autonomous.ParkPosition", "Test");
        String savedBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "Zero");
        String savedConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "Test");
        String savedDelay = sharedPreferences.getString("club.towr5291.Autonomous.Delay", "00");

        teamNumber = (Spinner) findViewById(R.id.team_number_options);
        teamNumber.setOnItemSelectedListener(this);
        //teamNumber.setSelection(savedTeamNumber.equals("5291") ? 0 : 1, true);
        teamNumber.setSelection(((ArrayAdapter)teamNumber.getAdapter()).getPosition(savedTeamNumber));

        allianceColor = (Spinner) findViewById(R.id.alliance_color_options);
        allianceColor.setOnItemSelectedListener(this);
        //allianceColor.setSelection(savedColor.equals("Red") ? 0 : 1, true);
        allianceColor.setSelection(((ArrayAdapter)allianceColor.getAdapter()).getPosition(savedColor));

        allianceStartPosition = (Spinner) findViewById(R.id.spinnerStartPosition);
        allianceStartPosition.setOnItemSelectedListener(this);
        //allianceStartPosition.setSelection(savedPosition.equals("Left") ? 0 : 1, true);
        allianceStartPosition.setSelection(((ArrayAdapter)allianceStartPosition.getAdapter()).getPosition(savedStartPosition));

        allianceParkPosition = (Spinner) findViewById(R.id.spinnerParkPosition);
        allianceParkPosition.setOnItemSelectedListener(this);
        allianceParkPosition.setSelection(((ArrayAdapter)allianceParkPosition.getAdapter()).getPosition(savedParkPosition));

        allianceBeacons = (Spinner) findViewById(R.id.spinnerBeacons);
        allianceBeacons.setOnItemSelectedListener(this);
        //allianceBeacons.setSelection(savedBeacons.equals("One") ? 0 : 1, true);
        allianceBeacons.setSelection(((ArrayAdapter)allianceBeacons.getAdapter()).getPosition(savedBeacons));

        robotConfig = (Spinner) findViewById(R.id.spinnerRobotConfig);
        robotConfig.setOnItemSelectedListener(this);
        //robotConfig.setSelection(savedConfig.equals("TileRunner-2x40") ? 0 : 1, true);
        robotConfig.setSelection(((ArrayAdapter)robotConfig.getAdapter()).getPosition(savedConfig));

        delay = (Spinner) findViewById(R.id.spinnerAllianceDelay);
        delay.setOnItemSelectedListener(this);
        delay.setSelection(savedDelay.equals("00") ? 0 : 1, true);

        storeValues();
    }

    public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {
        storeValues();
    }

    public void onNothingSelected(AdapterView<?> parentView) {

    }

    private void storeValues() {
        editor.putString("club.towr5291.Autonomous.TeamNumber", teamNumber.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Color", allianceColor.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.StartPosition", allianceStartPosition.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.ParkPosition", allianceParkPosition.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Beacons", allianceBeacons.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Delay", delay.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.RobotConfig", robotConfig.getSelectedItem().toString());

        editor.commit();
    }
}