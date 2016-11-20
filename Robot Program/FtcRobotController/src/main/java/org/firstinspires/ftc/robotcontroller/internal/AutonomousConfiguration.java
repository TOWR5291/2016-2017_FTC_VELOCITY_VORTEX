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
import android.widget.NumberPicker;
import android.widget.Spinner;
import android.widget.Toast;

import com.qualcomm.ftcrobotcontroller.R;


public class AutonomousConfiguration extends Activity implements Spinner.OnItemSelectedListener {
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    Spinner allianceColor;
    Spinner alliancePosition;
    Spinner allianceBeacons;
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

        String savedColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "null");
        String savedPosition = sharedPreferences.getString("club.towr5291.Autonomous.Position", "null");
        String savedBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "null");
        String savedConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "null");
        String savedDelay = sharedPreferences.getString("club.towr5291.Autonomous.Delay", "null");

        allianceColor = (Spinner) findViewById(R.id.alliance_color_options);
        allianceColor.setOnItemSelectedListener(this);
        allianceColor.setSelection(savedColor.equals("Red") ? 0 : 1, true);

        alliancePosition = (Spinner) findViewById(R.id.spinnerPosition);
        alliancePosition.setOnItemSelectedListener(this);
        alliancePosition.setSelection(savedPosition.equals("Left") ? 0 : 1, true);

        allianceBeacons = (Spinner) findViewById(R.id.spinnerBeacons);
        allianceBeacons.setOnItemSelectedListener(this);
        allianceBeacons.setSelection(savedBeacons.equals("One") ? 0 : 1, true);

        robotConfig = (Spinner) findViewById(R.id.spinnerRobotConfig);
        robotConfig.setOnItemSelectedListener(this);
        robotConfig.setSelection(savedConfig.equals("TileRunner-2x40") ? 0 : 1, true);

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
        editor.putString("club.towr5291.Autonomous.Color", allianceColor.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Position", alliancePosition.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Beacons", allianceBeacons.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Delay", delay.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.RobotConfig", robotConfig.getSelectedItem().toString());

        editor.commit();
    }
}