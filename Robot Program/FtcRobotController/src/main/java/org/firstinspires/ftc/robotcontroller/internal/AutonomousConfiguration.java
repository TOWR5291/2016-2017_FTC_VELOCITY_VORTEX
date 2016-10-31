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


public class AutonomousConfiguration extends Activity implements NumberPicker.OnValueChangeListener, Spinner.OnItemSelectedListener {
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    Spinner allianceColor;
    Spinner alliancePosition;
    NumberPicker delay;

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
        int savedDelay = sharedPreferences.getInt("club.towr5291.Autonomous.Delay", 0);

        allianceColor = (Spinner) findViewById(R.id.spinner);
        allianceColor.setOnItemSelectedListener(this);
        allianceColor.setSelection(savedColor.equals("Red") ? 0 : 1, true);

        alliancePosition = (Spinner) findViewById(R.id.spinnerPosition);
        alliancePosition.setOnItemSelectedListener(this);
        alliancePosition.setSelection(savedPosition.equals("Left") ? 0 : 1, true);

        delay = (NumberPicker) findViewById(R.id.delay);
        delay.setOnValueChangedListener(this);
        delay.setMinValue(0);
        delay.setMaxValue(30);
        delay.setValue(savedDelay);
        delay.getValue();

        storeValues();
    }

    public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {
        //editor.putString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", parentView.getItemAtPosition(position).toString());
        //editor.putString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", allianceColor.getSelectedItem().toString());
        //editor.putString("com.qualcomm.ftcrobotcontroller.Autonomous.Position", alliancePosition.getSelectedItem().toString());
        //editor.putInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", delay.getValue());

        storeValues();

    }

    public void onNothingSelected(AdapterView<?> parentView) {

    }

    public void onValueChange(NumberPicker numberPicker, int oldVal, int newVal) {
        //editor.putInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", newVal);
        //editor.commit();
        storeValues();
    }

    private void storeValues() {
        editor.putString("club.towr5291.Autonomous.Color", allianceColor.getSelectedItem().toString());
        editor.putString("club.towr5291.Autonomous.Position", alliancePosition.getSelectedItem().toString());
        editor.putInt("club.towr5291.Autonomous.Delay", delay.getValue());

        editor.commit();
    }
}