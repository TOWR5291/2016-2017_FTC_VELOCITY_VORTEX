package com.qualcomm.ftcrobotcontroller.opmodes;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.FileWriter;

import club.towr5291.functions.Log;
import club.towr5291.functions.Timers;

/**
 * Created by zoe on 8/9/2016.
 */
public class HelloDriverZoe extends OpMode {

    int counter = 0;
    Log logging;
    Timers timing;

    public HelloDriverZoe()  {
        super();
    }

    @Override
    public void init() {
        Long tsLong = System.currentTimeMillis()/1000;
        String ts = tsLong.toString();
        timing = new Timers();
        timing.startClock("global");
        logging = new Log("/FTC_Logs/", "test");
        try {
            String location;
            location = Environment.getExternalStorageDirectory().toString();
            FileWriter write = new FileWriter(location+"/FTC_Logs/text.txt", true);
            write.append("This is first written/n");
            telemetry.addData("1", "Logged " + location);
            write.close();
        }
        catch (Exception e) {
            telemetry.addData("1", "Error" + e);
        }

    }

    @Override
    public void loop() {
        counter++;
        if(counter % 10 == 0) {
            telemetry.addData("1", "HelloDriver count = " + counter);
            logging.add(counter + ":simul ", counter);
        }

    }
    @Override
    public void stop() {
        logging.saveAs(Log.FileType.CSV);
        logging.saveAs(Log.FileType.JSON);
        logging.saveAs(Log.FileType.TEXT);
        telemetry.addData("2", "HelloDriver Writing Log ");
    }
}
