package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import ftc5291.Log;
import ftc5291.Timers;

/**
 * Created by zoe on 8/9/2016.
 */
public class HelloDriver extends OpMode {

    int counter = 0;
    Log logging;
    Timers timing;

    public HelloDriver()  {
        super();
    }

    @Override
    public void init() {
        Long tsLong = System.currentTimeMillis()/1000;
        String ts = tsLong.toString();
        timing = new Timers();
        timing.startClock("global");
        logging = new Log("/FTC_Logs/", "test"+ts);
    }

    @Override
    public void loop() {
        counter++;
        if(counter % 10 == 0) {
            telemetry.addData("1", "HelloDriver count = " + counter);
            logging.add("simul ", counter);
        }

    }
    @Override
    public void stop() {
        logging.saveAs(Log.FileType.CSV);
        logging.saveAs(Log.FileType.JSON);
        logging.saveAs(Log.FileType.TEXT);
    }
}
