package club.towr5291.Concepts;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;

import club.towr5291.functions.FileLogger;


/**
 * Created by ianhaden on 19/01/2017.
 */

@TeleOp(name = "Concept Read Steps", group = "5291Concept")

public class ConceptReadStepFile extends LinearOpMode {


    //set up the variables for the logger
    final String TAG = "Concept Read File";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    private void readSteps() {

        try {
            File f = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Sequences"), "5291RedLeft.csv");
            BufferedReader reader = new BufferedReader(new FileReader(f));

            String csvLine;
            while((csvLine = reader.readLine()) != null) {
                //check if line is a comment and ignore it
                if (csvLine.substring(0, 2).equals("//")) {

                } else {

                    String[] row = csvLine.split(",");
                    fileLogger.writeEvent(TAG, "CSV Value " + row[0].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[1].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[2].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[3].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[4].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[5].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[6].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[7].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[8].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[9].trim());
                    fileLogger.writeEvent(TAG, "CSV Value " + row[10].trim());
                    fileLogger.writeEvent(TAG, "Finished Line....");

                }



            }
        } catch(IOException ex) {
            //throw new RuntimeException("Error in reading CSV file:" + ex);
            fileLogger.writeEvent(TAG, "Error in reading CSV file:" + ex);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent(TAG, "Log Started");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        readSteps();

        while (opModeIsActive()) {

            fileLogger.writeEvent("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            idle();
        }
    }
}
