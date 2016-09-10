package club.towr5291.opmodes;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.AStarValue;
import club.towr5291.functions.AStarGetPath;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.Point;



/**
 * Created by ianhaden on 2/09/16.
 */

@Autonomous(name="Pushbot: Path Generator", group="5291Test")
//@Disabled
public class AutoDriveRedAPath extends OpMode {


    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    //define each state for the step.  Each step should go through some of the states below
    private enum stepState {
        STATE_INIT,
        STATE_START,
        STATE_RUNNING,
        STATE_PAUSE,
        STATE_COMPLETE,
        STATE_TIMEOUT,
        STATE_ERROR,
        STATE_FINISHED
    }

    private int mCurrentStep = 0;                               // Current State Machine State.
    private stepState  mCurrentStepState;                       // Current State Machine State.
    private stepState  mCurrentDriveState;                      // Current State Machine State.
    private stepState  mCurrentTurnState;                       // Current State Machine State.
    private LibraryStateSegAuto[] mStateSegAuto;
    double mStepTimeout;
    double mStepDistance;
    double mStepSpeed;
    String mRobotDirection;
    double mStepTurnL;
    double mStepTurnR;
    int mStepLeftTarget;
    int mStepRightTarget;
    boolean baseStepComplete = false;
    boolean armStepComplete = true;

    public static final int FIELDWIDTH = 144;
    public static final int FIELDLENGTH = 144;
    boolean[][] fieldOpen = new boolean[FIELDWIDTH][FIELDLENGTH];
    boolean[][] fieldObstacles = new boolean[FIELDWIDTH][FIELDLENGTH];
    boolean[][] gValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // Movement Cost
    boolean[][] hValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // Heuristic
    boolean[][] fValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // this is hValues + gValues
    boolean[][] parentValues = new boolean[FIELDWIDTH][FIELDLENGTH];    // parents
    boolean[] closedList = new boolean[FIELDWIDTH * FIELDLENGTH];  // closed list.  List of nodes already checked
    boolean[] openList = new boolean[FIELDWIDTH * FIELDLENGTH];    // open list.  List of nodes to be checked

    AStarValue AStarValue = new AStarValue();
    //AStarValue AStarGValue = new AStarValue();
    //AStarValue AStarHValue = new AStarValue();
    //AStarValue AStarParent = new AStarValue();
    AStarValue AStarClosed = new AStarValue();
    AStarValue AStarOpen = new AStarValue();

    HashMap<Integer,AStarValue> AStarValueMap = new HashMap<Integer,AStarValue>(500);
    //HashMap<Integer,AStarValue> AStarGValueMap = new HashMap<Integer,AStarValue>(500);
    //HashMap<Integer,AStarValue> AStarHValueMap = new HashMap<Integer,AStarValue>(500);
    //HashMap<Integer,AStarValue> AStarParentMap = new HashMap<Integer,AStarValue>();
    HashMap<Integer,AStarValue> AStarClosedMap = new HashMap<Integer,AStarValue>();
    HashMap<Integer,AStarValue> AStarOpenMap = new HashMap<Integer,AStarValue>();

    A0Star a0Star = new A0Star();

    String fieldOutput;

    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();           // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();         // Time into current state

    //this is the sequence the state machine will follow
    private LibraryStateSegAuto[] mRobotAutonomous = {
            //                        time, head, dist, powe
            //                        out   ing   ance  r
            //                         s    deg   inch   %
            new LibraryStateSegAuto ( 10,  "L90",  12,  0.5 )

    };

    /*
    * Code to run ONCE when the driver hits INIT
    */
    @Override
    public void init() {
        int loopColumn;
        int loopRow;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("FileLogger: ", runtime.toString());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        // Send telemetry message to signify robot waiting;
        telemetry.update();

        mCurrentStepState = stepState.STATE_INIT;
        mCurrentTurnState = stepState.STATE_INIT;
        mCurrentDriveState = stepState.STATE_INIT;

        //fileLogger.writeEvent("init()","Creating Field Array");
        //generate field
//        for(loopRow = 0; loopRow < FIELDLENGTH; loopRow++)
//        {
//            for(loopColumn = 0; loopColumn < FIELDWIDTH; loopColumn++){
//                fieldOpen[loopColumn][loopRow] = true;
//            }
//        }
//
//        fieldOutput = "";
//        fileLogger.writeEvent("init()","Writing Field Array");
//        for(loopRow = 0; loopRow < FIELDLENGTH; loopRow++)
//        {
//            for(loopColumn = 0; loopColumn < FIELDWIDTH; loopColumn++){
//                if (fieldOpen[loopColumn][loopRow])
//                    fieldOutput = fieldOutput + "1";
//            }
//            fileLogger.writeEvent("init()",fieldOutput);
//            fieldOutput = "";
//        }


        for(loopRow = 0; loopRow < FIELDLENGTH; loopRow++)
        {
            for(loopColumn = 0; loopColumn < FIELDWIDTH; loopColumn++){
                if (a0Star.walkable[loopColumn][loopRow])
                    fieldOutput = fieldOutput + "1";
            }
            fileLogger.writeEvent("init()",fieldOutput);
            fieldOutput = "";
        }
        fileLogger.writeEvent("init()","setting the class astargvalue");


        //load start point
        AStarValue.xvalue = 122;
        AStarValue.yvalue = 122;
        AStarValue.GValue = 0;
        AStarValue.HValue = 0;
        AStarValue.FValue = 0;
        AStarValue.Parent = 0;
        AStarValue.ID = getKey(AStarValue.xvalue, AStarValue.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        AStarValueMap.put(AStarValue.ID, AStarValue);

        fileLogger.writeEvent("init()","loaded astargvalue into hashmap");

        for (Integer key: AStarValueMap.keySet())
            fileLogger.writeEvent("init()","Keys " + key + ": x:" + AStarValueMap.get(key).xvalue + " y:" + AStarValueMap.get(key).yvalue);


        fileLogger.writeEvent("init()","Init Complete");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        fileLogger.writeEvent("start()","START PRESSED: ");


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        fileLogger.writeEvent("loop()","STATE: " + String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString() + " Current Turn State:- " + mCurrentTurnState.toString()+ " Current Drive State:- " + mCurrentDriveState.toString());
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("STATE", String.format("%4.1f ", mStateTime.time()) + " Current Step:- " + mCurrentStep + " Current Step State:- " + mCurrentStepState.toString());
        // Execute the current state.  Each STATE's case code does the following:

        switch (mCurrentStepState)
        {
            case STATE_INIT:
            {

            }
            break;
            case STATE_START:
            {

            }
            break;
            case STATE_RUNNING:
            {

                if ((mCurrentDriveState == stepState.STATE_COMPLETE) && (mCurrentTurnState == stepState.STATE_COMPLETE) && (armStepComplete))
                {
                    //  Transition to a new state.
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
                fileLogger.writeEvent("loop()","Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                if ((mCurrentStep) < (mRobotAutonomous.length - 1)) {
                    fileLogger.writeEvent("loop()","Current Step:- " + mCurrentStep + ", Array Size: " + mRobotAutonomous.length);
                    //  Transition to a new state and next step.
                    mCurrentStep++;
                    mCurrentStepState = stepState.STATE_INIT;

                } else {
                    fileLogger.writeEvent("loop()","STATE_COMPLETE - Setting FINISHED ");
                    //  Transition to a new state.
                    mCurrentStepState = stepState.STATE_FINISHED;
                }
            }
            break;
            case STATE_TIMEOUT:
            {
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
                telemetry.addData("STATE", "FINISHED " + mCurrentStep);
            }
            break;

        }

        //check timeout vale
        if ((mStateTime.seconds() > mStepTimeout  ) && ((mCurrentStepState != stepState.STATE_ERROR) && (mCurrentStepState != stepState.STATE_FINISHED))) {
            //  Transition to a new state.
            mCurrentStepState = stepState.STATE_TIMEOUT;
        }

        telemetry.update();
    }


    /*
    * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop()
    {
        telemetry.addData("FileLogger Op Stop: ", runtime.toString());
        if (fileLogger != null) {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    }


    public int getKey(int xPos, int yPos, int width, int length) {
        int value;
        value = yPos*length + xPos;
        return value;
    }
    public int getXPos(int key, int width, int length) {
        int value;
        value = key % width;  //modulo
        return value;
    }
    public int getYPos(int key, int width, int length) {
        int value;
        value = key / width;  //integer division
        return value;
    }

    public int ProcessCurrentNode (int currentx, int currenty, int endX, int endY) {
        boolean closed = false;
        boolean diagonal = false;
        boolean canWalk = false;
        boolean searching = false;
        boolean found = false;
        boolean empty = false;
        double distFromCurrentToIJ = 0, distFromStartToCurrent = 0;
        double tempF, tempG, tempH;
        int minF;
        AStarValue AStarValueCurrerntXY = new AStarValue();
        AStarValue AStarValueCurrerntIJ = new AStarValue();

        //add point to be process to the closed list
        AStarClosed.xvalue = currentx;
        AStarClosed.yvalue = currenty;
        AStarClosed.ID = getKey(AStarClosed.xvalue, AStarClosed.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        AStarClosedMap.put(AStarClosed.ID, AStarClosed);
        fileLogger.writeEvent("ProcessCurrentNode()","Added to closed list ");

        //analyze adjacent blocks/grid locations
        //key exists so get the values
        if (AStarValueMap.containsKey(getKey(currentx, currenty, a0Star.fieldWidth, a0Star.fieldLength)))
            AStarValueCurrerntXY = AStarValueMap.get(getKey(currentx, currenty, a0Star.fieldWidth, a0Star.fieldLength));
        else {
            AStarValueCurrerntXY.xvalue = currentx;
            AStarValueCurrerntXY.yvalue = currenty;
            AStarValueCurrerntXY.FValue = 0;
            AStarValueCurrerntXY.GValue = 0;
            AStarValueCurrerntXY.HValue = 0;
            AStarValueCurrerntXY.ID = 0;
        }
        //var distFromStartToCurrent=ds_map_find_value(G,getKey(curX,curY));

        AStarValue.xvalue = currentx;
        AStarValue.yvalue = currenty;
        AStarValue.ID = getKey(AStarValue.xvalue, AStarValue.yvalue, a0Star.fieldWidth, a0Star.fieldLength);

        for (int i = Math.max(0, (currentx - 1)); i <= Math.min(a0Star.fieldWidth - 1, currentx + 1); i++ ) {
            for (int j = Math.max(0, (currenty - 1)); j <= Math.min(a0Star.fieldLength - 1, currenty + 1); j++ ) {
                if ((i == currentx) && (j == currenty)) {
                    fileLogger.writeEvent("ProcessCurrentNode()","i=x and j=y - nothing to do");
                } else {
                    //check if its on the closed list
                    fileLogger.writeEvent("ProcessCurrentNode()","checking if on closed list " + i + " " + j);
                    if (AStarClosedMap.containsKey(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength)))
                        closed = true;      //need to check if this returns null it doesn't error out
                    else
                        closed = false;                                                                 //not on closed list, must be on open list
                    fileLogger.writeEvent("ProcessCurrentNode()","on closed list " + closed);

                    fileLogger.writeEvent("ProcessCurrentNode()","checking if diagonal ");
                    if (( i + j ) % 2  == (currentx + currenty) % 2)
                        diagonal = true;
                    else
                        diagonal = false;

                    fileLogger.writeEvent("ProcessCurrentNode()","Is diagonal " + diagonal);

                    if (diagonal) {
                        canWalk = a0Star.walkable[i][j] && a0Star.walkable[currentx][j] && a0Star.walkable[i][currenty];
                        distFromCurrentToIJ = 1.414;                                                        //G Value
                    } else {
                        canWalk = a0Star.walkable[i][j];
                        distFromCurrentToIJ = 1;                                                            //G Value
                    }
                }
                if (!closed && canWalk){
                    //calculated G,H,and F
                    tempG = AStarValueCurrerntXY.GValue + distFromCurrentToIJ;
                    tempH = Math.abs(i - endX) + Math.abs(j - endY);                              //insert heuristic of choice (we use manhattan)
                    //NOTE : you could also use point_distance(i,j,endX,endY);
                    tempF = tempG + tempH;
                    //update if necessary
                    if (AStarValueMap.containsKey(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength))){   //see if key is in G Map, means already processed
                        //var oldG=;
                        //show_debug_message(string(tempG)+" compare to "+string(oldG));
                        AStarValueCurrerntIJ = AStarValueMap.get(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength));
                        if (tempG < AStarValueCurrerntIJ.GValue) {
                            AStarValueMap.remove(AStarValue.ID);
                            fileLogger.writeEvent("ProcessCurrentNode()","Removed OLD Key (" + i + "," + j + ")     G:" + tempG + "     H:" + tempH + "     F:" + tempF);

                            AStarValue.xvalue = i;
                            AStarValue.yvalue = j;
                            AStarValue.GValue = tempG;
                            AStarValue.HValue = tempH;
                            AStarValue.FValue = tempF;
                            AStarValue.Parent = getKey(currentx, currenty, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValue.ID = getKey(AStarValue.xvalue, AStarValue.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValueMap.put(AStarValue.ID, AStarValue);
                            fileLogger.writeEvent("ProcessCurrentNode()","Updating (" + i + "," + j + ")     G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                        }
                    } else {
                        AStarValue.xvalue = i;
                        AStarValue.yvalue = j;
                        AStarValue.GValue = tempG;
                        AStarValue.HValue = tempH;
                        AStarValue.FValue = tempF;
                        AStarValue.Parent = getKey(currentx, currenty, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValue.ID = getKey(AStarValue.xvalue, AStarValue.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValueMap.put(AStarValue.ID, AStarValue);
                        fileLogger.writeEvent("ProcessCurrentNode()","Adding (" + i + "," + j + ")     G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                    }
                }
            }
        }
        //find best option
        minF = -1;
        //empty = ds_priority_empty(F);
        //if (!empty)
        //    minF = ds_priority_delete_min(F);
        //decide what to do
        if (minF == -1){
            searching = false;
            found = false;
            fileLogger.writeEvent("ProcessCurrentNode()","No More Nodes Left");
        } else {
            fileLogger.writeEvent("ProcessCurrentNode()","Trying (" + getXPos(minF,a0Star.fieldWidth, a0Star.fieldLength) +  " " + getYPos(minF,a0Star.fieldWidth, a0Star.fieldLength) + ")");
            currentx = getXPos(minF, a0Star.fieldWidth, a0Star.fieldLength);
            currenty = getYPos(minF, a0Star.fieldWidth, a0Star.fieldLength);
        }
        //check whether we're at the end
        if (currentx == endX && currentx == endY){
            searching = false;
            found = true;
            fileLogger.writeEvent("ProcessCurrentNode()","You found me I'm the final block");
        }



    }


    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------



}
