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
import club.towr5291.astarpathfinder.fourValues;
import club.towr5291.functions.AStarGetPath;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.Point;


/**
 * Created by ianhaden on 2/09/16.
 */

@Autonomous(name="Concept: A Star Path Fincder", group="5291Concept")
//@Disabled
public class ConceptAStarPathFinder extends OpMode {


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

    AStarValue AStarValues = new AStarValue();
    //AStarValue AStarGValue = new AStarValue();
    //AStarValue AStarHValue = new AStarValue();
    //AStarValue AStarParent = new AStarValue();
    AStarValue AStarClosed = new AStarValue();
    AStarValue AStarOpen = new AStarValue();

    public HashMap<String,AStarValue> AStarValueMap = new HashMap<String,AStarValue>();

    public fourValues[] pathValues = new fourValues[1000];
    public int pathIndex = 0;

    //HashMap<Integer,AStarValue> AStarGValueMap = new HashMap<Integer,AStarValue>(500);
    //HashMap<Integer,AStarValue> AStarHValueMap = new HashMap<Integer,AStarValue>(500);
    //HashMap<Integer,AStarValue> AStarParentMap = new HashMap<Integer,AStarValue>();
    public HashMap<String,AStarValue> AStarClosedMap = new HashMap<String,AStarValue>();
    public HashMap<String,AStarValue> AStarOpenMap = new HashMap<String,AStarValue>();

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
        boolean searching = true;
        for ( int i = 0; i < pathValues.length; i++) {
            pathValues[i] = new fourValues();
        }

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


        //load start point
        int startX = 122;
        int startY = 122;
        int endX = 12;
        int endY = 80;
        AStarValues.xvalue = startX;
        AStarValues.yvalue = startY;
        AStarValues.GValue = 0;
        AStarValues.HValue = 0;
        AStarValues.FValue = 0;
        AStarValues.Parent = 0;
        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue));

        fileLogger.writeEvent("init()","loaded astargvalue into hashmap");

        for (String key: AStarValueMap.keySet())
            fileLogger.writeEvent("init()","Keys " + key + ": x:" + AStarValueMap.get(key).xvalue + " y:" + AStarValueMap.get(key).yvalue);

        //process map
        fourValues currentResult = new fourValues(1,0,AStarValues.xvalue,AStarValues.yvalue);

        while (searching) {
            fileLogger.writeEvent("init()","Searching ");
            currentResult = (ProcessCurrentNode((int)currentResult.val3, (int)currentResult.val4, endX, endY));
            searching = (currentResult.val1 == 1);
        }

        boolean found = (currentResult.val2 == 1);

        if (found) {

            for ( int i = 0; i < pathValues.length; i++) {
                fileLogger.writeEvent("init()","Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 );
                if ((pathValues[i].val2 == endX) && (pathValues[i].val3 == endY)) {
                    break;
                }
            }

//            curNode = getKey(endX, endY, a0Star.fieldWidth, a0Star.fieldLength);
//
//
//            while (curNode != getKey(startX, startY, a0Star.fieldWidth, a0Star.fieldLength)) {
//                fileLogger.writeEvent("init()","curNode " + curNode);
//                pathValues[pathIndex].val1 = (double)pathIndex;
//                pathValues[pathIndex].val2 = (double)getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);
//                pathValues[pathIndex].val3 = (double)getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);
//
//                processNodes = AStarValueMap.get(String.valueOf(curNode));   //curNode = ds_map_find_value(P, curNode);
//                curNode = processNodes.Parent;
//                pathIndex++;
//           }
//            fileLogger.writeEvent("init()","Outside While");
//            pathValues[pathIndex].val1 = pathIndex;
//            pathValues[pathIndex].val2 = startX;
//            pathValues[pathIndex].val3 = startY;
//
//            //path_reverse(path);
//            //path_set_closed(path, false);

        }

        //plot out path..
        //for (int i = pathIndex; i >= 0; i-- ) {
        //    fileLogger.writeEvent("init()","Path X= " + pathValues[pathIndex].val2  + " Y= " + pathValues[pathIndex].val3);

        //}



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
    public int getXPos(double key, int width, int length) {
        int value;
        value = (int)key % width;  //modulo
        return value;
    }
    public int getYPos(double key, int width, int length) {
        int value;
        value = (int)key / width;  //integer division
        return value;
    }

    public fourValues ProcessCurrentNode (int currentX, int currentY, int endX, int endY) {
        boolean closed = false;
        boolean diagonal = false;
        boolean canWalk = false;
        int searching = 1;
        int found = 0;
        boolean empty = false;
        double distFromCurrentToIJ = 0, distFromStartToCurrent = 0;
        double tempF, tempG, tempH;
        int minF;
        double lowestF = -1;
        int lowestFKey = 0;
        fourValues returnValue = new fourValues(0,0,0,0);

        AStarValue AStarValueCurrerntXY = new AStarValue();
        AStarValue AStarValueCurrerntIJ = new AStarValue();

        tempF = 99999;

        //add point to be process to the closed list
        AStarClosed.xvalue = currentX;
        AStarClosed.yvalue = currentY;
        AStarClosed.ID = getKey(AStarClosed.xvalue, AStarClosed.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        AStarClosedMap.put(String.valueOf(AStarClosed.ID), AStarClosed);
        //fileLogger.writeEvent("ProcessCurrentNode()","Added to closed list ");

        //analyze adjacent blocks/grid locations
        //key exists so get the values
        if (AStarValueMap.containsKey(getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength)))
            AStarValueCurrerntXY = AStarValueMap.get(String.valueOf(getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength)));
        else {
            AStarValueCurrerntXY.xvalue = currentX;
            AStarValueCurrerntXY.yvalue = currentY;
            AStarValueCurrerntXY.FValue = 0;
            AStarValueCurrerntXY.GValue = 0;
            AStarValueCurrerntXY.HValue = 0;
            AStarValueCurrerntXY.ID = 0;
        }

        AStarValues.xvalue = currentX;
        AStarValues.yvalue = currentY;
        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);

        for (int i = Math.max(0, (currentX - 1)); i <= Math.min(a0Star.fieldWidth - 1, currentX + 1); i++ ) {
            for (int j = Math.max(0, (currentY - 1)); j <= Math.min(a0Star.fieldLength - 1, currentY + 1); j++ ) {
                if ((i == currentX) && (j == currentY)) {
                    //fileLogger.writeEvent("ProcessCurrentNode()","i=x and j=y - nothing to do");
                } else {
                    //check if its on the closed list
                    //fileLogger.writeEvent("ProcessCurrentNode()","checking if on closed list " + i + " " + j);
                    if (AStarClosedMap.containsKey(String.valueOf(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength))))
                        closed = true;      //need to check if this returns null it doesn't error out
                    else
                        closed = false;                                                                 //not on closed list, must be on open list
                    //fileLogger.writeEvent("ProcessCurrentNode()","on closed list " + closed);

                    //fileLogger.writeEvent("ProcessCurrentNode()","checking if diagonal ");
                    if (( i + j ) % 2  == (currentX + currentY) % 2)
                        diagonal = true;
                    else
                        diagonal = false;

                    //fileLogger.writeEvent("ProcessCurrentNode()","Is diagonal " + diagonal);

                    if (diagonal) {
                        canWalk = a0Star.walkable[i][j] && a0Star.walkable[currentX][j] && a0Star.walkable[i][currentY];
                        distFromCurrentToIJ = 1420;                                                      //G Value
                    } else {
                        canWalk = a0Star.walkable[i][j];
                        distFromCurrentToIJ = 1000;                                                      //G Value
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
                        AStarValueCurrerntIJ = AStarValueMap.get(String.valueOf(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength)));
                        if (tempG < AStarValueCurrerntIJ.GValue) {
                            AStarValueMap.remove(String.valueOf(AStarValues.ID));
                            //fileLogger.writeEvent("ProcessCurrentNode()", "Removed OLD Key (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);

                            AStarValues.xvalue = i;
                            AStarValues.yvalue = j;
                            AStarValues.GValue = tempG;
                            AStarValues.HValue = tempH;
                            AStarValues.FValue = tempF;
                            AStarValues.Parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue));
                            //fileLogger.writeEvent("ProcessCurrentNode()", "Updating (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                        }
                    } else {
                        AStarValues.xvalue = i;
                        AStarValues.yvalue = j;
                        AStarValues.GValue = tempG;
                        AStarValues.HValue = tempH;
                        AStarValues.FValue = tempF;
                        AStarValues.Parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue));
                        //fileLogger.writeEvent("ProcessCurrentNode()", "Adding (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                    }
                }
            }
        }
        lowestF = 999999;
        //find best option
        for (String key: AStarValueMap.keySet()) {
            AStarValueCurrerntIJ = AStarValueMap.get(key);
            //fileLogger.writeEvent("ProcessCurrentNode()", "Valid key " + key + " ID " + AStarValueCurrerntIJ.ID + " FValue " + AStarValueCurrerntIJ.FValue + " LowestF " + lowestF);
            if (AStarValueCurrerntIJ.FValue < lowestF) {
                lowestF = AStarValueCurrerntIJ.FValue;
                lowestFKey = AStarValueCurrerntIJ.ID;
                //fileLogger.writeEvent("ProcessCurrentNode()", "Found LowerF " + lowestF);
            }
        }
        if (lowestF != 999999) {
            //found low values in map, remove it from map
            pathValues[pathIndex].val1 = (double)pathIndex;
            pathValues[pathIndex].val2 = currentX;
            pathValues[pathIndex].val3 = currentY;  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);

            pathIndex++;

            AStarValueMap.remove(lowestFKey);
            currentX = getXPos(lowestFKey, a0Star.fieldWidth, a0Star.fieldLength);
            currentY = getYPos(lowestFKey, a0Star.fieldWidth, a0Star.fieldLength);
            fileLogger.writeEvent("ProcessCurrentNode()", "Trying Key " + getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength) + " (" + currentX +  " " + currentY + ")");
            searching = 1;
            found = 0;
        } else {
            searching = 0;
            found = 0;
            fileLogger.writeEvent("ProcessCurrentNode()", "No More Nodes Left");
        }
        //check whether we're at the end
        if ((currentX == endX) && (currentY == endY)) {
            pathValues[pathIndex].val1 = (double)pathIndex;
            pathValues[pathIndex].val2 = currentX;
            pathValues[pathIndex].val3 = currentY;  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);

            searching = 0;
            found = 1;
            fileLogger.writeEvent("ProcessCurrentNode()", "You found me I'm the final block");
        }
        returnValue.val1 = searching;
        returnValue.val2 = found;
        returnValue.val3 = currentX;
        returnValue.val4 = currentY;
        //fileLogger.writeEvent("ProcessCurrentNode()", "Returning 1- " + searching + " 2- " + found + " 3- "  + currentX + " 4- " + currentY);
        return returnValue;
    }


    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------



}
