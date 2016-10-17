package club.towr5291.functions;

import android.util.Log;

import java.util.HashMap;

import club.towr5291.astarpathfinder.A0Star;
import club.towr5291.astarpathfinder.AStarValue;
import club.towr5291.astarpathfinder.sixValues;

/**
 * Created by ianhaden on 5/09/16.
 * //creates a path from (startX,startY) to (endX,endY)
 // param0 startFieldX = startX : starting x position
 // param1 startFieldY = startY : starting y position
 // param2 targetFieldX = endX : ending x position
 // param3 targetFieldY = endY : ending y position

 //NOTE : Inputs are in terms of room positions.
 //       All other positions will be in terms of grid

 */

public class AStarGetPathBasic {

    private static final String TAG = "AStarGetPathBasic";

    private A0Star a0Star = new A0Star();
    private sixValues[] pathValues = new sixValues[1000];
    private int pathIndex;

    private AStarValue AStarValues = new AStarValue();
    private AStarValue AStarClosed = new AStarValue();
    private AStarValue AStarOpen = new AStarValue();

    private HashMap<String, AStarValue> AStarClosedMap = new HashMap<String, AStarValue>();
    private HashMap<String, AStarValue> AStarValueMap = new HashMap<String, AStarValue>();

    public AStarGetPathBasic(){
        for ( int i = 0; i < pathValues.length; i++) {
            pathValues[i] = new sixValues();
        }
        pathIndex = 0;


        //_____PRE-ALGOR____
        //convert vars into grid


//        F=ds_priority_create();
//        C=ds_list_create();


        //init first G value
        //ds_map_add(G,getKey(startX,startY),0);

        //_____ALGOR____
//        searching=true;
//        found=false;
//        curX=startX;
//        curY=startY;
//        while(searching){
//            processCurrentNode();
//        }
//
//        var path=-1;
//        if(found){
//            path=path_add();
//            var curNode=getKey(endX,endY);
//            while(curNode!=getKey(startX,startY)){
//                path_add_point(path,getKeyX(curNode)*oAStar.blockSize,
//                        getKeyY(curNode)*oAStar.blockSize,100);
//                curNode=ds_map_find_value(P,curNode);
//            }
//            path_add_point(path,startX*oAStar.blockSize,startY*oAStar.blockSize,100);
//            path_reverse(path);
//            path_set_closed(path,false);
//        }


        //_____POST-ALGOR____
        //destroy datastructures
//        ds_map_destroy(G);
//        ds_map_destroy(H);
//        ds_priority_destroy(F);
//        ds_map_destroy(P);
//        ds_list_destroy(C);

        //return our result

    }


    public int getKey(int xPos, int yPos, int width, int length) {
        int value;
        value = yPos*length + xPos;
        return value;
    }

    public int getXPos(int key, int width, int length) {
        int value;
        value = (int)key % width;  //modulo
        return value;
    }

    public int getYPos(int key, int width, int length) {
        int value;
        value = (int)key / width;  //integer division
        return value;
    }

    public sixValues ProcessCurrentNode (int currentX, int currentY, int endX, int endY) {
        boolean closed = false;
        boolean diagonal = false;
        boolean canWalk = false;
        int searching = 1;
        int found = 0;
        double distFromCurrentToIJ = 0, distFromStartToCurrent = 0;
        double tempF, tempG, tempH;
        double lowestF = -1;
        int lowestFKey = 0;
        sixValues returnValue = new sixValues(0,0,0,0,0,0);

        AStarValue AStarValueCurrentXY = new AStarValue();
        AStarValue AStarValueCurrentIJ = new AStarValue();

        //add point to be process to the closed list
        AStarClosed.xvalue = currentX;
        AStarClosed.yvalue = currentY;
        AStarClosed.ID = getKey(AStarClosed.xvalue, AStarClosed.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        //AStarClosedMap.put(String.valueOf(AStarClosed.ID), AStarClosed);
        AStarClosedMap.put(String.valueOf(AStarClosed.ID), new  AStarValue(AStarClosed.ID, AStarClosed.FValue, AStarClosed.GValue, AStarClosed.HValue, AStarClosed.Parent, AStarClosed.xvalue, AStarClosed.yvalue, AStarClosed.zvalue));

        //fileLogger.writeEvent("ProcessCurrentNode()","Added to closed list ");
        Log.d(TAG, "Added to closed list ");

        //analyze adjacent blocks/grid locations
        //key exists so get the values
        int nodeKey = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
        if (AStarValueMap.containsKey(String.valueOf(nodeKey))){
            AStarValueCurrentXY = AStarValueMap.get(String.valueOf(nodeKey));
        }
        else {
            AStarValueCurrentXY.xvalue = currentX;
            AStarValueCurrentXY.yvalue = currentY;
            AStarValueCurrentXY.FValue = 0;
            AStarValueCurrentXY.GValue = 0;
            AStarValueCurrentXY.HValue = 0;
            AStarValueCurrentXY.ID = 0;
        }

        AStarValues.xvalue = currentX;
        AStarValues.yvalue = currentY;
        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);

        for (int i = Math.max(0, (currentX - 1)); i <= Math.min(a0Star.fieldWidth - 1, currentX + 1); i++ ) {
            for (int j = Math.max(0, (currentY - 1)); j <= Math.min(a0Star.fieldLength - 1, currentY + 1); j++ ) {
                if ((i == currentX) && (j == currentY)) {
                    //fileLogger.writeEvent("ProcessCurrentNode()","i=x and j=y - nothing to do");
                    Log.d(TAG, "i=x and j=y - nothing to do");
                } else {
                    //check if its on the closed list
                    //fileLogger.writeEvent("ProcessCurrentNode()","checking if on closed list " + i + " " + j);
                    Log.d(TAG, "checking if on closed list " + i + " " + j);
                    if (AStarClosedMap.containsKey(String.valueOf(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength))))
                        closed = true;      //need to check if this returns null it doesn't error out
                    else
                        closed = false;                                                                 //not on closed list, must be on open list
                    //fileLogger.writeEvent("ProcessCurrentNode()","on closed list " + closed);
                    Log.d(TAG, "on closed list " + closed + " then checking if diagonal ");

                    if (( i + j ) % 2  == (currentX + currentY) % 2)
                        diagonal = true;
                    else
                        diagonal = false;

                    Log.d(TAG, "Is diagonal " + diagonal);

                    if (diagonal) {
                        canWalk = a0Star.walkable[i][j] && a0Star.walkable[currentX][j] && a0Star.walkable[i][currentY];
                        distFromCurrentToIJ = 1410;                                                      //G Value
                    } else {
                        canWalk = a0Star.walkable[i][j];
                        distFromCurrentToIJ = 1000;                                                      //G Value
                    }
                }
                if (!closed && canWalk){
                    //calculated G,H,and F
                    tempG = AStarValueCurrentXY.GValue + distFromCurrentToIJ;                     // distance from starttocurrent + value just calculated
                    Log.d(TAG, "tempG " + tempG);
                    tempH = ((Math.abs(i - endX) * 1000 + Math.abs(j - endY) * 1000));                            //insert heuristic of choice (we use manhattan)
                    Log.d(TAG, "tempH " + tempH);
                    tempF = tempG + tempH;
                    Log.d(TAG, "tempF " + tempF);
                    //update if necessary
                    if (AStarValueMap.containsKey(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength))){   //see if key is in G Map, means already processed
                        AStarValueCurrentIJ = AStarValueMap.get(String.valueOf(getKey(i, j, a0Star.fieldWidth, a0Star.fieldLength)));
                        if (tempG < AStarValueCurrentIJ.GValue) {
                            AStarValueMap.remove(String.valueOf(AStarValues.ID));
                            //fileLogger.writeEvent("ProcessCurrentNode()", "Removed OLD Key (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                            //Log.d(TAG, "Removed OLD Key (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);

                            AStarValues.xvalue = i;
                            AStarValues.yvalue = j;
                            AStarValues.zvalue = 0;
                            AStarValues.GValue = tempG;
                            AStarValues.HValue = tempH;
                            AStarValues.FValue = tempF;
                            AStarValues.Parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                            AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue, AStarValues.zvalue));
                            //fileLogger.writeEvent("ProcessCurrentNode()", "Updating (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                            Log.d(TAG, "Updating (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                        }
                    } else {
                        AStarValues.xvalue = i;
                        AStarValues.yvalue = j;
                        AStarValues.zvalue = 0;
                        AStarValues.GValue = tempG;
                        AStarValues.HValue = tempH;
                        AStarValues.FValue = tempF;
                        AStarValues.Parent = getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
                        AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue, AStarValues.yvalue));
                        //fileLogger.writeEvent("ProcessCurrentNode()", "Adding (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                        Log.d(TAG, "Adding (" + i + "," + j + ") Key " + AStarValues.ID + "    G:" + tempG + "     H:" + tempH + "     F:" + tempF);
                    }
                }
            }
        }
        lowestF = 999999;
        //find best option
        for (String key: AStarValueMap.keySet()) {
            AStarValueCurrentIJ = AStarValueMap.get(key);
            //fileLogger.writeEvent("ProcessCurrentNode()", "Valid key " + key + " ID " + AStarValueCurrerntIJ.ID + " FValue " + AStarValueCurrerntIJ.FValue + " LowestF " + lowestF);
            //Log.d(TAG, "Valid key " + key + " ID " + AStarValueCurrentIJ.ID + " FValue " + AStarValueCurrentIJ.FValue + " LowestF " + lowestF);
            if (AStarValueCurrentIJ.FValue < lowestF) {
                lowestF = AStarValueCurrentIJ.FValue;
                lowestFKey = AStarValueCurrentIJ.ID;
                //fileLogger.writeEvent("ProcessCurrentNode()", "Found LowerF " + lowestF);
                Log.d(TAG, "Found LowerF " + lowestF);
            }
        }
        if (lowestF != 999999) {
            if (pathIndex == 100) {
                Log.d(TAG, "Can't find path, exiting ");
                // break out of loop, can't find path....
                searching = 0;
                found = 1;
            } else {
                //found low values in map, remove it from map
                Log.d(TAG, "Found Lowest F, now getting next point ");
                pathValues[pathIndex].val1 = (double) pathIndex;
                pathValues[pathIndex].val2 = currentX;
                pathValues[pathIndex].val3 = currentY;  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);

                pathIndex++;

                AStarValueMap.remove(lowestFKey);
                currentX = getXPos(lowestFKey, a0Star.fieldWidth, a0Star.fieldLength);
                currentY = getYPos(lowestFKey, a0Star.fieldWidth, a0Star.fieldLength);
                //fileLogger.writeEvent("ProcessCurrentNode()", "Trying Key " + getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength) + " (" + currentX +  " " + currentY + ")");
                Log.d(TAG, "Trying Key " + getKey(currentX, currentY, a0Star.fieldWidth, a0Star.fieldLength) + " (" + currentX + " " + currentY + ")");
                searching = 1;
                found = 0;
            }
        } else {
            searching = 0;
            found = 0;
            //fileLogger.writeEvent("ProcessCurrentNode()", "No More Nodes Left");
            Log.d(TAG, "No More Nodes Left");
        }
        //check whether we're at the end
        if ((currentX == endX) && (currentY == endY)) {
            pathValues[pathIndex].val1 = (double)pathIndex;
            pathValues[pathIndex].val2 = currentX;
            pathValues[pathIndex].val3 = currentY;  //path_add_point (path, getXPos(curNode, a0Star.fieldWidth, a0Star.fieldLength), getYPos(curNode, a0Star.fieldWidth, a0Star.fieldLength);

            searching = 0;
            found = 1;
            //fileLogger.writeEvent("ProcessCurrentNode()", "You found me I'm the final block");
            Log.d(TAG, "You found me I'm the final block");
        }
        returnValue.val1 = searching;
        returnValue.val2 = found;
        returnValue.val3 = currentX;
        returnValue.val4 = currentY;
        //fileLogger.writeEvent("ProcessCurrentNode()", "Returning 1- " + searching + " 2- " + found + " 3- "  + currentX + " 4- " + currentY);
        Log.d(TAG, "Returning 1- " + searching + " 2- " + found + " 3- "  + currentX + " 4- " + currentY);
        return returnValue;
    }

    public sixValues[] findPathAStar (int startX, int startY, int endX, int endY ) {
        boolean searching = true;

        AStarValues.xvalue = startX;
        AStarValues.yvalue = startY;
        AStarValues.zvalue = 0;
        AStarValues.GValue = 0;
        AStarValues.HValue = 0;
        AStarValues.FValue = 0;
        AStarValues.Parent = 0;
        AStarValues.ID = getKey(AStarValues.xvalue, AStarValues.yvalue, a0Star.fieldWidth, a0Star.fieldLength);
        AStarValueMap.put(String.valueOf(AStarValues.ID), new  AStarValue(AStarValues.ID, AStarValues.FValue, AStarValues.GValue, AStarValues.HValue, AStarValues.Parent, AStarValues.xvalue, AStarValues.yvalue, AStarValues.zvalue));

        for (String key: AStarValueMap.keySet()) {
            //fileLogger.writeEvent("init()", "Keys " + key + ": x:" + AStarValueMap.get(key).xvalue + " y:" + AStarValueMap.get(key).yvalue);
            Log.d(TAG, "Keys " + key + ": x:" + AStarValueMap.get(key).xvalue + " y:" + AStarValueMap.get(key).yvalue );
        }
        //process map
        sixValues currentResult = new sixValues(1,0,startX,startY, 0, 0);

        while (searching) {
            //fileLogger.writeEvent("init()","Searching ");
            Log.d(TAG, "Searching..." );
            currentResult = (ProcessCurrentNode((int)currentResult.val3, (int)currentResult.val4, endX, endY));
            searching = (currentResult.val1 == 1);
        }

        boolean found = (currentResult.val2 == 1);

//        if (found) {
//
//            for ( int i = 0; i < pathValues.length; i++) {
//                //fileLogger.writeEvent("init()","Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 );
//                Log.d(TAG, "Path " + pathValues[i].val1 + " " + pathValues[i].val2 + " " + pathValues[i].val3 );
//                if ((pathValues[i].val2 == endX) && (pathValues[i].val3 == endY)) {
//                    break;
//                }
//            }
//        }
        return pathValues;
    }
}
