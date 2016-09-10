package club.towr5291.functions;

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

public class AStarGetPath {

    public int startFieldX;
    public int startFieldY;
    public int targetFieldX;
    public int targetFieldY;


    public int AStarGetPath (int startX, int startY, int endX, int endY){
        this.startFieldX  = startX;
        this.startFieldY  = startY;
        this.targetFieldX = endX;
        this.targetFieldY = endY;

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
        int path = 1112;
        return path;


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

}
