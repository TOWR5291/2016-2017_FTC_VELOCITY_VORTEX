package club.towr5291.astarpathfinder;

/**
 * Created by ianhaden on 7/09/16.
 */
public class A0Star {

    public static final int FIELDWIDTH = 144;
    public static final int FIELDLENGTH = 144;
    public int blockSize = 1;
    public boolean[][] walkable = new boolean[FIELDWIDTH][FIELDLENGTH];
    public boolean[][] walkableBlue = new boolean[FIELDWIDTH][FIELDLENGTH];
    public boolean[][] walkableRed = new boolean[FIELDWIDTH][FIELDLENGTH];
    public int fieldWidth;
    public int fieldLength;


    public A0Star (){
        fieldWidth = (int)(FIELDWIDTH / blockSize);
        fieldLength = (int)(FIELDLENGTH / blockSize);

        for (int y = 0; y < fieldLength; y++) {
            for (int x = 0; x < fieldWidth; x++) {
                walkable[y][x] = true;  //used for test program, not used for anything else
                if (y >= x) {
                    //map half the field as walkable
                    walkableRed[y][x] = true;
                    walkableBlue[y][x] = false;
                    //mark corner ramps as unwalkable
                    if ( y < (x + 96)) {
                        walkableRed[y][x] = true;
                    } else {
                        walkableRed[y][x] = false;
                    }

                    //mark center as unwalkable 1foot square
                    if (((x >= 66 ) && (x <= 78)) && ((y >= 66 ) && (y <= 78))) {
                        walkableRed[y][x] = false;
                        walkableBlue[y][x] = false;
                    }
                } else {
                    //map half the field as walkable
                    walkableRed[y][x] = false;
                    walkableBlue[y][x] = true;
                    //mark corner ramps as unwalkable
                    if ( x < (y + 96)) {
                        walkableBlue[y][x] = true;
                    } else {
                        walkableBlue[y][x] = false;
                    }
                    //mark center as unwalkable 1foot square
                    if (((x >= 66 ) && (x <= 78)) && ((y >= 66 ) && (y <= 78))) {
                        walkableRed[y][x] = false;
                        walkableBlue[y][x] = false;
                    }

                }
            }
        }
    }
}
