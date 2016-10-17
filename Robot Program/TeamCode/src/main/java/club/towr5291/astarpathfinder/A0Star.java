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

        for (int i = 0; i < fieldWidth; i++) {
            for (int j = 0; j < fieldLength; j++) {
                walkable[i][j] = true;  //used for test program, not used for anything else
                if (j > i) {
                    //map half the field as walkable
                    walkableRed[i][j] = false;
                    walkableBlue[i][j] = true;
                    //mark corner ramps as unwalkable
                    if ( i < (j + 96)) {
                        walkableBlue[i][j] = true;
                    } else {
                        walkableBlue[i][j] = false;
                    }
                    //mark center as unwalkable 1foot square
                    if (((j >= 66 ) && (j <= 78)) && ((i >= 66 ) && (i <= 78))) {
                        walkableBlue[i][j] = false;
                    }
                } else {
                    //map half the field as walkable
                    walkableRed[i][j] = true;
                    walkableBlue[i][j] = false;
                    //mark corner ramps as unwalkable
                    if ( i < (j + 96)) {
                        walkableRed[i][j] = true;
                    } else {
                        walkableRed[i][j] = false;
                    }

                    //mark center as unwalkable 1foot square
                    if (((j >= 66 ) && (j <= 78)) && ((i >= 66 ) && (i <= 78))) {
                        walkableRed[i][j] = false;
                    }
                }
            }
        }
    }
}
