package club.towr5291.astarpathfinder;


/**
 * Created by ianhaden on 7/09/16.
 */
public class A0Star {

    public static final int FIELDWIDTH = 144;
    public static final int FIELDLENGTH = 144;
    public int blockSize = 1;
    public boolean[][] walkable = new boolean[FIELDWIDTH][FIELDLENGTH];
    public int fieldWidth;
    public int fieldLength;

    public A0Star (){

        fieldWidth = (int)(FIELDWIDTH / blockSize);
        fieldLength = (int)(FIELDLENGTH / blockSize);

        for (int i = 0; i < fieldWidth; i++) {
            for (int j = 0; j < fieldLength; j++) {
                walkable[i][j] = true;
            }
        }
    }



}
