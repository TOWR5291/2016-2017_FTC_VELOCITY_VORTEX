package club.towr5291.functions;

/**
 * Created by LZTDD0 on 11/7/2016.
 */

public abstract class Constants {
    //BEACON
    public static final double BEACON_WIDTH = 21.8;     //entire beacon width
    public static final double BEACON_HEIGHT = 14.5;    //entire beacon height
    public static final double BEACON_WH_RATIO = BEACON_WIDTH / BEACON_HEIGHT; //entire beacon ratio

    public enum BeaconColours {
        BEACON_RED,
        BEACON_BLUE,
        BEACON_RED_BLUE,
        BEACON_BLUE_RED,
        BEACON_RED_LEFT,
        BEACON_RED_RIGHT,
        BEACON_BLUE_LEFT,
        BEACON_BLUE_RIGHT,
        UNKNOWN;

        public String toString() {
            switch (this) {
                case BEACON_RED:
                    return "RED";
                case BEACON_BLUE:
                    return "BLUE";
                case BEACON_RED_BLUE:
                    return "RED_BLUE";
                case BEACON_BLUE_RED:
                    return "BLUE_RED";
                case UNKNOWN:
                default:
                    return "???";
            }
        }

    }
}
