package club.towr5291.opencvutils;

import org.opencv.core.Mat;

/**
 * Created by ianhaden on 24/10/2016.
 */

public interface OCVCallbackInterface {
    public Mat process(Mat mRgba);
}
