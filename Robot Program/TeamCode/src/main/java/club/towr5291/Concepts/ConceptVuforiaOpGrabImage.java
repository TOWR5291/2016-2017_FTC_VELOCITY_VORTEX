package club.towr5291.Concepts;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs; // imread, imwrite, etc
import org.opencv.imgproc.Moments;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import club.towr5291.functions.Constants;
import club.towr5291.functions.BeaconAnalysisOCV;
import club.towr5291.functions.FileLogger;
import club.towr5291.opmodes.R;

import static org.opencv.imgproc.Imgproc.contourArea;


/**
 * Created by ianhaden on 4/10/2016.
 */

@Autonomous(name="Concept Vuforia Grab Image", group="5291Test")
public class ConceptVuforiaOpGrabImage extends LinearOpMode{
    OpenGLMatrix lastLocation = null;
    private double robotX;
    private double robotY;
    private double robotBearing;

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    //set up openCV stuff


    private Point redpoint = new Point(0,0);
    private Point bluepoint = new Point(0,0);

    private double contourarea;
    private double ContourAreaLast;
    private double redlength;
    private double bluelength;
    private double directionOfBeacon;
    private boolean beaconLeft;
    private double beaconLeftXPos;


    protected ImageView grabbedImage;


    @Override
    public void runOpMode() throws InterruptedException {

        //start the log
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        BeaconAnalysisOCV beaconColour = new BeaconAnalysisOCV();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AVATY7T/////AAAAGQJxfNYzLUgGjSx0aOEU0Q0rpcfZO2h2sY1MhUZUr+Bu6RgoUMUP/nERGmD87ybv1/lM2LBFDxcBGRHkXvxtkHel4XEUCsNHFTGWYcVkMIZqctQsIrTe13MnUvSOfQj8ig7xw3iULcwDpY+xAftW61dKTJ0IAOCxx2F0QjJWqRJBxrEUR/DfQi4LyrgnciNMXCiZ8KFyBdC63XMYkQj2joTN579+2u5f8aSCe8jkAFnBLcB1slyaU9lhnlTEMcFjwrLBcWoYIFAZluvFT0LpqZRlS1/XYf45QBSJztFKHIsj1rbCgotAE36novnAQBs74ewnWsJifokJGOYWdFJveWzn3GE9OEH23Y5l7kFDu4wc";
        //parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);                                          //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1);                                                           //tells VuforiaLocalizer to only store one frame at a time
        //ConceptVuforiaGrabImage vuforia = new ConceptVuforiaGrabImage(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables velocityVortex = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels = velocityVortex.get(0);
        wheels.setName("wheels");  // wheels target

        VuforiaTrackable tools  = velocityVortex.get(1);
        tools.setName("tools");  // tools target

        VuforiaTrackable legos = velocityVortex.get(2);
        legos.setName("legos");  // legos target

        VuforiaTrackable gears  = velocityVortex.get(3);
        gears.setName("gears");  // gears target

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(velocityVortex);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Wheels Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         *
         */

        // RED Targets
        // To Place GEARS Target
        // position is approximately - (-6feet, -1feet)

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -1 * 12 * mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        // To Place GEARS Target
        // position is approximately - (-6feet, 3feet)
        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFTCFieldWidth/2, 3 * 12 * mmPerInch, 0)
                //.translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        //Finsih RED Targets

        // BLUE Targets
        // To Place LEGOS Target
        // position is approximately - (-3feet, 6feet)

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-3 * 12 * mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(legosTargetLocationOnField));

        // To Place WHEELS Target
        // position is approximately - (1feet, 6feet)
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(1 * 12 * mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(wheelsTargetLocationOnField));

        //Finsih BLUE Targets

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation((mmBotWidth/2), 50,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        //set up openCV stuff
        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar(0,100,150);
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar(22,255,255);  //was 30,255,255

        //these work pretty good
        Scalar RED_LOWER_BOUNDS_HSV = new Scalar ((int) (300.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
        Scalar RED_UPPER_BOUNDS_HSV = new Scalar ((int) (400.0 / 360.0 * 255.0), 255, 255);

        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar ((int) (300.0 / 360.0 * 255.0), (int) (0.09 * 255.0), (int) (0.500 * 255.0));
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar (255, 255, 255);

//        Scalar RED_LOWER_BOUNDS_HSV = new Scalar(0, 160, 160);
//        Scalar RED_UPPER_BOUNDS_HSV = new Scalar(180,255,255);

        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar(240, 229, 127);
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar(30,255,255);

        //Scalar RED_LOWER_BOUNDS_HSV = new Scalar(30,100,100);  //get nothing with this set
        //Scalar RED_UPPER_BOUNDS_HSV = new Scalar(120,100,255);

//        Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar(150,100,100);
//        Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar(270,255,255);

        //get really good blue with this set
        Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar((int) (170.0 / 360.0 * 255.0), (int) (0.090 * 255.0), (int) (0.500 * 255.0));
        Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar((int) (270.0 / 360.0 * 255.0), 255, 255);

        //Scalar BLUE_LOWER_BOUNDS_HSV = new Scalar(120, 230, 127);  //Get nothing with this set, got no idea why
        //Scalar BLUE_UPPER_BOUNDS_HSV = new Scalar(192, 255, 255);


        Mat mat1 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat2 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat3 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat4 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat5 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat6 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat7 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat8 = new Mat(720,1280, CvType.CV_8UC4);
        Mat mat9 = new Mat(720,1280, CvType.CV_8UC4);



        Mat houghlines = new Mat();
        Mat lines = new Mat();
        Mat mHierarchy = new Mat();

        //MatOfPoint2f approxCurve = new MatOfPoint2f();
        waitForStart();

        //Mat tmp = new Mat();

        velocityVortex.activate();

        Image rgb = null;

        int loop = 0;

        while (opModeIsActive()) {
            List<MatOfPoint> contoursRed  = new ArrayList<MatOfPoint>();
            List<MatOfPoint> contoursBlue = new ArrayList<MatOfPoint>();

            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++)
            {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
                {
                    rgb = frame.getImage(i);
                    break;
                }
            }

            /*rgb is now the Image object that we’ve used in the video*/
            Log.d("OPENCV","Height " + rgb.getHeight() + " Width " + rgb.getWidth());

            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());
            Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, tmp);

            Constants.BeaconColours Colour = beaconColour.beaconAnalysisOCV(tmp, loop);

            Log.d("OPENCV","Returned " + Colour);
/*
            SaveImage(tmp, "-raw");
            fileLogger.writeEvent("process()","Saved original file ");
            Log.d("OPENCV","tmp CV_8UC4 Height " + tmp.height() + " Width " + tmp.width());
            Log.d("OPENCV","Channels " + tmp.channels());

            tmp.convertTo(mat1, CvType.CV_8UC4);
            SaveImage(mat1, "-convertcv_8uc4");
            Log.d("OPENCV","mat1 CV_8UC4 Height " + mat1.height() + " Width " + mat1.width());
            fileLogger.writeEvent("process()","converted to cv_8uc3");
            Log.d("OPENCV","mat1 convertcv_8uc4 Channels " + mat1.channels());

            Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_RGB2HSV_FULL);
            //mat2.convertTo(mat2, CvType.CV_8UC4);
            SaveImage(mat2, "-COLOR_RGB2HSV_FULL");
            Log.d("OPENCV","mat2 COLOR_RGB2HSV Height " + mat2.height() + " Width " + mat2.width());
            Log.d("OPENCV","mat2 Channels " + mat2.channels());

            Imgproc.cvtColor(tmp, mat6, Imgproc.COLOR_RGB2YCrCb);
            SaveImage(mat6, "-COLOR_RGB2YCrCb");
            Log.d("OPENCV","mat6 COLOR_RGB2HSV Height " + mat6.height() + " Width " + mat6.width());
            Log.d("OPENCV","mat6 Channels " + mat6.channels());

            Core.inRange(mat2, RED_LOWER_BOUNDS_HSV, RED_UPPER_BOUNDS_HSV, mat3);
            Log.d("OPENCV","mat2 Channels " + mat2.channels() + " empty " + mat2.empty());
            Log.d("OPENCV","mat3 Channels " + mat3.channels() + " empty " + mat3.empty());
            Log.d("OPENCV","mat3 COLOR_RGB2HSV Height " + mat3.height() + " Width " + mat3.width());
            //Core.inRange(mat2, new Scalar(0,100,150), new Scalar(22,255,255), mat3);
            fileLogger.writeEvent("process()","Set Red window Limits: ");


            SaveImage(mat3, "-red limits");

            Imgproc.dilate(mat3, mat7, new Mat());
            Imgproc.dilate(mat7, mat8, new Mat()); //fill in holes
            Imgproc.Canny(mat8, mat9, 20, 100);
            Imgproc.findContours(mat9, contoursRed, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(mat9, contoursRed, -1, new Scalar(255,255,255), 4);


            Point centroid = massCenterMatOfPoint2f(contoursRed.get(0));
            Log.d("OPENCV","Centroid " + centroid);
            Imgproc.circle(mat9, centroid, 50, new Scalar(255, 255, 255), 5);
            Imgproc.putText(mat9,"RED",centroid, 3, 0.5, new Scalar(255, 255, 255), 1);
            MatOfPoint line = contoursRed.get(0);
            double area = contourArea(contoursRed.get(0));
            Log.d("OPENCV","Area " + area);
            SaveImage(mat9, "-red contours");

            Core.inRange(mat2, BLUE_LOWER_BOUNDS_HSV, BLUE_UPPER_BOUNDS_HSV, mat4);
            fileLogger.writeEvent("process()","Set Blue window Limits: ");
            Log.d("OPENCV","mat4 COLOR_RGB2HSV Height " + mat4.height() + " Width " + mat4.width());
            SaveImage(mat4, "-blue limits");

            //Log.d("OPENCV","mat1 Channels " + mat1.channels() + " Height " + mat1.height() + " Width " + mat1.width());
            //Log.d("OPENCV","mat2 Channels " + mat2.channels() + " Height " + mat2.height() + " Width " + mat2.width());

            Log.d("OPENCV","mat5 Channels " + mat5.channels() + " Height " + mat5.height() + " Width " + mat5.width());
            Core.bitwise_or(mat3, mat4, mat5);
            SaveImage(mat5, "-bitwise red and blue images");

            // convert to bitmap:
            Bitmap bmDisplay = Bitmap.createBitmap(mat5.cols(), mat5.rows(),Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat5, bmDisplay);
            */
            frame.close();


            for (VuforiaTrackable beac : velocityVortex) {

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getRawPose();

                if (pose != null) {

                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);

                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,92,0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,92,0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,-92,0));
                    Vec2F lowerright = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,-92,0));

                }

            }
            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                VectorF trans = lastLocation.getTranslation();
                Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;
                if (robotBearing < 0)
                {
                    robotBearing = 360 + robotBearing;
                }

                telemetry.addData("Pos X ", robotX);
                telemetry.addData("Pos Y ", robotY);
                telemetry.addData("Bear  ", robotBearing);
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos   ", format(lastLocation));

                telemetry.addData("Text ", "*** Vision Data***");
                //telemetry.addData("Red  ", "Red :  " + redpoint);
                //telemetry.addData("Blue ", "Blue:  " + bluepoint);
                //telemetry.addData("Dir  ", "Direction:  " + directionOfBeacon);
            } else {
                telemetry.addData("Pos   ", "Unknown");
            }
            telemetry.update();
            loop++;
        }

        //stop the log
        if (fileLogger != null)
        {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }

    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public void SaveImage (Mat mat, String info) {
        Mat mIntermediateMat = new Mat();
        Mat mIntermediateMat2 = new Mat();

        mat.convertTo(mIntermediateMat2, CvType.CV_8UC4);
        if (mIntermediateMat2.channels() > 2)
            Imgproc.cvtColor(mIntermediateMat2, mIntermediateMat, Imgproc.COLOR_RGBA2BGR, 3);
        else
            mIntermediateMat = mIntermediateMat2;


        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = "ian" + info + ".png";
        File file = new File(path, filename);

        Boolean bool = null;
        filename = file.toString();
        bool = Imgcodecs.imwrite(filename, mIntermediateMat);

        if (bool == true)
            Log.d("filesave", "SUCCESS writing image to external storage");
        else
            Log.d("filesave", "Fail writing image to external storage");
    }

    public Mat loadImageFromFile(String fileName) {

        Mat rgbLoadedImage = null;

        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File file = new File(path, fileName);

        // this should be in BGR format according to the
        // documentation.
        Mat image = Imgcodecs.imread(file.getAbsolutePath());

        if (image.width() > 0) {

            rgbLoadedImage = new Mat(image.size(), image.type());
            Imgproc.cvtColor(image, rgbLoadedImage, Imgproc.COLOR_RGB2HSV_FULL);

            Log.d("OpenCVLoadImage", "loadedImage: " + "chans: " + image.channels() + ", (" + image.width() + ", " + image.height() + ")");

            image.release();
            image = null;
        }

        return rgbLoadedImage;

    }

    private Point massCenterMatOfPoint2f(MatOfPoint map)
    {
        Moments moments = Imgproc.moments(map, true);
        Point centroid = new Point();
        centroid.x = moments.get_m10() / moments.get_m00();
        centroid.y = moments.get_m01() / moments.get_m00();
        return centroid;
    }
}
