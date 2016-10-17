package club.towr5291.opmodes;

/**
 * Created by ianhaden on 2/09/16.
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */

// Michael was here//
public class LibraryStateSegAuto {

    public double mRobotTimeOut;        //how much time is allowed for the step to complete
    public String mRobotCommand;        //how far to move in inches
    public double mRobotParm1;          //how far to move in inches
    public double mRobotParm2;        //how far to move in inches
    public double mRobotParm3;        //how far to move in inches
    public double mRobotSpeed;          //what angle to move in

    // Constructor
    public LibraryStateSegAuto(double timeout, String RobotCommand, double RobotParm1, double RobotParm2, double RobotParm3, double robotSpeed)
    {
        mRobotTimeOut = timeout;
        mRobotCommand = RobotCommand;
        mRobotParm1 = RobotParm1;
        mRobotParm2 = RobotParm2;
        mRobotParm3 = RobotParm3;
        mRobotSpeed = robotSpeed;
    }

    public void setmRobotTimeOut(double mRobotTimeOut) {
        this.mRobotTimeOut = mRobotTimeOut;
    }

    public void setmRobotCommand(String mRobotCommand) {
        this.mRobotCommand = mRobotCommand;
    }

    public void setmRobotParm1(double mRobotParm1) {
        this.mRobotParm1 = mRobotParm1;
    }

    public void setmRobotParm2(double mRobotParm2) {
        this.mRobotParm2 = mRobotParm2;
    }

    public void setmRobotParm3(double mRobotParm3) {
        this.mRobotParm3 = mRobotParm3;
    }

    public void setmRobotSpeed(double mRobotSpeed) {
        this.mRobotSpeed = mRobotSpeed;
    }

    public double getmRobotTimeOut() {
        return mRobotTimeOut;
    }

    public String getmRobotCommand() {
        return mRobotCommand;
    }

    public double getmRobotParm1() {
        return mRobotParm1;
    }

    public double getmRobotParm2() {
        return mRobotParm2;
    }

    public double getmRobotParm3() {
        return mRobotParm3;
    }

    public double getmRobotSpeed() {
        return mRobotSpeed;
    }
}
