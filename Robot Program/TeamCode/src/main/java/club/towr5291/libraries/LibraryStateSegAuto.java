package club.towr5291.libraries;

/**
 * Created by ianhaden on 2/09/16.
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */

public class LibraryStateSegAuto {

    private int mStep;                   //step number
    private double mRobotTimeOut;        //how much time is allowed for the step to complete
    private String mRobotCommand;        //Command
    private double mRobotParm1;          //1st Parameter
    private double mRobotParm2;          //2nd Parameter
    private double mRobotParm3;          //3rd Parameter
    private double mRobotParm4;          //4th Parameter
    private double mRobotParm5;          //5rd Parameter
    private double mRobotParm6;          //6th Parameter
    private double mRobotSpeed;          //what angle to move in
    private boolean mRobotStepComplete;  //status of the step

    // Constructor
    public LibraryStateSegAuto(int step, double timeout, String RobotCommand, double RobotParm1, double RobotParm2, double RobotParm3, double RobotParm4, double RobotParm5, double RobotParm6, double robotSpeed, boolean complete)
    {
        mStep = step;
        mRobotTimeOut = timeout;
        mRobotCommand = RobotCommand;
        mRobotParm1 = RobotParm1;
        mRobotParm2 = RobotParm2;
        mRobotParm3 = RobotParm3;
        mRobotParm4 = RobotParm4;
        mRobotParm5 = RobotParm5;
        mRobotParm6 = RobotParm6;
        mRobotSpeed = robotSpeed;
        mRobotStepComplete = complete;
    }

    public void setmRobotTimeOut(double mRobotTimeOut)
    {
        this.mRobotTimeOut = mRobotTimeOut;
    }

    public void setmRobotCommand(String mRobotCommand)
    {
        this.mRobotCommand = mRobotCommand;
    }

    public void setmRobotParm1(double mRobotParm1)
    {
        this.mRobotParm1 = mRobotParm1;
    }

    public void setmRobotParm2(double mRobotParm2)
    {
        this.mRobotParm2 = mRobotParm2;
    }

    public void setmRobotParm3(double mRobotParm3)
    {
        this.mRobotParm3 = mRobotParm3;
    }

    public void setmRobotParm4(double mRobotParm4)
    {
        this.mRobotParm3 = mRobotParm4;
    }

    public void setmRobotParm5(double mRobotParm3)
    {
        this.mRobotParm3 = mRobotParm5;
    }

    public void setmRobotParm6(double mRobotParm4)
    {
        this.mRobotParm3 = mRobotParm6;
    }

    public void setmRobotSpeed(double mRobotSpeed)
    {
        this.mRobotSpeed = mRobotSpeed;
    }

    public void setmRobotStepComplete(boolean mRobotStepComplete)
    {
        this.mRobotStepComplete = mRobotStepComplete;
    }

    public double getmRobotTimeOut()
    {
        return mRobotTimeOut;
    }

    public String getmRobotCommand()
    {
        return mRobotCommand;
    }

    public double getmRobotParm1()
    {
        return mRobotParm1;
    }

    public double getmRobotParm2()
    {
        return mRobotParm2;
    }

    public double getmRobotParm3()
    {
        return mRobotParm3;
    }

    public double getmRobotParm4()
    {
        return mRobotParm4;
    }

    public double getmRobotParm5()
    {
        return mRobotParm5;
    }

    public double getmRobotParm6()
    {
        return mRobotParm6;
    }

    public double getmRobotSpeed()
    {
        return mRobotSpeed;
    }

    public boolean getmRobotStepComplete()
    {
        return mRobotStepComplete;
    }
}
