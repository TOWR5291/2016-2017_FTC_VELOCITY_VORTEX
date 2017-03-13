/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package club.towr5291.Concepts;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the Modern Robotics Gyro.
 *
 * The op mode assumes that the gyro sensor
 * is attached to a Device Interface Module I2C channel
 * and is configured with a name of "gyro".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/
@TeleOp(name = "Sensor: MR Gyro and IMU", group = "5291Concept")
//@Disabled
public class ConceptSensorMRGyroAdafruitIMU extends LinearOpMode {

  //----------------------------------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------------------------------

  // The IMU sensor object
  BNO055IMU imu;

  // State used for updating telemetry
  Orientation angles;
  Acceleration gravity;

  @Override
  public void runOpMode() {

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = true;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    // get a reference to a Modern Robotics GyroSensor object.
    gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

    // start calibrating the gyro.
    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    telemetry.update();
    gyro.calibrate();

    // make sure the gyro is calibrated.
    while (!isStopRequested() && gyro.isCalibrating())  {
      sleep(50);
      idle();
    }

    telemetry.addData(">", "Gyro Calibrated.  Press Start.");
    telemetry.update();

    // wait for the start button to be pressed.
    waitForStart();
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    while (opModeIsActive())  {

      // if the A and B buttons are pressed just now, reset Z heading.
      curResetState = (gamepad1.a && gamepad1.b);
      if(curResetState && !lastResetState)  {
        gyro.resetZAxisIntegrator();
      }
      lastResetState = curResetState;

      // get the x, y, and z values (rate of change of angle).
      xVal = gyro.rawX();
      yVal = gyro.rawY();
      zVal = gyro.rawZ();

      // get the heading info.
      // the Modern Robotics' gyro sensor keeps
      // track of the current heading for the Z axis only.
      heading = gyro.getHeading();
      angleZ  = gyro.getIntegratedZValue();

      //read imu
      angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
      double imuz = angleToHeading(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

      telemetry.addData(">", "Press A & B to reset Heading.");
      telemetry.addData("0", "GYRO Heading %03d", heading);

      telemetry.addData("1", "GYRO Int. Ang. %03d", angleZ);
      telemetry.addData("2", "GYRO X av. %03d", xVal);
      telemetry.addData("3", "GRYO Y av. %03d", yVal);
      telemetry.addData("4", "GYRO Z av. %03d", zVal);
      heading = (int)teamAngleAdjust(heading);
      telemetry.addData("5", "GYRO Adjusted %03d", heading);
      telemetry.addData("6", "IMU Z " + imuz );


      telemetry.update();
    }
  }
  private double teamAngleAdjust ( double angle ) {
    String allianceColor = "Red";

    if (allianceColor.equals("Red")) {
      //angle = angle + 90;  if starting against the wall
      //angle = angle + 225; if starting at 45 to the wall facing the beacon
      angle = angle + 225;
      if (angle > 360) {
        angle = angle - 360;
      }
    } else
    if (allianceColor.equals("Blue")) {
      angle = angle - 180;
      if (angle < 0) {
        angle = angle + 360;
      }
    }
    return angle;
  }

  //----------------------------------------------------------------------------------------------
  // Formatting
  //----------------------------------------------------------------------------------------------

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees){
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }

  private double angleToHeading(double z) {

    double angle = -z;
    if (angle < 0)
      return angle + 360;
    else if (angle > 360)
      return angle - 360;
    else
      return angle;
  }
}
