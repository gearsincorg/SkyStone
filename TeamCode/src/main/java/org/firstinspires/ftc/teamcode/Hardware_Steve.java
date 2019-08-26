

package org.firstinspires.ftc.teamcode;
import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for the G-FORCE robot.
 * All robot devices and routines common to Auto and Teleop are included here
 *
 */
public class Hardware_Steve
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  arm = null;
    public static BNO055IMU imu         = null;

    public static final double ARM_STOP       =  0 ;
    public static final double ARM_DUMP    =  -0.2 ;
    public static final double ARM_COLLECT  = 0.2 ;
    public final int COLLECT_LIMIT = -12;
    public final int AUTO_SCORE = -225;
    public final int DUMP_LIMIT = -470;

    // Driving constants Yaw heading
    final double HEADING_GAIN       = 0.012;  // Was 0.02
    final double GYRO_360_READING   = 360.0;
    final double GYRO_SCALE_FACTOR  = 360.0 / GYRO_360_READING;
    final double YAW_IS_CLOSE       = 2.0;  // angle within which we are "close"
    final double AXIAL_SCALE = 1;
    final double YAW_SCALE = 0.5;

    final double COUNTSPERDEGREE = 6.11;
    final double ENCODER_COUNTS_PER_INCH = 88.5;

    private static LinearOpMode myOpMode = null;

    private double driveAxial = 0;
    private double driveYaw = 0;

    // gyro
    private double lastHeading          = 0;
    private double lastGyroMs           = 0;
    private double intervalGyroMs       = 0;
    private double integratedZAxis      = 0;
    private double adjustedIntegratedZAxis = 0;
    private double headingSetpoint      = 0;
    private int timeoutSoundID          = 0;
    private Context myApp;
    SoundPlayer.PlaySoundParams params;


    public  double robotPitch           = 0;
    public boolean redSide = false;
    public boolean scoreBucket = false;

    /* local OpMode members. */
    private ElapsedTime runTime  = new ElapsedTime();
    private ElapsedTime     gyroTime    = new ElapsedTime();
    private ElapsedTime     navTime     = new ElapsedTime();

    /* Constructor */
    public Hardware_Steve(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;

        myOpMode.sleep(260);
        myOpMode.telemetry.addData("Status", "Initializing -- PLEASE WAIT");
        myOpMode.telemetry.update();

        // Define and Initialize Motors
        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class,"right_drive");
        arm = myOpMode.hardwareMap.get(DcMotor.class,"arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myApp = myOpMode.hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        timeoutSoundID = myApp.getResources().getIdentifier("ss_mine", "raw", myApp.getPackageName());
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().preload(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }

        // setting up Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // use RUN_USING_ENCODERS
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        arm.setPower(0);
    }

    public void runMenu() {
        // Wait for the game to start (driver presses PLAY)
        while (!myOpMode.opModeIsActive() && !myOpMode.isStopRequested())
        {
            if (myOpMode.gamepad1.x) {
                redSide = false;
            }
            if (myOpMode.gamepad1.b) {
                redSide = true;
            }
            if (myOpMode.gamepad1.y) {
                scoreBucket = false;
            }
            if (myOpMode.gamepad1.a) {
                scoreBucket = true;
            }
            myOpMode.telemetry.addData("Alliance (X/B)", redSide ? "Red" : "Blue");
            myOpMode.telemetry.addData("Score (Y/A)", scoreBucket ? "Bucket" : "Corner");
            myOpMode.telemetry.addData(">", "- - - - - - - - - - - - Press Play to Start");
            myOpMode.telemetry.update();
        }

        myOpMode.telemetry.addData("Running", scoreBucket ? "Bucket" : "Corner");
        myOpMode.telemetry.update();
    }

    // Autonomous driving methods

    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     * @param inches
     * @param heading
     * @param speed
     * @param timeOutSec
     * @return
     */
    public boolean driveStraight(double inches, double heading, double speed, double timeOutSec) {

        int desiredEncoderCounts = (int) (inches * ENCODER_COUNTS_PER_INCH);
        int startEncoderCount = leftDrive.getCurrentPosition();

        //Reverse angles for red autonomous
        if (redSide) {
            heading=-heading;
        }

        if ((inches < 0.0) || (speed < 0.0)) {
            inches = Math.abs(inches);
            speed = -Math.abs(speed);
        }

        // Loop until the robot has driven to where it needs to go
        navTime.reset();
        while (myOpMode.opModeIsActive() &&
               ((Math.abs(leftDrive.getCurrentPosition() - startEncoderCount)) < desiredEncoderCounts) &&
                (navTime.seconds() < timeOutSec)) {
            setAxial(speed);
            setYawToHoldHeading(heading);
            moveRobot();
        }
        stopRobot();

        // Check for Timeout
        if (navTime.time() > timeOutSec) {
            playTimoutSound();
        }

        // Return true if we have not timed out
        return(navTime.seconds() < timeOutSec);
    }

    // turn with both wheels
    public boolean turnToHeading(double heading, double timeOutSec) {
        // Flip translation and rotations if we are RED
        //Reverse angles for red autonomous
        if (redSide) {
            heading=-heading;
        }

        setHeadingSetpoint(heading);
        setAxial(0);

        navTime.reset();
        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSec)&&
                !setYawToHoldHeading()) {
            moveRobot();
        }

        stopRobot();

        // Check for Timeout
        if (navTime.time() > timeOutSec) {
            playTimoutSound();
        }

         return (navTime.time() < timeOutSec);
    }

    public boolean sleepAndHoldHeading(double heading, double timeOutSEC) {
        // Flip translation and rotations if we are RED
        //Reverse angles for red autonomous
        if (redSide) {
            heading=-heading;
        }

        setHeadingSetpoint(heading);
        navTime.reset();
        setAxial(0);
        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSEC)) {
            setYawToHoldHeading();
            moveRobot();
        }
        if (navTime.time() > timeOutSEC) {
            playTimoutSound();
        }
        stopRobot();
        return (navTime.time() < timeOutSEC);
    }

    public void playTimoutSound() {
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().startPlaying(myApp, timeoutSoundID, params, null, null);
        }
    }

    public void resetEncoders () {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDriveMode (DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
    }

    // The O' Great Button Click dilemma. Lest we forget this ever tricky conundrum. - Juan F. Aleman IV
    public void updateClickState () {

    }

    // --------------------------------------------------------------------
    // Heading/Gyro Control
    // --------------------------------------------------------------------
    public double getHeading() {

        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading;
        double timeMs;
        double deltaH;

        heading = angles.firstAngle;
        robotPitch = -angles.thirdAngle;

        if (heading != lastHeading) {
            // determine change in heading and apply to integrated Z
            deltaH = heading - lastHeading;
            if (deltaH < -180)
                deltaH += 360;
            else if (deltaH > 180)
                deltaH -= 360;

            integratedZAxis += deltaH;
            lastHeading = heading;

            // determine update rate for gyro
            timeMs = gyroTime.milliseconds();
            intervalGyroMs = timeMs - lastGyroMs;
            lastGyroMs = timeMs;
        }
        adjustedIntegratedZAxis = integratedZAxis * GYRO_SCALE_FACTOR;
        myOpMode.telemetry.addData("Heading", "%+3.1f (%.0fmS)", adjustedIntegratedZAxis, intervalGyroMs);
        return (adjustedIntegratedZAxis);
    }

    public void setHeadingSetpoint(double newSetpoint) {
        headingSetpoint = newSetpoint;
    }

    public boolean setYawToHoldHeading(double newSetpoint) {

        setHeadingSetpoint(newSetpoint);
        return (setYawToHoldHeading());
    }

    public boolean setYawToHoldHeading() {
        double error = normalizeHeading(headingSetpoint - getHeading());
        double yaw = Range.clip(error * HEADING_GAIN, -1.0, 1.0);

        setYaw(yaw);
        return (Math.abs(error) < YAW_IS_CLOSE);
    }

    public void setHeading(double newHeading) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // set new integrated value and save current heading for future calculations
        lastHeading = angles.firstAngle;
        integratedZAxis = newHeading;
    }

    public void resetHeading() {
        setHeading(0.0);
    }

    public double normalizeHeading(double heading) {
        while (heading <= -180) {
            heading += 360;
        }
        while (heading >= 180) {
            heading -= 360;
        }
        return heading;
    }

    public void moveRobot(double axial, double yaw) {
        setAxial(axial);
        setYaw(yaw);
        moveRobot();
    }

    public void setAxial(double axial) { driveAxial = Range.clip(axial, -1, 1);}
    public void setYaw(double yaw) {driveYaw = Range.clip(yaw, -1, 1);}

    public void moveRobot() {
        //calculate required motor speeds
        double leftSpeed = driveAxial - driveYaw;
        double rightSpeed = driveAxial + driveYaw;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
        myOpMode.telemetry.addData("Arm position", arm.getCurrentPosition());
    }

    public void stopRobot() {
        moveRobot(0,0);
    }

   // ---------------------------------  Bucket methods

    public void homeBucket() {
        myOpMode.telemetry.addData(">", "Homing Bucket");
        myOpMode.telemetry.update();

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0.2);
        myOpMode.sleep(1000);
        arm.setPower(0);
        myOpMode.sleep(1000);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move the arm until it reaches the dump limit
    public void dumpRings (double timeOutSec) {
        myOpMode.telemetry.addData(">", "Dumping");
        myOpMode.telemetry.update();
        arm.setPower(ARM_DUMP);
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                (arm.getCurrentPosition() > DUMP_LIMIT) && (navTime.time() < timeOutSec)) {

        }
        arm.setPower(0);
    }

    // Move the arm to prepare to dump the rings in the bucket
    public void prepareRings (double timeOutSec) {
        arm.setPower(ARM_DUMP);
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                (arm.getCurrentPosition() > AUTO_SCORE) && (navTime.time() < timeOutSec)) {
        }
        arm.setPower(0);
    }

    // Move the arm up after dumping the rings in the bucket
    public void releaseRings (double timeOutSec) {
        arm.setPower(-ARM_DUMP);
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                (arm.getCurrentPosition() < AUTO_SCORE) && (navTime.time() < timeOutSec)) {
        }
        arm.setPower(0);
    }
}

