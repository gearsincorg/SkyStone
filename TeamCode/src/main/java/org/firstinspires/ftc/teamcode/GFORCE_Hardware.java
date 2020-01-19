/* Copyright (c) 2019 G-FORCE.
 * This class is used to define all the specific Hardware elements for the G-FORCE robot
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Temporary
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.List;


public class GFORCE_Hardware {
    public static enum AllianceColor {
        UNKNOWN_COLOR,
        RED,
        BLUE
    }

    public static final String TAG = "G-FORCE";

    /* Public OpMode members. */
    public AllianceColor allianceColor = AllianceColor.UNKNOWN_COLOR;
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightBackDrive = null;

    public DcMotorEx collect = null;
    public DcMotorEx lift = null;
    public DcMotorEx arm = null;

    List<LynxModule> allHubs = null;

    public Servo skystoneLiftRed = null;
    public Servo skystoneLiftBlue = null;
    public Servo stoneGrab = null;
    public Servo stoneRotate = null;
    public Servo foundationGrabberLeft = null;
    public Servo foundationGrabberRight = null;

    public static BNO055IMU imu = null;

    public final double MAX_VELOCITY        = 2500;  // Counts per second
    public final double MAX_VELOCITY_MMPS   = 2540;  // MM Per Second
    public final double AUTO_ROTATION_DPS   = 2540;   // Degrees per second

    public final double ACCELERATION_LIMIT  = 1000;  // MM per second per second  was 1524

    public final double YAW_GAIN            = 0.010;  // Rate at which we respond to heading error 0.013
    public final double LATERAL_GAIN        = 0.0025; // Distance from x axis that we start to slow down. 0027
    public final double AXIAL_GAIN          = 0.0015; // Distance from target that we start to slow down. 0017

    public final double LIFT_COUNTS_PER_DEGREE   = (2786 * 100) / (20 * 360) ;  // 20-100 gear reduction
    public final double ARM_COUNTS_PER_DEGREE    = 2786 / 360;   //

    public final double LIFT_START_ANGLE         =  10;  //
    public final double ARM_START_ANGLE          = -110; //
    public final double STONE_OPEN               = 0.5;
    public final double STONE_CLOSE              = 0;
    public final double STONE_AXIAL              = 0.51;
    public final double FOUNDATION_SAFE_R = 0.49;
    public final double FOUNDATION_SAFE_L = 0.55;
    public final double FOUNDATION_DOWN_R = 0.18;   // 0.28 for spike
    public final double FOUNDATION_DOWN_L = 0.89;   // 0.77 for spike

    // Driving constants Yaw heading
    final double HEADING_GAIN       = 0.012;  // Was 0.02
    final double TURN_RATE_TC       = 0.6;
    final double STOP_TURNRATE      = 0.020;
    final double GYRO_360_READING   = 360.0;
    final double GYRO_SCALE_FACTOR  = 360.0 / GYRO_360_READING;

    final double YAW_IS_CLOSE = 2.0;  // angle within which we are "close"

    final double AXIAL_ENCODER_COUNTS_PER_MM   = 0.8602;
    final double LATERAL_ENCODER_COUNTS_PER_MM = 0.9134;

    // Robot states that we share with others
    public double axialMotion = 0;
    public double lateralMotion = 0;
    public double armAngle = 0;
    public double liftAngle = 0;
    public double currentHeading = 0;
    public int   encoderLift;
    public int   encoderArm;

    private static LinearOpMode myOpMode = null;

    // Sensor Read Info.
    private int   encoderLB;
    private int   encoderLF;
    private int   encoderRB;
    private int   encoderRF;

    private int startLeftBack = 0;
    private int startLeftFront = 0;
    private int startRightBack = 0;
    private int startRightFront = 0;
    private int deltaLeftBack = 0;
    private int deltaLeftFront = 0;
    private int deltaRightBack = 0;
    private int deltaRightFront = 0;

    private double driveAxial = 0;
    private double driveYaw = 0;
    private double driveLateral = 0;
    private double startTime = 0;

    // gyro
    private double lastHeading = 0;
    private double lastCycle = 0;
    private double intervalCycle = 0;
    private double filteredTurnRate = 0;
    private double integratedZAxis = 0;
    private double adjustedIntegratedZAxis = 0;
    private double headingSetpoint = 0;

    private int timeoutSoundID = 0;
    public String autoPathName = "";

    /* local OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime cycleTime = new ElapsedTime();
    private ElapsedTime navTime = new ElapsedTime();

    /* Constructor */
    public GFORCE_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        leftBackDrive = configureMotor("left_back_drive", DcMotor.Direction.REVERSE);
        leftFrontDrive = configureMotor("left_front_drive", DcMotor.Direction.REVERSE);
        rightBackDrive = configureMotor("right_back_drive", DcMotor.Direction.FORWARD);
        rightFrontDrive = configureMotor("right_front_drive", DcMotor.Direction.FORWARD);

        lift = configureMotor("lift", DcMotor.Direction.FORWARD);
        arm = configureMotor("arm", DcMotor.Direction.REVERSE);
        collect = myOpMode.hardwareMap.get(DcMotorEx.class,"collect");
        collect.setDirection(DcMotor.Direction.REVERSE);

        // Set all Expansion hubs to use the MANUAL Bulk Caching mode
        allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        resetEncoders();
        stoneGrab = myOpMode.hardwareMap.get(Servo.class,"stone_grab");
        stoneRotate = myOpMode.hardwareMap.get(Servo.class,"stone_rotate");
        skystoneLiftRed = myOpMode.hardwareMap.get(Servo.class, "lift_red");
        skystoneLiftBlue = myOpMode.hardwareMap.get(Servo.class, "lift_blue");
        foundationGrabberRight = myOpMode.hardwareMap.get(Servo.class,"foundation_GR" );
        foundationGrabberLeft = myOpMode.hardwareMap.get(Servo.class,"foundation_GL" );

        setRedSkystoneGrabber(SkystoneGrabberPositions.START);
        setBlueSkystoneGrabber(SkystoneGrabberPositions.START);

        stoneGrab.setPosition(STONE_OPEN);
        stoneRotate.setPosition(STONE_AXIAL);
        foundationGrabberRight.setPosition(FOUNDATION_SAFE_R);
        foundationGrabberLeft.setPosition(FOUNDATION_SAFE_L);

        timeoutSoundID = myOpMode.hardwareMap.appContext.getResources().getIdentifier("ss_siren", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().preload(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetHeading();

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        stopRobot();
    }

    // Configure a motor
    public DcMotorEx configureMotor( String name, DcMotor.Direction direction) {
        DcMotorEx motorObj = myOpMode.hardwareMap.get(DcMotorEx.class, name);
        motorObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObj.setDirection(direction);
        motorObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motorObj;
    }

    // Autonomous driving methods
    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     *
     * @param mm
     * @param heading
     * @param vel
     * @param timeOutSec
     * @return
     */
    public boolean driveAxialVelocity(double mm, double heading, double vel, double timeOutSec, boolean hardBreak, boolean flipOnBlue) {
        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

        // Reverse axial directions for blue autonomous if flipOnBlue is true
        if ((allianceColor == AllianceColor.BLUE) && flipOnBlue) {
            vel = -vel;
        }

        // If we are moving backwards, set vel negative
        if ((mm * vel) < 0.0) {
            vel = -Math.abs(vel);
        } else {
            vel = Math.abs(vel);
        }

        RobotLog.ii(TAG, String.format("Drive-Axial adjusted mm:vel:head %5.0f:%5.0f ", mm, vel, heading));

        //Save the current position
        startMotion();

        // Loop until the robot has driven to where it needs to go
        // Remember to call updateMotion() once per loop cycle.
        while (myOpMode.opModeIsActive() && updateMotion() &&
                (Math.abs(getAxialMotion()) < absMm) &&
                (runTime.seconds() < endingTime)) {
            RobotLog.ii(TAG, String.format("Motion A:L %5.0f:%5.0f ", axialMotion, lateralMotion));
            setAxialVelocity(getProfileVelocity(vel, getAxialMotion(), absMm));
            setLateralVelocity(-lateralMotion);  // Reverse any drift
            setYawVelocityToHoldHeading(heading);
            moveRobotVelocity();
            showEncoders();
        }

        RobotLog.ii(TAG, String.format("Breaking A:L %5.0f:%5.0f ", axialMotion, lateralMotion));
        updateMotion();
        RobotLog.ii(TAG, String.format("Last A:L %5.0f:%5.0f ", axialMotion, lateralMotion));


        // Return true if we have not timed out
        return (runTime.seconds() < endingTime);
    }

    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     *
     * @param mm
     * @param heading
     * @param vel
     * @param timeOutSec
     * @return
     */
    public boolean driveLateralVelocity(double mm, double heading, double vel, double timeOutSec, boolean hardBreak, boolean flipOnBlue) {

        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

        RobotLog.ii(TAG, String.format("Drive-Lateral mm:vel:head %5.0f:%5.0f ", mm, vel, heading));

        // Reverse Lateral directions for blue autonomous and when flipOnBlue is true
        if ((allianceColor == AllianceColor.BLUE) && flipOnBlue) {
          vel = -vel;
        }

        // If we are moving backwards, set vel negative
         if ((mm * vel) < 0.0) {
            vel = -Math.abs(vel);
        } else {
            vel = Math.abs(vel);
        }

        //Save the current position
        startMotion();

        // Loop until the robot has driven to where it needs to go
        // Remember to call updateMotion() once per loop cycle.
        while (myOpMode.opModeIsActive() && updateMotion() &&
                (Math.abs(getLateralMotion()) < absMm) &&
                (runTime.seconds() < endingTime)) {
            RobotLog.ii(TAG, String.format("Motion A:L %5.0f:%5.0f ", axialMotion, lateralMotion));
            setAxialVelocity(-axialMotion);  //  Reverse any drift
            setLateralVelocity(getProfileVelocity(vel, getLateralMotion(), absMm));
            setYawVelocityToHoldHeading(heading);
            moveRobotVelocity();
            showEncoders();
        }
        stopRobot();

        if (hardBreak) {
            setLateralVelocity(-vel);
            myOpMode.sleep(100);
        }

        stopRobot();
        RobotLog.ii(TAG, String.format("Motion A:L %5.0f:%5.0f ", axialMotion, lateralMotion));

        // Return true if we have not timed out
        return (runTime.seconds() < endingTime);
    }

    //
    public double getProfileVelocity(double topVel, double dTraveled, double dGoal) {
        double profileVelocity = 0;

        // Make all distances positive
        double absdTraveled = Math.abs(dTraveled);
        double absdGoal     = Math.abs(dGoal);
        double absTopVel = Math.abs(topVel);
        double currentVel = leftBackDrive.getVelocity();

        // Treat the profile as an acceleration half and a deceleration half, based on distance traveled.
        // Determine the velocity, then just clip the requested velocity based on the requested top speed.
        // While Accelerating, V = Alimit * Time * 2
        // While Decelerating, V = ALimit * SQRT(2 * dRemaining / ALimit )

        if (absdTraveled < (absdGoal / 2.0)) {
            // We are accelerating, give a little boost
            profileVelocity = (ACCELERATION_LIMIT * getMotionTime() * 2) ;

        } else if (absdTraveled < absdGoal ) {
            // We are Decelerating
            double dRemaining = Range.clip((absdGoal - absdTraveled), 0, absdGoal); // Don't let this go negative.
            profileVelocity = ACCELERATION_LIMIT * Math.sqrt(2 * dRemaining / ACCELERATION_LIMIT);
        } else {
            // We Are Stopped
            profileVelocity = 0.0;
        }

            //Brake if velocity is too high
        if (Math.abs(currentVel) > profileVelocity) {
            profileVelocity = 0;
        }

        // Make sure the final velocity sign is correct.
        profileVelocity = Range.clip(profileVelocity, 0, absTopVel) * Math.signum(topVel);

        Log.d("G-FORCE AUTO", String.format("T:V:D:A %5.3f %4.2f %5.2f %5.2f",
                 getMotionTime(), profileVelocity, absdTraveled, currentVel / AXIAL_ENCODER_COUNTS_PER_MM));

        return (profileVelocity);
    }

    // Common location to read all sensors
    public void readSensors() {

        // Clear the BulkCache once per control cycle
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        encoderLF = leftFrontDrive.getCurrentPosition();
        encoderRF = rightFrontDrive.getCurrentPosition();
        encoderLB = leftBackDrive.getCurrentPosition();
        encoderRB = rightBackDrive.getCurrentPosition();

        encoderLift = lift.getCurrentPosition();
        encoderArm = arm.getCurrentPosition();

        liftAngle = (encoderLift / LIFT_COUNTS_PER_DEGREE) + LIFT_START_ANGLE;
        armAngle = (encoderArm / ARM_COUNTS_PER_DEGREE) + ARM_START_ANGLE;

        currentHeading = getHeading();

        intervalCycle = cycleTime.milliseconds() - lastCycle;
        lastCycle = cycleTime.milliseconds();
    }

    //Get the current encoder counts of the drive motors
    public void startMotion() {
        readSensors();
        startLeftBack = encoderLB;
        startLeftFront = encoderLF;
        startRightBack = encoderRB;
        startRightFront = encoderRF;
        deltaLeftBack = 0;
        deltaLeftFront = 0;
        deltaRightBack = 0;
        deltaRightFront = 0;
        startTime = runTime.time();
    }

    public boolean updateMotion() {
        readSensors();
        deltaLeftBack = encoderLB - startLeftBack;
        deltaLeftFront = encoderLF - startLeftFront;
        deltaRightBack = encoderRB - startRightBack;
        deltaRightFront = encoderRF - startRightFront;
        axialMotion = ((deltaLeftBack + deltaLeftFront + deltaRightBack + deltaRightFront) / 4);
        axialMotion /= AXIAL_ENCODER_COUNTS_PER_MM;
        lateralMotion = ((-deltaLeftBack + deltaLeftFront + deltaRightBack - deltaRightFront) / 4);
        lateralMotion /= LATERAL_ENCODER_COUNTS_PER_MM;

        return (true);
    }

    public double getMotionTime() {
        return(runTime.time() - startTime);
    }

    public double getAxialMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (axialMotion);
    }

    public double getLateralMotion() {
        // NOTE:  Must call updateMotion() once before calling this method;
        return (lateralMotion);
    }

    // turn with both wheels
    public boolean turnToHeading(double heading, double timeOutSEC) {
        return generalRotationControl(heading, timeOutSEC, false);
    }

    public boolean sleepAndHoldHeading(double heading, double timeOutSEC) {
        return generalRotationControl(heading, timeOutSEC, true);
    }


    public boolean generalRotationControl(double heading, double timeOutSEC, boolean waitFullTimeout) {
        boolean inPosition = false;  // needed to run at least one control cycle.
        boolean timedOut = false;


        /*
        Verify this, but heading is not reversed because robot orientation is flipped 180
        Reverse heading if alliance color is blue and flipOnBlue is true
        if ((allianceColor == AllianceColor.BLUE) && flipOnBlue) {
            heading = -heading;
            }
        */

        setHeadingSetpoint(heading);
        setAxialVelocity(0);
        setLateralVelocity(0);
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSEC) &&
                (!inPosition || waitFullTimeout)) {

            // currentHeading = getHeading();

            updateMotion();
            RobotLog.ii(TAG, String.format("sleep A:L %5.0f:%5.0f ", axialMotion, lateralMotion));

            inPosition = setYawVelocityToHoldHeading(heading);
            moveRobotVelocity();
            showEncoders();
            myOpMode.sleep(10);
        }
        stopRobot();

        if (!waitFullTimeout && (navTime.time() > timeOutSEC)) {
            timedOut = true;
            playTimoutSound();
        }

        return (!timedOut);
    }

    public void playTimoutSound() {
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().startPlaying(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }
    }

    public void resetEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void showEncoders() {
        myOpMode.telemetry.addData("Heading", "%+3.1f (%.0fmS)", adjustedIntegratedZAxis, intervalCycle);
        myOpMode.telemetry.addData("axes",  "A:L:Y %6.0f %6.0f %6.0f", driveAxial, driveLateral,driveYaw);
        myOpMode.telemetry.addData("motion","axial %6.1f, lateral %6.1f", getAxialMotion(), getLateralMotion());
        myOpMode.telemetry.addData("angles","lift: %6.1f, Arm %6.1f", liftAngle, armAngle);
        myOpMode.telemetry.update();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    // --------------------------------------------------------------------
    // Heading/Gyro Control
    // --------------------------------------------------------------------

    public double getHeading() {

        Orientation angles;
        double heading;
        double timeMs;
        double deltaH;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        if (heading != lastHeading) {
            // determine change in heading and apply to integrated Z
            deltaH = heading - lastHeading;
            if (deltaH < -180)
                deltaH += 360;
            else if (deltaH > 180)
                deltaH -= 360;

            integratedZAxis += deltaH;
            lastHeading = heading;

        }
        adjustedIntegratedZAxis = integratedZAxis * GYRO_SCALE_FACTOR;
        return (adjustedIntegratedZAxis);
    }

    public void setHeadingSetpoint(double newSetpoint) {
        headingSetpoint = newSetpoint;
    }

    public boolean setYawVelocityToHoldHeading(double newSetpoint) {

        setHeadingSetpoint(newSetpoint);
        return (setYawVelocityToHoldHeading());
    }

    public boolean setYawVelocityToHoldHeading() {
        // double error = normalizeHeading(headingSetpoint - getHeading());
        double error = normalizeHeading(headingSetpoint - currentHeading);
        double yaw = Range.clip(error * HEADING_GAIN, -0.25, 0.25);

        setYawVelocity(yaw * AUTO_ROTATION_DPS);
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

    public boolean notTurning() {

        AngularVelocity velocities;
        velocities = imu.getAngularVelocity();
        double rate = velocities.zRotationRate;

        filteredTurnRate += ((rate - filteredTurnRate) * TURN_RATE_TC);
        myOpMode.telemetry.addData("Turn Rate", "%6.3f", filteredTurnRate);

        return (Math.abs(filteredTurnRate) < STOP_TURNRATE);
    }


    // Robot movement using +/- MAX_VELOCITY

    public void setAxialVelocity(double axialV) {
        driveAxial = Range.clip(axialV * AXIAL_ENCODER_COUNTS_PER_MM, -MAX_VELOCITY, MAX_VELOCITY);
    }

    public void setYawVelocity(double yawV) {
        driveYaw = Range.clip(yawV * AXIAL_ENCODER_COUNTS_PER_MM, -MAX_VELOCITY, MAX_VELOCITY);
    }

    public void setLateralVelocity(double lateralV) {
        driveLateral = Range.clip(lateralV * LATERAL_ENCODER_COUNTS_PER_MM, -MAX_VELOCITY, MAX_VELOCITY);
    }

    public void moveRobotVelocity(double axialV, double yawV, double lateralV) {
        setAxialVelocity(axialV);
        setYawVelocity(yawV);
        setLateralVelocity(lateralV);
        moveRobotVelocity();
    }

    public void moveRobotVelocity() {
        //calculate required motor speeds
        double leftFrontVel = driveAxial - driveYaw + driveLateral;
        double leftBackVel = driveAxial - driveYaw - driveLateral;
        double rightFrontVel = driveAxial + driveYaw - driveLateral;
        double rightBackVel = driveAxial + driveYaw + driveLateral;

        double biggest = Math.max(Math.abs(leftFrontVel), Math.abs(rightFrontVel));
        biggest = Math.max(biggest, Math.abs(leftBackVel));
        biggest = Math.max(biggest, Math.abs(rightBackVel));

        if (biggest > MAX_VELOCITY) {
            double scale = MAX_VELOCITY / biggest;
            leftFrontVel *= scale;
            rightFrontVel *= scale;
            leftBackVel *= scale;
            rightBackVel *= scale;
        }

        leftFrontDrive.setVelocity(leftFrontVel);
        rightFrontDrive.setVelocity(rightFrontVel);
        leftBackDrive.setVelocity(leftBackVel);
        rightBackDrive.setVelocity(rightBackVel);

        // Log.d("G-FORCE AUTO", String.format("M %5.1f %5.1f %5.1f %5.1f ", leftFrontVel, rightFrontVel, leftBackVel, rightBackVel));

    }

    public void setAxialPower (double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void stopRobot() {
        moveRobotVelocity(0, 0, 0);
    }

    // Actuator Methods
    private final double LIFT_RED_SAFE = 0.07;
    private final double LIFT_RED_READY = 0.55;

    public void setRedSkystoneGrabber(SkystoneGrabberPositions position) {

        switch (position) {
            case START:
                skystoneLiftRed.setPosition(LIFT_RED_SAFE);
                break;

            case GRAB_DOWN:
                skystoneLiftRed.setPosition(LIFT_RED_READY);
                break;
        }
    }

    private final double LIFT_BLUE_SAFE = 0.96;
    private final double LIFT_BLUE_READY = 0.52;

    public void setBlueSkystoneGrabber(SkystoneGrabberPositions position) {

        switch (position) {
            case START:
                skystoneLiftBlue.setPosition(LIFT_BLUE_SAFE);
                break;

            case GRAB_DOWN:
                skystoneLiftBlue.setPosition(LIFT_BLUE_READY);
                break;
        }
    }

    public void setSkystoneGrabber (SkystoneGrabberPositions position) {
        switch (allianceColor) {
            case UNKNOWN_COLOR:
                break;

            case BLUE:
                setBlueSkystoneGrabber(position);
                break;

            case RED:
                setRedSkystoneGrabber(position);
                break;
        }

    }
}
