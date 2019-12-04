/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

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

    private ExpansionHubMotor leftFrontDriveEH = null;
    private ExpansionHubMotor rightFrontDriveEH = null;
    private ExpansionHubMotor leftBackDriveEH = null;
    private ExpansionHubMotor rightBackDriveEH = null;
    private ExpansionHubMotor liftEH = null;
    private ExpansionHubMotor armEH = null;

    public Servo skystoneLiftRed = null;
    public Servo skystoneLiftBlue = null;
    public Servo stoneGrab = null;
    public Servo stoneRotate = null;
    public Servo foundationGrabberLeft = null;
    public Servo foundationGrabberRight = null;

    public static BNO055IMU imu = null;

    public final double MAX_VELOCITY        = 2500;  // Counts per second
    public final double MAX_VELOCITY_MMPS   = 2540;  // MM Per Second
    public final double MAX_ROTATION_DPS    = 100;   // Degrees per second
    public final double AUTO_ROTATION_DPS   = 2540;   // Degrees per second
    public final double ACCELERATION_LIMIT  = 1524;  // MM per second per second

    public final double YAW_GAIN            = 0.010;  // Rate at which we respond to heading error 0.013
    public final double LATERAL_GAIN        = 0.0025; // Distance from x axis that we start to slow down. 0027
    public final double AXIAL_GAIN          = 0.0015; // Distance from target that we start to slow down. 0017

    public final double LIFT_COUNTS_PER_DEGREE   = (2786 * 100) / (20 * 360) ;  // 60-100 gear reduction
    public final double ARM_COUNTS_PER_DEGREE    = 2786 / 360;   //

    public final double LIFT_START_ANGLE         =  10;  // 60-100 gear reduction
    public final double ARM_START_ANGLE          = -110;  //
    public final double STONE_OPEN               = 0.5;
    public final double STONE_CLOSE              = 0;
    public final double STONE_AXIAL              = 0.51;
    public final double FOUNDATION_SAFE_R = 0.5;
    public final double FOUNDATION_SAFE_L = 0.5;
    public final double FOUNDATION_DOWN_R = 0.18;
    public final double FOUNDATION_DOWN_L = 0.89;

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
    ExpansionHubEx masterHub = null;
    ExpansionHubEx slaveHub = null;
    RevBulkData    masterHubValues = null;
    RevBulkData    slaveHubValues = null;

    private int   encoderLB;
    private int   encoderLF;
    private int   encoderRB;
    private int   encoderRF;

    private double driveAxial = 0;
    private double driveYaw = 0;
    private double driveLateral = 0;

    private double startTime = 0;
    private double startLeftBack = 0;
    private double startLeftFront = 0;
    private double startRightBack = 0;
    private double startRightFront = 0;
    private double deltaLeftBack = 0;
    private double deltaLeftFront = 0;
    private double deltaRightBack = 0;
    private double deltaRightFront = 0;


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

        // Temp Speed Up Code
        masterHub = myOpMode.hardwareMap.get(ExpansionHubEx.class, "Parent");
        slaveHub = myOpMode.hardwareMap.get(ExpansionHubEx.class, "Child");

        leftFrontDriveEH  = (ExpansionHubMotor)leftFrontDrive;
        rightFrontDriveEH  = (ExpansionHubMotor) rightFrontDrive;
        leftBackDriveEH  = (ExpansionHubMotor) leftBackDrive;
        rightBackDriveEH  = (ExpansionHubMotor) rightBackDrive;
        liftEH = (ExpansionHubMotor)lift;
        armEH = (ExpansionHubMotor)arm;


        resetEncoders();
        stoneGrab = myOpMode.hardwareMap.get(Servo.class,"stone_grab");
        stoneRotate = myOpMode.hardwareMap.get(Servo.class,"stone_rotate");
        skystoneLiftRed = myOpMode.hardwareMap.get(Servo.class, "lift_red");
        skystoneLiftBlue = myOpMode.hardwareMap.get(Servo.class, "lift_blue");
        foundationGrabberRight = myOpMode.hardwareMap.get(Servo.class,"foundation_GR" +
                "");
        foundationGrabberLeft = myOpMode.hardwareMap.get(Servo.class,"foundation_GL" +
                "");
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

        // setting up Gyro
        if (imu == null) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            resetHeading();
        }

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        stopRobot();
    }

    // Configure a motor
    public DcMotorEx configureMotor( String name, DcMotor.Direction direction) {
        PIDFCoefficients newPIDF = new PIDFCoefficients(10.0,  3,   0.0,  12);
        DcMotorEx motorObj = myOpMode.hardwareMap.get(DcMotorEx.class, name);

        motorObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObj.setDirection(direction);
        motorObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorObj.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
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
    public boolean driveAxialVelocity(double mm, double heading, double vel, double timeOutSec, boolean hardBreak) {
        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

        // Log what we are doing
        RobotLog.ii(TAG, String.format("Drive-Lateral mm:vel:head %5.0f:%5.0f ", mm, vel, heading));

        //Reverse angles for red autonomous
        if (allianceColor == AllianceColor.BLUE) {
            heading = -heading;
            vel = -vel;
        }

        // If we are moving backwars, set vel negative
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
        stopRobot();

        if (hardBreak) {
            setAxialVelocity(-vel);
            myOpMode.sleep(100);
        }

        stopRobot();
        RobotLog.ii(TAG, String.format("Motion A:L %5.0f:%5.0f ", axialMotion, lateralMotion));

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
    public boolean driveLateralVelocity(double mm, double heading, double vel, double timeOutSec, boolean hardBreak) {

        double endingTime = runTime.seconds() + timeOutSec;
        double absMm = Math.abs(mm);

        RobotLog.ii(TAG, String.format("Drive-Lateral mm:vel:head %5.0f:%5.0f ", mm, vel, heading));

        //Reverse angles for red autonomous
        if (allianceColor == AllianceColor.BLUE) {
            heading = -heading;
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

        // Treat the profile as an acceleration half and a deceleration half, based on distance traveled.
        // Determine the velocity, then just clip the requested velocity based on the requested top speed.

        // While Accelerating, V = Alimit * Time
        // While Decelerating, V = ALimit * SQRT(2 * dRemaining / ALimit )

        if (absdTraveled < (absdGoal / 2.0)) {
            // We are accelerating, give a little boost
            profileVelocity = (ACCELERATION_LIMIT * getMotionTime()) ;

        } else if (absdTraveled < absdGoal ) {
            // We are Decelerating
            double dRemaining = Range.clip((absdGoal - absdTraveled), 0, absdGoal); // Don't let this go negative.
            profileVelocity = ACCELERATION_LIMIT * Math.sqrt(2 * dRemaining / ACCELERATION_LIMIT);
        } else {
            // We Are Stopped
            profileVelocity = 0.0;
        }

        // Make sure the final velocity sign is correct.
        profileVelocity = Range.clip(profileVelocity, 0, absTopVel) * Math.signum(topVel);

        //Log.d("G-FORCE AUTO", String.format("T:V:D:A %5.3f %4.2f %5.2f %5.2f",
        //         getMotionTime(), profileVelocity, absdTraveled, leftBackDrive.getVelocity() / AXIAL_ENCODER_COUNTS_PER_MM));

        return (profileVelocity);
    }

    // Common location to read all sensors
    public void readSensors() {
        /*
        encoderLF = leftFrontDrive.getCurrentPosition();
        encoderRF = rightFrontDrive.getCurrentPosition();
        encoderLB = leftBackDrive.getCurrentPosition();
        encoderRB = rightBackDrive.getCurrentPosition();

        encoderLift = lift.getCurrentPosition();
        encoderArm = arm.getCurrentPosition();
        */

        RevBulkData holding;
        if ((masterHub != null) && (slaveHub != null)) {
            // ensure that getBulkinputData returns valid data ( comms loss can reutrn null)
            if ((holding = masterHub.getBulkInputData()) != null)
                masterHubValues = holding;
            if ((holding = slaveHub.getBulkInputData()) != null)
                slaveHubValues = holding;

            encoderLF = masterHubValues.getMotorCurrentPosition(leftFrontDriveEH);
            encoderLB = masterHubValues.getMotorCurrentPosition(leftBackDriveEH);
            encoderRF = masterHubValues.getMotorCurrentPosition(rightFrontDriveEH);
            encoderRB = masterHubValues.getMotorCurrentPosition(rightBackDriveEH);

            encoderLift = slaveHubValues.getMotorCurrentPosition(liftEH);
            encoderArm = slaveHubValues.getMotorCurrentPosition(armEH);
        }


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

        // Flip rotations if we are RED
        //Reverse angles for red autonomous
        if (allianceColor == AllianceColor.BLUE) {
            heading = -heading;
        }

        setHeadingSetpoint(heading);
        setAxialVelocity(0);
        setLateralVelocity(0);
        navTime.reset();

        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSEC) &&
                (!inPosition || waitFullTimeout)) {
            currentHeading = getHeading();
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


    /*
     * Temporarity comment out non-velocity code.
     * Eliminate this completely once not needed.
    // Robot movement using +/- 1.0 Range
    public void moveRobot(double axial, double yaw, double lateral) {
        setAxial(axial);
        setYaw(yaw);
        setLateral(lateral);
        moveRobot();
    }

    public void setAxial(double axial) {
        driveAxial = Range.clip(axial, -1, 1);
    }

    public void setYaw(double yaw) {
        driveYaw = Range.clip(yaw, -1, 1);
    }

    public void setLateral(double lateral) {
        driveLateral = Range.clip(lateral, -1, 1);
    }

    public void moveRobot() {
        //calculate required motor speeds
        double leftFrontSpeed = driveAxial - driveYaw + driveLateral;
        double leftBackSpeed = driveAxial - driveYaw - driveLateral;
        double rightFrontSpeed = driveAxial + driveYaw - driveLateral;
        double rightBackSpeed = driveAxial + driveYaw + driveLateral;

        double biggest = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
        biggest = Math.max(biggest, Math.abs(leftBackSpeed));
        biggest = Math.max(biggest, Math.abs(rightBackSpeed));

        if (biggest > 1.0) {
            leftFrontSpeed /= biggest;
            rightFrontSpeed /= biggest;
            leftBackSpeed /= biggest;
            rightBackSpeed /= biggest;
        }

        leftFrontDrive.setPower(leftFrontSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);
        leftBackDrive.setPower(leftBackSpeed);
        rightBackDrive.setPower(rightBackSpeed);

        //myOpMode.telemetry.addData("Arm position", arm.getCurrentPosition());
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

     */


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
        //myOpMode.telemetry.addData("Arm position", arm.getCurrentPosition());

        // Log.d("G-FORCE AUTO", String.format("M %5.1f %5.1f %5.1f %5.1f ", leftFrontVel, rightFrontVel, leftBackVel, rightBackVel));

    }

    public void stopRobot() {
        moveRobotVelocity(0, 0, 0);
    }

    // Actuator Methods
    private final double GRAB_LEFT_SAFE = 0;
    private final double GRAB_LEFT_CLOSE = 1;

    private final double LIFT_RED_SAFE = 0.07;
    private final double LIFT_RED_READY = 0.55;

    public void setRedSkystoneGrabber(SkystoneGrabberPositions position) {

        switch (position) {
            case START:
                //skystoneGrabRed.setPosition(GRAB_LEFT_SAFE);
                skystoneLiftRed.setPosition(LIFT_RED_SAFE);
                break;

            case GRAB_DOWN:
                //skystoneGrabRed.setPosition(GRAB_LEFT_CLOSE);
                skystoneLiftRed.setPosition(LIFT_RED_READY);
                break;
        }
    }

    private final double GRAB_RIGHT_SAFE = 0.55;
    private final double GRAB_RIGHT_CLOSE = 1;

    private final double LIFT_BLUE_SAFE = 0.96;
    private final double LIFT_BLUE_READY = 0.52;


    public void setBlueSkystoneGrabber(SkystoneGrabberPositions position) {

        switch (position) {
            case START:
                //skystoneGrabBlue.setPosition(GRAB_RIGHT_SAFE);
                skystoneLiftBlue.setPosition(LIFT_BLUE_SAFE);
                break;


            case GRAB_DOWN:
                //skystoneGrabBlue.setPosition(GRAB_RIGHT_CLOSE);
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

/*
// Arm methods

    public void homeArm () {
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
*/
}
