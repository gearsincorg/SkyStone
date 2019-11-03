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

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    /* Public OpMode members. */
    public AllianceColor allianceColor = AllianceColor.UNKNOWN_COLOR;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor arm = null;

    public Servo skystoneGrabLeft = null;
    public Servo skystoneLiftLeft = null;
    public Servo skystoneGrabRight = null;
    public Servo skystoneLiftRight = null;
    public Servo stoneGrab = null;
    public Servo stoneRotate = null;

    public static BNO055IMU imu = null;

    public static final double YAW_GAIN = 0.010;  // Rate at which we respond to heading error 0.013
    public static final double LATERAL_GAIN = 0.0025; // Distance from x axis that we start to slow down. 0027
    public static final double AXIAL_GAIN = 0.0015; // Distance from target that we start to slow down. 0017

    // Driving constants Yaw heading
    private final double HEADING_GAIN = 0.012;  // Was 0.02
    private final double TURN_RATE_TC = 0.6;
    private final double STOP_TURNRATE = 20;
    private final double GYRO_360_READING = 360.0;
    private final double GYRO_SCALE_FACTOR = 360.0 / GYRO_360_READING;
    private final double ACCELERATION_LIMIT = 0.5;
    private final double NUDGE_FACTOR = 0.1;


    final double YAW_IS_CLOSE = 2.0;  // angle within which we are "close"

    final double AXIAL_SCALE = 1;
    final double YAW_SCALE = 0.5;

    final double COUNTSPERDEGREE = 6.11;
    final double AXIAL_ENCODER_COUNTS_PER_INCH = 21.85; // Was 23.2
    final double LATERAL_ENCODER_COUNTS_PER_INCH = 23.2;

    private static LinearOpMode myOpMode = null;
    private boolean runningAuto = false;

    private double driveAxial = 0;
    private double driveYaw = 0;
    private double driveLateral = 0;

    private double startLeftBack = 0;
    private double startLeftFront = 0;
    private double startRightBack = 0;
    private double startRightFront = 0;
    private double deltaLeftBack = 0;
    private double deltaLeftFront = 0;
    private double deltaRightBack = 0;
    private double deltaRightFront = 0;

    private double startTime = 0;
    private double rampDistance = 0;
    private double tGoal = 0;
    public double axialMotion = 0;
    public double lateralMotion = 0;

    private static double targetArmAngle = 0;
    private static double currentArmAngle = 85;

    // gyro
    private double lastHeading = 0;
    private double lastGyroMs = 0;
    public double intervalGyroMs = 0;
    private double filteredTurnRate = 0;
    private double integratedZAxis = 0;
    private double adjustedIntegratedZAxis = 0;
    private double headingSetpoint = 0;
    public double robotPitch = 0;
    private int timeoutSoundID = 0;
    public boolean redSide = false;
    public boolean scoreBucket = false;
    public String autoPathName = "";


    /* local OpMode members. */

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime gyroTime = new ElapsedTime();
    private ElapsedTime navTime = new ElapsedTime();

    /* Constructor */
    public GFORCE_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode, boolean pleaseResetEncoders) {
        // Save reference to Hardware map
        myOpMode = opMode;
        runningAuto = pleaseResetEncoders;

        // Define and Initialize Motors
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        //arm = myOpMode.hardwareMap.get(DcMotor.class,"arm");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //arm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        skystoneGrabLeft = myOpMode.hardwareMap.get(Servo.class,"grab_left");
        skystoneLiftLeft = myOpMode.hardwareMap.get(Servo.class,"lift_left");
        skystoneGrabRight = myOpMode.hardwareMap.get(Servo.class,"grab_right");
        skystoneLiftRight = myOpMode.hardwareMap.get(Servo.class,"lift_right");
        setLeftSkystoneGrabber(SkystoneGrabberPositions.START);
        setRightSkystoneGrabber(SkystoneGrabberPositions.START);

        timeoutSoundID = myOpMode.hardwareMap.appContext.getResources().getIdentifier("squeek", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        if (timeoutSoundID != 0) {
            SoundPlayer.getInstance().preload(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }

        // setting up Gyro
        if (imu == null) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        if (pleaseResetEncoders) {
            resetEncoders();
        }

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        stopRobot();
        //arm.setPower(0);
        /*// Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/
    }

    // Autonomous driving methods

    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     *
     * @param inches
     * @param heading
     * @param speed
     * @param timeOutSec
     * @return
     */
    public boolean driveAxial(double inches, double heading, double speed, double timeOutSec) {
        double endingTime = runTime.seconds() + timeOutSec;
        double dTravelled;

        //Reverse angles for red autonomous
        if (redSide) {
            heading = -heading;
        }

        if ((inches < 0.0) || (speed < 0.0)) {
            speed = -Math.abs(speed);
        }

        //Save the current encoder counts
        startMotion();
        // Loop until the robot has driven to where it needs to go
        while (myOpMode.opModeIsActive() && updateMotion() &&
                (Math.abs(getAxialMotion()) < Math.abs(inches)) &&
                (runTime.seconds() < endingTime)) {
            setAxial(getProfileSpeed(speed, getAxialMotion(), inches));
            setLateral(0);
            setYawToHoldHeading(heading);
            moveRobot();
            showEncoders();
        }
        stopRobot();

        // Return true if we have not timed out
        return (runTime.seconds() < endingTime);
    }

    /**
     * Drive a set distance at a set heading at a set speed until the timeout occurs
     *
     * @param inches
     * @param heading
     * @param speed
     * @param timeOutSec
     * @return
     */
    public boolean driveLateral(double inches, double heading, double speed, double timeOutSec) {

        int desiredEncoderCounts = (int) (Math.abs(inches) * LATERAL_ENCODER_COUNTS_PER_INCH);
        double endingTime = runTime.seconds() + timeOutSec;

        if ((inches < 0.0) || (speed < 0.0)) {
            speed = -Math.abs(speed);
        }

        //Reverse angles for red autonomous
        if (redSide) {
            heading = -heading;
            speed = -speed;
        }

        //Save the current encoder counts
        startMotion();
        // Loop until the robot has driven to where it needs to go
        while (myOpMode.opModeIsActive() && updateMotion() &&
                (Math.abs(getLateralMotion()) < desiredEncoderCounts) &&
                (runTime.seconds() < endingTime)) {
            setAxial(0);
            setLateral(getProfileSpeed(speed, getAxialMotion(), inches));
            setYawToHoldHeading(heading);
            moveRobot();
            showEncoders();
        }
        stopRobot();

        // Return true if we have not timed out
        return (runTime.seconds() < endingTime);
    }

    //
    public double getProfileSpeed(double topSpeed, double dTraveled,double dGoal) {
        double profileSpeed = 0;
        double rampTime = Math.abs(topSpeed / ACCELERATION_LIMIT);

        //Determine if we are accelerating, constant, or decelerating
        dTraveled = Math.abs(dTraveled);
        if (getMotionTime() < rampTime) {
            //Determine speed and ensure sign matches topSpeed
            profileSpeed = ((getMotionTime() * ACCELERATION_LIMIT) + NUDGE_FACTOR) * Math.signum(topSpeed);
        } else {
            //Set the rampDistance the first time in the else
            if (rampDistance == 0) {
                rampDistance = dTraveled;
            }

            if (dTraveled < (dGoal-rampDistance)) {
                //At the constant velocity
                profileSpeed = topSpeed;
                tGoal = (getMotionTime() + rampTime);
            } else if (dTraveled < dGoal){
                profileSpeed = ((tGoal-getMotionTime()) * ACCELERATION_LIMIT) * Math.signum(topSpeed);
            } else {
                profileSpeed = 0;
            }
        }

        Log.d("G-FORCE AUTO", String.format("D: %5.2f, S: %4.2f", dTraveled, profileSpeed));
        return (profileSpeed);
    }

    //Get the current encoder counts of the drive motors
    public void startMotion() {
        startLeftBack = leftBackDrive.getCurrentPosition();
        startLeftFront = leftFrontDrive.getCurrentPosition();
        startRightBack = rightBackDrive.getCurrentPosition();
        startRightFront = rightFrontDrive.getCurrentPosition();
        deltaLeftBack = 0;
        deltaLeftFront = 0;
        deltaRightBack = 0;
        deltaRightFront = 0;
        startTime = runTime.time();
        rampDistance = 0;
    }

    public boolean updateMotion() {
        deltaLeftBack = leftBackDrive.getCurrentPosition() - startLeftBack;
        deltaLeftFront = leftFrontDrive.getCurrentPosition() - startLeftFront;
        deltaRightBack = rightBackDrive.getCurrentPosition() - startRightBack;
        deltaRightFront = rightFrontDrive.getCurrentPosition() - startRightFront;
        axialMotion = ((deltaLeftBack + deltaLeftFront + deltaRightBack + deltaRightFront) / 4);
        axialMotion /= AXIAL_ENCODER_COUNTS_PER_INCH;
        lateralMotion = ((-deltaLeftBack + deltaLeftFront + deltaRightBack - deltaRightFront) / 4);
        lateralMotion /= LATERAL_ENCODER_COUNTS_PER_INCH;

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
        // Flip translation and rotations if we are RED
        //Reverse angles for red autonomous
        if (redSide) {
            heading = -heading;
        }

        setHeadingSetpoint(heading);
        navTime.reset();
        setAxial(0);
        while (myOpMode.opModeIsActive() &&
                (navTime.time() < timeOutSEC) &&
                !setYawToHoldHeading()) {
            moveRobot();
        }
        if (navTime.time() > timeOutSEC) {
            playTimoutSound();
        }
        stopRobot();
        return (navTime.time() < timeOutSEC);
    }

    public boolean sleepAndHoldHeading(double heading, double timeOutSEC) {
        // Flip translation and rotations if we are RED
        //Reverse angles for red autonomous
        if (redSide) {
            heading = -heading;
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
            SoundPlayer.getInstance().startPlaying(myOpMode.hardwareMap.appContext, timeoutSoundID);
        }
    }

    public void resetEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void showEncoders() {
        myOpMode.telemetry.addData("motion","axial %6.1f, lateral %6.1f", getAxialMotion(), getLateralMotion());
        myOpMode.telemetry.addData("front", "%5d %5d ", deltaLeftFront, deltaRightFront);
        myOpMode.telemetry.addData("back",  "%5d %5d ", deltaLeftBack, deltaRightBack);
        myOpMode.telemetry.update();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    // The O' Great Button Click dilemma. Lest we forget this ever tricky conundrum. - Juan F. Aleman IV
    public void updateClickState() {

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

    public void stopRobot() {
        moveRobot(0, 0, 0);
    }


    // Actuator Methods
    private final double GRAB_LEFT_SAFE = 0;
    private final double GRAB_LEFT_READY = 0.5;
    private final double GRAB_LEFT_CLOSE = 1;

    private final double LIFT_LEFT_SAFE = 0;
    private final double LIFT_LEFT_CARRY = 0.25;
    private final double LIFT_LEFT_FOUNDATION = 0.5;
    private final double LIFT_LEFT_READY = 1;

    public void setLeftSkystoneGrabber(SkystoneGrabberPositions position) {

        switch (position) {
            case START:
                skystoneGrabLeft.setPosition(GRAB_LEFT_SAFE);
                skystoneLiftLeft.setPosition(LIFT_LEFT_SAFE);
                break;

            case READY:
                skystoneGrabLeft.setPosition(GRAB_LEFT_READY);
                skystoneLiftLeft.setPosition(GRAB_LEFT_READY);
                break;

            case GRAB_DOWN:
                skystoneGrabLeft.setPosition(GRAB_LEFT_CLOSE);
                skystoneLiftLeft.setPosition(LIFT_LEFT_READY);
                break;

            case GRAB_UP:
                skystoneGrabLeft.setPosition(GRAB_LEFT_CLOSE);
                skystoneLiftLeft.setPosition(LIFT_LEFT_CARRY);
                break;

            case FOUNDATION_READY:
                skystoneGrabLeft.setPosition(GRAB_LEFT_CLOSE);
                skystoneLiftLeft.setPosition(LIFT_LEFT_FOUNDATION);
                break;

            case FOUNDATION_RELEASE:
                skystoneGrabLeft.setPosition(GRAB_LEFT_SAFE);
                skystoneLiftLeft.setPosition(LIFT_LEFT_FOUNDATION);
                break;
        }
    }

    private final double GRAB_RIGHT_SAFE = 0.55;
    private final double GRAB_RIGHT_READY = 0.55;
    private final double GRAB_RIGHT_CLOSE = 1;

    private final double LIFT_RIGHT_SAFE = 0.0;
    private final double LIFT_RIGHT_CARRY = 0.2;
    private final double LIFT_RIGHT_FOUNDATION = 0.4;
    private final double LIFT_RIGHT_READY = 0.5;


    public void setRightSkystoneGrabber(SkystoneGrabberPositions position) {

        switch (position) {
            case START:
                skystoneGrabRight.setPosition(GRAB_RIGHT_SAFE);
                skystoneLiftRight.setPosition(LIFT_RIGHT_SAFE);
                break;

            case READY:
                skystoneGrabRight.setPosition(GRAB_RIGHT_READY);
                skystoneLiftRight.setPosition(LIFT_RIGHT_READY);
                break;

            case GRAB_DOWN:
                skystoneGrabRight.setPosition(GRAB_RIGHT_CLOSE);
                skystoneLiftRight.setPosition(LIFT_RIGHT_READY);
                break;

            case GRAB_UP:
                skystoneGrabRight.setPosition(GRAB_RIGHT_CLOSE);
                skystoneLiftRight.setPosition(LIFT_RIGHT_CARRY);
                break;

            case FOUNDATION_READY:
                skystoneGrabRight.setPosition(GRAB_RIGHT_CLOSE);
                skystoneLiftRight.setPosition(LIFT_RIGHT_FOUNDATION);
                break;

            case FOUNDATION_RELEASE:
                skystoneGrabRight.setPosition(GRAB_RIGHT_SAFE);
                skystoneLiftRight.setPosition(LIFT_RIGHT_FOUNDATION);
                break;
        }
    }

    public void setSkystoneGrabber (SkystoneGrabberPositions position) {
        switch (allianceColor) {
            case UNKNOWN_COLOR:
                break;

            case RED:
                setRightSkystoneGrabber(position);
                break;

            case BLUE:
                setLeftSkystoneGrabber(position);
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
