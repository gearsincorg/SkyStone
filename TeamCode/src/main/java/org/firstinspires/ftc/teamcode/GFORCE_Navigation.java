package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific navigation tasks for the G-FORCE robot
 */

public class GFORCE_Navigation
{
    private static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targetsSkyStone = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    // Constants
    static final String TAG = "NAV";

    private static final int     MAX_TARGETS         =  13;
    private static final double  ON_AXIS             =  10;      // Within 1 cm of centerline
    private static final double  CLOSE_ENOUGH        =  20;      // Within 2.0 cm of final target
    private static final double  NEAR_CENTER         = 100;      // Within 4 inches of center line

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    // Public Members
    public  double robotX;
    public  double robotY;
    public  double targetRange;
    public  double targetBearing;
    public  double robotBearing;
    public  double relativeBearing;

    /* Private OpMode members. */
    private LinearOpMode        myOpMode;
    private GFORCE_Hardware     myRobot;

    // Following data is only valid if targetFound == true;
    private boolean             targetFound;
    private String              targetName;
    private OpenGLMatrix        robotLocation;
    private ElapsedTime navTime;    // Used to track driving timeout

    /* Constructor */
    public GFORCE_Navigation(){

        robotX = 0;
        robotY = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;
        targetFound = false;
        targetName = null;
        robotLocation = null;
        navTime = new ElapsedTime();
        targetsSkyStone = null;
    }

    /***
     * Send telemetry data for navigation
     */
    public void addNavTelemetry() {
        if (targetFound)
        {
            myOpMode.telemetry.addData(targetName, "X:Y B [%+5.0f]:[%+5.0f] (%+5.2f)",
                    robotX, robotY, robotBearing);
            myOpMode.telemetry.addData(targetName, "R B:<> [%+5.0f] (%+5.2f):(%+5.2f)",
                    targetRange, targetBearing, relativeBearing);
        }
        // myOpMode.telemetry.addData("Gyro", "%6.1f°", myRobot.getHeading()  );
    }

    /***
     *  Lock onto target and track to desired standoff position in front of image
     * @param targetID          Vuforia Target Index
     * @param standOffDistance  Desired target distance in mm
     * @param axialDrift        Default motion if no image found
     * @param timeOutSEC        Max time to acquire target
     * @return
     */
    public boolean acquireTarget(int targetID, double standOffDistance, double axialDrift, double timeOutSEC){
        navTime.reset();
        while (myOpMode.opModeIsActive() && !atTarget(targetID, standOffDistance, axialDrift) && (navTime.time() < timeOutSEC)) {

            myOpMode.telemetry.addData("Path", myRobot.autoPathName);
            myOpMode.telemetry.addData("Acquire Targ" , "Elapsed Time %3.1f", navTime.time());
            myOpMode.telemetry.update();
        }
        myRobot.stopRobot();
        myOpMode.telemetry.update();

       //return true if we saw the target
        return (navTime.time() < timeOutSEC);
    }

    /***
     * Start tacking Vuforia images
     */
    public void activateTracking() {

        /** Start tracking the data sets we care about. */
        if (targetsSkyStone != null)
            targetsSkyStone.activate();
    }

    /***
     *  Determines drive required to get in front of target (on X axis) and away by STANDOFF_DISTANCE
     * @param targetID
     * @param standOffDistance
     * @param axialDrift
     * @return
     */
    boolean atTarget(int targetID, double standOffDistance, double axialDrift) {
        boolean closeEnough = false;

        // In this mode the robot attempts to get in front of target (on X axis) and away by STANDOFF_DISTANCE
        if (targetIsVisible(targetID))
        {
            addNavTelemetry();
            closeEnough = targetDirections(standOffDistance);
        }
        else
        {
            // Drive slowly forward
            myRobot.setAxialVelocity(axialDrift);
            myRobot.setLateralVelocity(0);
            myRobot.setYawVelocity(0);
        }
        myRobot.moveRobotVelocity();
        return closeEnough;
    }

    /***
     *  Strafes across the field looking for a target image.  Stop when near the target centerline
     * @param targetID  Vuforia target index
     * @param axial     Desired Axial motion to find target
     * @param lateral   desired Lateral motion to find target
     * @param heading   desired robot heading while searching for target
     * @param timeOutSEC Max time to locate target
     * @return
     */
    public boolean findTarget(int targetID, double axial, double lateral, double heading, double timeOutSEC) {
        boolean         targetLock      = false;

        // Flip translation and rotations if we are RED
        if (myRobot.allianceColor == GFORCE_Hardware.AllianceColor.RED) {
            lateral *= -1.0;
            heading *= -1.0;
        }

        navTime.reset();
        while (myOpMode.opModeIsActive() && !targetLock && (navTime.time() <  timeOutSEC)){
            myRobot.setAxialVelocity(axial);
            myRobot.setLateralVelocity(lateral);
            myRobot.getHeading();
            myRobot.setYawVelocityToHoldHeading();
            myRobot.moveRobotVelocity();

            if (targetIsVisible(targetID)) {
                targetLock = (Math.abs(robotY) < NEAR_CENTER);
            }
            myOpMode.telemetry.addData("Path", myRobot.autoPathName);
            myOpMode.telemetry.addData("Find Targ", "A:L:H %5.3f %5.3f %3.0f", axial, lateral, heading);
            showNavTelemetry(false);
            myOpMode.telemetry.update();
        }
        myRobot.stopRobot();
        return (targetLock);
    }

     /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode, GFORCE_Hardware robot) {

        // Save reference to Hardware map
        myOpMode = opMode;
        myRobot = robot;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == FRONT) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 70;    // eg: Camera is 70 mm behind the robot origin (front edge for grabbing SkyStone)
        final float CAMERA_VERTICAL_DISPLACEMENT = 100;   // eg: Camera is 100 mm above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsSkyStone.activate();
    }

    /***
     * driveBlind simply drives in deadreckoning mode in the requested direction
     * @param axial     Power in axial direction
     * @param lateral   Power in Lateral Direction
     * @param heading   Desired robot heading
     * @param timeOutSEC  Desited transit time.
     */
    public void driveBlind(double axial, double lateral, double heading, double timeOutSEC) {
        // Flip translation and rotations if we are RED
        if (myRobot.allianceColor == GFORCE_Hardware.AllianceColor.RED) {
            lateral *= -1.0;
            heading *= -1.0;
        }

        navTime.reset();
        while (myOpMode.opModeIsActive() && (navTime.time() <  timeOutSEC)){
            myRobot.setAxialVelocity(axial);
            myRobot.setLateralVelocity(lateral);
            myRobot.getHeading();
            myRobot.setYawVelocityToHoldHeading(heading);
            myRobot.moveRobotVelocity();

            myOpMode.telemetry.addData("Path", myRobot.autoPathName);
            myOpMode.telemetry.addData("Blind Move", "A:L:H %5.2f %5.2f %3.0f", axial, lateral, heading);
            myOpMode.telemetry.update();
        }
        myRobot.stopRobot();
    }

    // See if ANY of the targets are visible.
    public boolean targetsAreVisible()  {

        int targetTestID = 0;

        // Check each target and stop looking when any target is found.
        while (myOpMode.opModeIsActive() && (targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++ ;
        }

        return (targetFound);
    }


    public boolean waitForTarget(int timeout) {
        navTime.reset();
        while (myOpMode.opModeIsActive() && !targetIsVisible(0) && (navTime.time() < timeout)) {
            showNavTelemetry(true);
        }
        showNavTelemetry(true);

        return (targetFound);
    }

    /***
     * use target position to determine directions to target
     * @return true if we are close to target
     * @param standOffDistance how close do we get to the target(mm)
     */
    public boolean targetDirections(double standOffDistance) {
        // Rotate to always be pointing at the target (for best target retention).
        // Drive forward based on the desiredHeading target standoff ditance
        // Drive laterally based on distance from X axis (same as y value)
        boolean closeEnough;
        double Y  = (relativeBearing * myRobot.YAW_GAIN);
        double A  = (-(robotX + standOffDistance) * myRobot.AXIAL_GAIN);
        double L  =(robotY * myRobot.LATERAL_GAIN);

        myRobot.setYawVelocity(Y);
        myRobot.setAxialVelocity(A);
        myRobot.setLateralVelocity(L);
        closeEnough = ( (Math.abs(robotX + standOffDistance) < CLOSE_ENOUGH) &&
                (Math.abs(robotY) < ON_AXIS));

        return (closeEnough);
    }

    /***
     * Determine if specified target ID is visible and save relative positions
     * @param targetId
     * @return
     */
    public boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targetsSkyStone.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null;

        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            // Look for new location
            location  = listener.getUpdatedRobotLocation();
            if (location != null) {

                robotLocation = location;
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target currentHeading relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing;
            }
            targetFound = true;
        }
        else  {
            targetFound = false;
            robotLocation = null;
            targetName = "None";
        }

        return targetFound;
    }

    public void showNavTelemetry(boolean doUpdate) {
        if (targetFound) {
            myOpMode.telemetry.addData("Target",targetName);
            myOpMode.telemetry.addData("Robot Location","X: %5.0f,  Y: %5.0f,  B: %6.1f",robotX,robotY,robotBearing);
            myOpMode.telemetry.addData("Target Location","R: %5.0f, B: %6.1f", targetRange,targetBearing);
        } else {
            myOpMode.telemetry.addData("Target","Not Found");
        }
        if (doUpdate) {
            myOpMode.telemetry.update();
        }
    }
}

