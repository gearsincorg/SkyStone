/**
 * This class is used to define all the specific navigation tasks for the G-FORCE robot
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
    static final boolean LOGGING = false;

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
    private boolean             targetAverageAvailable;
    private int                 targetAverageCount;
    public double              targetAverageX;
    public double              targetAverageY;

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
        targetAverageAvailable = false;
        targetAverageCount = 0;
        targetAverageX = 0;
        targetAverageY = 0;
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

     /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode, GFORCE_Hardware robot) {

        // Save reference to Hardware map
        myOpMode = opMode;
        myRobot = robot;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        // int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

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
        final float CAMERA_FORWARD_DISPLACEMENT  = 0;    // eg: Camera is 00 mm behind the robot origin (front edge for grabbing SkyStone)
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        final float CAMERA_VERTICAL_DISPLACEMENT = 100;   // eg: Camera is 100 mm above ground

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let stone target  trackable listeners know where the phone is.  */
        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        targetsSkyStone.activate();
    }

    public boolean waitForTarget(double timeout) {
        navTime.reset();
        resetTargetAverages();
        while (myOpMode.opModeIsActive() && !targetAverageAvailable && (navTime.time() < timeout)) {
            targetIsVisible(0);
            myRobot.setYawVelocityToHoldHeadingWithUpdate();
            myRobot.moveRobotVelocity();
            myRobot.runLiftControl();  // Keep the lift in desired position

            showNavTelemetry(true);
        }
        myRobot.stopRobot();
        showNavTelemetry(true);

        return (targetFound);
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

                //Calculate target averages if requested
                if (!targetAverageAvailable) {
                    targetAverageX += robotX;
                    targetAverageY += robotY;
                    targetAverageCount++;

                    if (targetAverageCount == 3) {
                        targetAverageX /= 3;
                        targetAverageY /= 3;
                        robotX = targetAverageX;
                        robotY = targetAverageY;
                        targetAverageAvailable = true;
                    }
                }
                if (LOGGING) RobotLog.ii(TAG, String.format("TIV NEW: Time=%5.3f robotX=%5.0f robotY=%5.0f ", navTime.time(), robotX, robotY));
            }
            else {
                if (LOGGING) RobotLog.ii(TAG, String.format("TIV OLD: Time=%5.3f robotX=%5.0f robotY=%5.0f ", navTime.time(), robotX, robotY));
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

    public void resetTargetAverages() {
        targetAverageX = 0;
        targetAverageY = 0;
        targetAverageAvailable = false;
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

