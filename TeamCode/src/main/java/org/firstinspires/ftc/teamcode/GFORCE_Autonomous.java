/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Autonomous opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="G-FORCE Autonomous", group="!Competition")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig   autoConfig    = new AutoConfig();
    GFORCE_Hardware     robot         = new GFORCE_Hardware();
    GFORCE_Navigation   nav           = new GFORCE_Navigation();

    final double STUD_RANGE = 10;               // Plus or minus 10 millimeters
    final double BLOCK_PUSH_DISTANCE = 30;      // Was an extra 10mm, changed to 30 mm for new phone mount because it is further back
    final double BLOCK_CENTER_OFFSET = -45;     // Camera to grab offset
    public static final String TAG = "G-FORCE";

    double error;
    int skyStonePosition = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this);
        nav.init(this, robot);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
            if (nav.targetIsVisible(0)) {
                nav.addNavTelemetry();
            }
            autoConfig.init_loop(); //Run menu system
        }

        // Get alliance color from Menu
        if (autoConfig.autoOptions.redAlliance) {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.RED;
        } else {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.BLUE;
        }

        //Starting autonomous reset heading to zero
        robot.resetHeading();
        if (!autoConfig.autoOptions.disabled) {
            if (autoConfig.autoOptions.foundation) {
                repositionFoundation();

            } else {
                scoreSkyStone();
            }

        }


        while(opModeIsActive()) {
            nav.waitForTarget(1);
            nav.showNavTelemetry(true);
        }
    }

    public void driveAndGrabBlock() {
        // Determine the error based on the target OFFSET required
        if (robot.allianceColor == GFORCE_Hardware.AllianceColor.RED) {
            error = ( BLOCK_CENTER_OFFSET - nav.robotY);
        } else {
            error = (BLOCK_CENTER_OFFSET + nav.robotY);
        }


        RobotLog.ii(TAG, String.format("Error=%5.0f robotY=%5.0f ", error, nav.robotY));
        robot.driveAxialVelocity(error, 0, 80,2,true);

        //Calculate new error
        error = (-nav.robotX);
        robot.driveLateralVelocity(error + BLOCK_PUSH_DISTANCE, 0, 380, 2, true);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(400, 0, -500, 4, true);
    }

    private void scoreSkyStone() {
        robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
        robot.driveLateralVelocity(600, 0, 600, 4, true);
        robot.sleepAndHoldHeading(0, 0.5);

        if (nav.waitForTarget(1.5)) {
            skyStonePosition = 1;
        } else {
            robot.driveAxialVelocity(200,0,150,2,true);
            if (nav.waitForTarget(1)) {
                skyStonePosition = 2;
            }
            else {
                robot.driveAxialVelocity(200,0,150,2,true);
                if (nav.waitForTarget(1)) {
                    skyStonePosition = 3;
                }
            }
        }


        if (skyStonePosition > 0) {
            driveAndGrabBlock();
            robot.driveAxialVelocity(900 + (skyStonePosition * 200), 0, -800, 20, true);
            robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
            robot.sleepAndHoldHeading(0, 1);
            robot.driveAxialVelocity( 1700 + (skyStonePosition * 200), 0, 1000, 20, true);
            robot.sleepAndHoldHeading(0, 0.5);
            if (!nav.waitForTarget(1)) {
                robot.driveLateralVelocity(300, 0, 600, 2, true);
            }

            if (nav.waitForTarget(1)) {
                driveAndGrabBlock();
                robot.driveAxialVelocity(1400 + (skyStonePosition * 200), 0, -800, 20, true);
                robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
                robot.driveAxialVelocity(300, 0, 400, 4, true);
                robot.sleepAndHoldHeading(0, 1);
            }
        }

    }

    private void repositionFoundation() {
        robot.driveLateralVelocity(300, 0, 300, 4, true);
        robot.driveAxialVelocity(200,0,-250,4,true);
        robot.turnToHeading(90,4);
        robot.sleepAndHoldHeading(90, 1);
        robot.driveAxialVelocity(500,90,-400,4,true);
        robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_DOWN_L);
        robot.foundationGrabberRight.setPosition(robot.FOUNDATION_DOWN_R);
        robot.sleepAndHoldHeading(90, 1);
        robot.turnToHeading(0,4);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(450,0,-800,3, true);
        robot.driveAxialVelocity(150,0,-1000,1,true);
        robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
        robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);

    }
}