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
            if (autoConfig.autoOptions.startBuilding) {
                startBuildingZone();

            } else {
                startQuarry();
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
        robot.driveAxialVelocity(error, 0, 80,2,true,true);

        //Calculate new error
        error = (-nav.robotX);
        robot.driveLateralVelocity(error + BLOCK_PUSH_DISTANCE, 0, 380, 2, true,false);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(400, 0, -500, 4, true,false);
    }

    private void startQuarry() {

        //Are we doing any SkyStones?
        if (autoConfig.autoOptions.scoreFirstSkyStone || autoConfig.autoOptions.scoreBothSkyStones) {

            //Finding the first SkyStone
            robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
            robot.driveLateralVelocity(600, 0, 600, 4, true, false);
            robot.sleepAndHoldHeading(0, 0.5);

            //Determine the position of the SkyStone
            if (nav.waitForTarget(1.5)) {
                skyStonePosition = 1;
            } else {
                robot.driveAxialVelocity(200, 0, 150, 2, true, true);
                if (nav.waitForTarget(1)) {
                    skyStonePosition = 2;
                } else {
                    robot.driveAxialVelocity(200, 0, 150, 2, true, true);
                    if (nav.waitForTarget(1)) {
                        skyStonePosition = 3;
                    }
                }
            }

            //Get the first SkyStone if we found it
            if (skyStonePosition > 0) {
                //Getting and placing the first SkyStone
                driveAndGrabBlock();
                robot.driveAxialVelocity(900 + (skyStonePosition * 200), 0, -800, 20, true, true);
                robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
                robot.sleepAndHoldHeading(0, 0.5);

                // Getting and placing second SkyStone if requested
                if (autoConfig.autoOptions.scoreBothSkyStones) {
                    robot.driveAxialVelocity(1625 + (skyStonePosition * 175), 0, 1100, 20, true, true);
//                    robot.sleepAndHoldHeading(0, 0.5);

                    //If we can't see the target drive closer
//                    if (!nav.waitForTarget(1)) {
                        robot.driveLateralVelocity(150, 0, 600, 2, true, false);
//                    }

                    //Get the second SkyStone if we found it
                    if (nav.waitForTarget(1)) {
                        driveAndGrabBlock();
                        robot.driveAxialVelocity(1400 + (skyStonePosition * 200), 0, -1000, 20, true, true);
                        robot.setSkystoneGrabber(SkystoneGrabberPositions.START);

                        //Parking under the bridge if requested regardless of if we did both SkyStones
                        if (autoConfig.autoOptions.parkUnderBridge) {
                            robot.driveAxialVelocity(300, 0, 400, 4, true, true);
                        }
                    } else {

                        //If we didn't find the second SkyStone
                        if (autoConfig.autoOptions.parkUnderBridge) {
                            robot.driveLateralVelocity(150,0,-300,3,true,false);
                            robot.driveAxialVelocity(1400, 0, -1000, 6, true, true);
                        }
                    }

                } else {

                    //If we only decided to find and place one SkyStone
                    if (autoConfig.autoOptions.parkUnderBridge) {
                        robot.driveAxialVelocity(750, 0, 400, 5, true, true);
                    }
                }

            } else {

                //We did not see ANY SkyStones
                if (autoConfig.autoOptions.parkUnderBridge) {
                    robot.driveLateralVelocity(150,0,-300,3,true,true);
                    robot.driveAxialVelocity(900, 0, -1000, 4, true, true);

                }
            }
        } else {

            //Parking under bridge if we choose not to score any SkyStones
            if (autoConfig.autoOptions.parkUnderBridge) {
                robot.driveLateralVelocity(50,0,400,3,true,true);
                robot.driveAxialVelocity(400, 0, -400, 4, true, true);
            }
        }

    }

    private void startBuildingZone() {
        if (autoConfig.autoOptions.moveFoundation) {
            robot.driveLateralVelocity(300, 0, 300, 4, true, false);
            robot.driveAxialVelocity(200, 0, -250, 4, true, true);
            robot.turnToHeading(90, 4);
            robot.sleepAndHoldHeading(90, 1);
            robot.driveAxialVelocity(500, 90, -400, 4, true, false);
            robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_DOWN_L);
            robot.foundationGrabberRight.setPosition(robot.FOUNDATION_DOWN_R);
            robot.sleepAndHoldHeading(90,1);
            robot.driveAxialVelocity(150, 90, 600, 4, true, false);

            if (robot.allianceColor == GFORCE_Hardware.AllianceColor.RED) {
                robot.turnToHeading(0, 4);
                robot.driveAxialVelocity(200, 0, -1000, 1, true, false);
                robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
                robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);
                if (autoConfig.autoOptions.parkUnderBridge) {
                    robot.driveLateralVelocity(600,0,-600,2,true,false);
                    robot.driveAxialVelocity(1100, 0, 800, 4, true, false);
                }
            } else {
                robot.turnToHeading(180, 4);
                robot.driveAxialVelocity(200, 180, -1000, 1, true, false);
                robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
                robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);
                if (autoConfig.autoOptions.parkUnderBridge) {
                    robot.driveLateralVelocity(600,180,600,2,true,false);
                    robot.driveAxialVelocity(1100, 180, 800, 4, true, false);
                }

            }
        } else if (autoConfig.autoOptions.parkUnderBridge) {
                robot.driveLateralVelocity(50,0,100,2,true,false);
                robot.driveAxialVelocity(800, 0, 1000, 4, true, true);
        }

    }
}