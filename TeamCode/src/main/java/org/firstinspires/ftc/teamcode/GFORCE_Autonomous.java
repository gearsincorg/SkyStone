/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Autonomous opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

@Autonomous(name="G-FORCE Autonomous", group="!Competition")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig   autoConfig    = new AutoConfig();
    public GFORCE_Hardware     robot         = new GFORCE_Hardware();
    public GFORCE_Navigation   nav           = new GFORCE_Navigation();

    final double BLOCK_PUSH_DISTANCE = 10;      // Was an extra 10mm, changed to 30 mm for new phone mount because it is further back
    final double BLOCK_CENTER_OFFSET = -60;    // Camera to grab offset
    public static final String TAG = "G-FORCE";

    private ElapsedTime autoTime = new ElapsedTime();

    double error;
    int skyStonePosition = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this);
        nav.init(this, robot);

        CameraDevice.getInstance().setFlashTorchMode( true );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
            if (nav.targetIsVisible(0)) {
                nav.addNavTelemetry();
            }
            autoConfig.init_loop(); //Run menu system
            sleep(20);
        }

        // Get alliance color from Menu
        if (autoConfig.autoOptions.redAlliance) {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.RED;
        } else {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.BLUE;
        }

        //Starting autonomous reset heading to zero
        robot.resetHeading();
        autoTime.reset();

        if (autoConfig.autoOptions.enabled) {
            if (autoConfig.autoOptions.startBuilding) {
                startBuildingZone();

            } else {
                startQuarry();
            }

        } else {
            // Testing code
            robot.driveLateralVelocity(500, 0, 600,5,true,false);
            robot.driveAxialVelocity(1000, 0, 600,5,true,false);
            robot.stopRobot();
            sleep(1000);

            /*
            robot.driveAxialVelocity(600, 0, 100,10,true,false);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveAxialVelocity(600, 0, -100,10,true,false);
            robot.sleepAndHoldHeading(0, 1);

            robot.driveAxialVelocity(600, 0, 250,10,true,false);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveAxialVelocity(600, 0, -250,10,true,false);
            robot.sleepAndHoldHeading(0, 1);

            robot.driveAxialVelocity(600, 0, 1000,10,true,false);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveAxialVelocity(600, 0, -1000,10,true,false);
            robot.sleepAndHoldHeading(0, 1);

            robot.driveAxialVelocity(600, 0, 1500,10,true,false);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveAxialVelocity(600, 0, -1500,10,true,false);
            robot.sleepAndHoldHeading(0, 1);

            robot.driveAxialVelocity(600, 0, 2000,10,true,false);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveAxialVelocity(600, 0, -2000,10,true,false);
            robot.sleepAndHoldHeading(0, 1);

             */
        }


        CameraDevice.getInstance().setFlashTorchMode( false );

        robot.stopRobot();
    }

    public void driveAndGrabBlock() {
        // Determine the error based on the target OFFSET required
        nav.targetIsVisible(0);
        if (robot.allianceColor == GFORCE_Hardware.AllianceColor.RED) {
            error = ( BLOCK_CENTER_OFFSET - nav.robotY);
        } else {
            error = (BLOCK_CENTER_OFFSET + nav.robotY);
        }

        RobotLog.ii(TAG, String.format("Error=%5.0f robotX=%5.0f robotY=%5.0f ", error, nav.robotX, nav.robotY));
        robot.driveAxialVelocity(error, 0, 240,2,true,true);

        //Calculate new error
        nav.targetIsVisible(0);
        RobotLog.ii(TAG, String.format("Error=%5.0f robotX=%5.0f robotY=%5.0f ", error, nav.robotX, nav.robotY));
        error = (-nav.robotX);

        if (robot.allianceColor == GFORCE_Hardware.AllianceColor.BLUE) {
            error -= 70;
        }

        robot.driveLateralVelocity(error + BLOCK_PUSH_DISTANCE, 0, 380, 2, true,false);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(400, 0, -500, 4, true,false);
    }

    private void startQuarry() {

        //Are we doing any SkyStones?
        if (autoConfig.autoOptions.scoreFirstSkyStone || autoConfig.autoOptions.scoreBothSkyStones) {

            // Move to where you should be able to see skystones 2 or 3 and release collectors
            robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
            robot.driveLateralVelocity(500, 0, 600, 4, true, false);
            robot.releaseCollectorArms();

            if (nav.waitForTarget(0.25)) {
                double blockCenter = 220 - nav.robotY ;  // Offset from tile center
                if (blockCenter > 200) {
                    skyStonePosition = 2;
                } else {
                    skyStonePosition = 3;
                }

                RobotLog.ii(TAG, String.format("BLOCK T:M:O:P %5.0f %5.0f %5.0f %d",
                        nav.robotY, robot.getAxialMotion(), blockCenter, skyStonePosition ));
            }
            else {
                robot.driveAxialVelocity(300, 0, -600, 2,true, true);
                if (nav.waitForTarget(0.5)) {
                    skyStonePosition = 1;
                }
            }

            //Get the SkyStone if we found it
            if (skyStonePosition > 0) {
                //Getting and placing the first SkyStone
                double axialDistance = (390 - nav.robotY);
                double lateralDistance = Math.abs(nav.robotX) - 170;
                robot.driveLateralVelocity(lateralDistance,0,600,3,true,false);
                robot.driveAxialVelocity(axialDistance,0,-600,3,true,true);
                sleep(100);
                robot.turnToHeading(-135,2);
                robot.sleepAndHoldHeading(-135,1);
                robot.transferStone(1);
                robot.runCollector(1);

                // Collect Skystone
                robot.driveAxialVelocity(400,-135,-300,2, true, true);
                robot.driveAxialVelocity(400,-135,400,2,true,true);
                robot.runCollector(0.25);
                robot.turnToHeading(-180,2);
                robot.runCollector(0);

                // Drive to Foundation
                robot.driveAxialVelocity(1200 + (200 * skyStonePosition),-180,1200,4,true,true);
                robot.grabStone(true);
                robot.stoneInGrasp = true;
                robot.turnToHeading(-270,2);
                robot.transferStone(0);
                robot.driveAxialVelocity(300,-270,200,1,true,true);
                robot.transferStone(0);

                // Place stone on foundation
                robot.extendStone(true);
                sleep(750);

                // Are we moving foundation
                if (autoConfig.autoOptions.moveFoundation) {
                    robot.grabFoundation(true);
                }

                // Release stone and retract shuttle
                robot.grabStone(false);
                sleep(500);
                robot.extendStone(false);

                // Are we moving the foundation?
                if (autoConfig.autoOptions.moveFoundation) {
                    // Pull foundation towards driver station and turn it to back wall
                    robot.driveAxialVelocity(600, -270, -600, 4, true, true);
                    robot.turnToHeading(-180, 4);

                    // Release foundation and push against back wall
                    robot.grabFoundation(false);
                    robot.driveAxialVelocity(200, -180, 300, 1, true, true);

                    // Are we parking?  Where?
                    if (autoConfig.autoOptions.park) {
                        if (autoConfig.autoOptions.parkCloseToWall) {
                            robot.driveLateralVelocity(500, -180, 600, 3, true,false);
                        } else {
                            robot.driveLateralVelocity(-700, -180, 600, 3, true,false);
                        }
                        robot.driveAxialVelocity(1200, -180, -1000, 2, true, true);
                    }
                } else {
                    if (autoConfig.autoOptions.park) {
                        if (autoConfig.autoOptions.parkCloseToWall) {
                            robot.driveAxialVelocity(800, -270, -600, 4, true, true);
                        } else {
                            robot.driveAxialVelocity(150, -270, -600, 4, true, true);
                        }
                        robot.turnToHeading(-180, 2);
                        robot.driveAxialVelocity(1000, -180, -1000, 2, true, true);
                    }
                }
            }
        } else {

            //Parking under bridge if we choose not to score any SkyStones
            if (autoConfig.autoOptions.park) {
                holdForPark();
                if (autoConfig.autoOptions.parkCloseToWall) {
                    //park near Wall
                    robot.driveLateralVelocity(25, 0, 400, 3, true, false);
                    robot.driveAxialVelocity(1300, 0, -400, 4, true, true);
                } else {
                    //park near bridge code
                    robot.driveLateralVelocity(700, 0, 400, 3, true, false);
                    robot.driveAxialVelocity(1300, 0, -400, 4, true, true);
                }
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
                if (autoConfig.autoOptions.park) {
                    holdForPark();
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        // Park by wall
                        robot.driveLateralVelocity(600, 0, -600, 2, true, false);
                        robot.driveAxialVelocity(1100, 0, 800, 4, true, false);
                    }
                    else {
                        // Park by Bridge
                        // robot.driveLateralVelocity(600, 0, -600, 2, true, false);  // May not need any movement to wards wall
                        robot.driveAxialVelocity(1100, 0, 800, 4, true, false);
                    }
                }
            } else {
                robot.turnToHeading(180, 4);
                robot.driveAxialVelocity(200, 180, -1000, 1, true, false);
                robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
                robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);
                if (autoConfig.autoOptions.park) {
                    holdForPark();
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        // Park by wall
                        robot.driveLateralVelocity(600, 180, 600, 2, true, false);
                        robot.driveAxialVelocity(1100, 180, 800, 4, true, false);
                    }
                    else {
                        // Park by Bridge
                        // robot.driveLateralVelocity(600, 180, 600, 2, true, false); // May not need any movement to wards wall
                        robot.driveAxialVelocity(1100, 180, 800, 4, true, false);
                    }
                }
            }
        } else if (autoConfig.autoOptions.park) {
            holdForPark();
            if (autoConfig.autoOptions.parkCloseToWall) {
                //park near Wall
                robot.driveLateralVelocity(25, 0, 100, 2, true, false);
                robot.driveAxialVelocity(800, 0, 1000, 4, true, true);
            } else {
                //park near bridge code
                robot.driveLateralVelocity(700, 0, 100, 2, true, false);
                robot.driveAxialVelocity(800, 0, 1000, 4, true, true);
            }
        }
    }

    private void holdForPark() {
        // Wait here till delay timer elapsed.
        while (autoTime.time() < autoConfig.autoOptions.delayInSec) {
            telemetry.addData("Park Countdown", "%4.1f", autoConfig.autoOptions.delayInSec - autoTime.time() );
            telemetry.update();
        }
    }


}