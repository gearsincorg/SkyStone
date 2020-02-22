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
            /*
            robot.driveLateralVelocity(500, 0, 600,5,false);
            robot.driveAxialVelocity(1000, 0, 600,5,false);
            robot.stopRobot();
            sleep(1000);

            robot.driveAxialVelocity(600, 0, 100,10,false);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveAxialVelocity(600, 0, -100,10,false);
            robot.sleepAndHoldHeading(0, 1);
          */
        }

        CameraDevice.getInstance().setFlashTorchMode( false );
        robot.stopRobot();
    }

    private void startQuarry() {

        //Are we doing any SkyStones?
        if (autoConfig.autoOptions.scoreFirstSkyStone || autoConfig.autoOptions.scoreBothSkyStones) {

            // Move to where you should be able to see skystones 2 or 3 and release collectors
            robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
            robot.driveLateralVelocity(500, 0, 600, 4,  false);
            robot.releaseCollectorArms();

            // Can we see skystones 2 or 3:
            if (nav.waitForTarget(0.25)) {
                double blockCenter = 220 - nav.robotY ;  // Offset from tile center
                if (blockCenter > 200) {
                    skyStonePosition = 2;
                } else {
                    skyStonePosition = 3;
                }

                RobotLog.ii(TAG, String.format("BLOCK T:M:O:P %5.0f %5.0f %5.0f %d", nav.robotY, robot.getAxialMotion(), blockCenter, skyStonePosition ));
            }
            else {
                // Did not see 2 or 3,  move to look for 1
                robot.driveAxialVelocity(300, 0, -600, 2,true);
                if (nav.waitForTarget(0.5)) {
                    skyStonePosition = 1;
                }
            }

            //Get the SkyStone if we found it
            if (skyStonePosition > 0) {
                //Getting and placing the first SkyStone
                double axialDistance = (390 - nav.robotY);
                double lateralDistance = Math.abs(nav.robotX) - 170;
                robot.driveLateralVelocity(lateralDistance,0,600,3,false);
                robot.driveAxialVelocity(axialDistance,0,-600,3,true);
                sleep(100);
                robot.turnToHeading(-135,2);
                robot.sleepAndHoldHeading(-135,1);
                robot.transferStone(1);
                robot.runCollector(1);

                // Collect Skystone
                robot.driveAxialVelocity(400,-135,-300,2, true);
                robot.driveAxialVelocity(400,-135,400,2,true);
                robot.runCollector(0.25);
                robot.turnToHeading(-180,2);
                robot.runCollector(0);

                // Drive to Foundation
                robot.driveAxialVelocity(1200 + (200 * skyStonePosition),-180,1200,4,true);
                robot.grabStone(true);
                robot.stoneInGrasp = true;
                robot.turnToHeading(-270,2);
                robot.transferStone(0);
                robot.driveAxialVelocity(300,-270,200,1,true);
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
                    robot.driveAxialVelocity(600, -270, -600, 4, true);
                    robot.turnToHeading(-180, 4);

                    // Release foundation and push against back wall
                    robot.grabFoundation(false);
                    robot.driveAxialVelocity(200, -180, 300, 1, true);

                    // Are we parking?  Where?
                    if (autoConfig.autoOptions.park) {
                        if (autoConfig.autoOptions.parkCloseToWall) {
                            robot.driveLateralVelocity(500, -180, 600, 3,false);
                        } else {
                            robot.driveLateralVelocity(-700, -180, 600, 3,false);
                        }
                        robot.driveAxialVelocity(1200, -180, -1000, 2,true);
                    }
                } else {
                    if (autoConfig.autoOptions.park) {
                        if (autoConfig.autoOptions.parkCloseToWall) {
                            robot.driveAxialVelocity(800, -270, -600, 4,true);
                        } else {
                            robot.driveAxialVelocity(150, -270, -600, 4,true);
                        }
                        robot.turnToHeading(-180, 2);
                        robot.driveAxialVelocity(1000, -180, -1000, 2, true);
                    }
                }
            } else {
                // We never saw a skystone
                if (autoConfig.autoOptions.park) {
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        robot.driveAxialVelocity(800, -270, -600, 4,true);
                    } else {
                        robot.driveAxialVelocity(150, -270, -600, 4,true);
                    }
                    robot.turnToHeading(-180, 2);
                    robot.driveAxialVelocity(1000, -180, -1000, 2, true);
                }
            }

        } else {

            //Parking under bridge if we choose not to score any SkyStones
            if (autoConfig.autoOptions.park) {
                holdForPark();
                if (autoConfig.autoOptions.parkCloseToWall) {
                    //park near Wall
                    robot.driveLateralVelocity(25, 0, 400, 3,  false);
                    robot.driveAxialVelocity(1300, 0, -400, 4,  true);
                } else {
                    //park near bridge code
                    robot.driveLateralVelocity(700, 0, 400, 3,  false);
                    robot.driveAxialVelocity(1300, 0, -400, 4,  true);
                }
            }
        }
    }

    private void startBuildingZone() {
        if (autoConfig.autoOptions.moveFoundation) {
            robot.driveLateralVelocity(300, 0, 300, 4,  false);
            robot.driveAxialVelocity(200, 0, -250, 4,  true);
            robot.turnToHeading(90, 4);
            robot.sleepAndHoldHeading(90, 1);
            robot.driveAxialVelocity(500, 90, -400, 4,  false);
            robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_DOWN_L);
            robot.foundationGrabberRight.setPosition(robot.FOUNDATION_DOWN_R);
            robot.sleepAndHoldHeading(90,1);
            robot.driveAxialVelocity(150, 90, 600, 4,  false);

            if (robot.allianceColor == GFORCE_Hardware.AllianceColor.RED) {
                robot.turnToHeading(0, 4);
                robot.driveAxialVelocity(200, 0, -1000, 1,  false);
                robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
                robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);
                if (autoConfig.autoOptions.park) {
                    holdForPark();
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        // Park by wall
                        robot.driveLateralVelocity(600, 0, -600, 2,  false);
                        robot.driveAxialVelocity(1100, 0, 800, 4,  false);
                    }
                    else {
                        // Park by Bridge
                        // robot.driveLateralVelocity(600, 0, -600, 2, true, false);  // May not need any movement to wards wall
                        robot.driveAxialVelocity(1100, 0, 800, 4,  false);
                    }
                }
            } else {
                robot.turnToHeading(180, 4);
                robot.driveAxialVelocity(200, 180, -1000, 1,  false);
                robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
                robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);
                if (autoConfig.autoOptions.park) {
                    holdForPark();
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        // Park by wall
                        robot.driveLateralVelocity(600, 180, 600, 2, false);
                        robot.driveAxialVelocity(1100, 180, 800, 4, false);
                    }
                    else {
                        // Park by Bridge
                        // robot.driveLateralVelocity(600, 180, 600, 2, true, false); // May not need any movement to wards wall
                        robot.driveAxialVelocity(1100, 180, 800, 4, false);
                    }
                }
            }
        } else if (autoConfig.autoOptions.park) {
            holdForPark();
            if (autoConfig.autoOptions.parkCloseToWall) {
                //park near Wall
                robot.driveLateralVelocity(25, 0, 100, 2, false);
                robot.driveAxialVelocity(800, 0, 1000, 4, true);
            } else {
                //park near bridge code
                robot.driveLateralVelocity(700, 0, 100, 2, false);
                robot.driveAxialVelocity(800, 0, 1000, 4, true);
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