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

    int skyStonePosition = 0;
    boolean isRed;

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
        isRed = autoConfig.autoOptions.redAlliance;

        if (autoConfig.autoOptions.redAlliance) {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.RED;
        } else {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.BLUE;
        }

        //Starting autonomous reset heading to zero
        robot.resetHeading();
        robot.readSensors();
        robot.lockLiftInPlace();
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
        double axialDistance;
        double lateralDistance;
        double blockCenter;  // Offset from tile center

        //Are we doing any SkyStones?
        if (autoConfig.autoOptions.scoreFirstSkyStone || autoConfig.autoOptions.scoreBothSkyStones) {

            // Move to where you should be able to see skystones 2 or 3 and release collectors
            robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
            robot.beginReleaseCollectorArms();
            robot.driveLateralVelocity(500, 0, 800, 2,  false);

            // Can we see skystones 2 or 3:
            if (nav.waitForTarget(0.5)) {

                if (isRed) {
                    blockCenter = 220 + nav.robotY;  // Offset from depot to tile center  Y is +ve to the right
                } else {
                    blockCenter = 220 - nav.robotY;  // Offset from depot to tile center  Y is -ve to the left
                }

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

            //Get the SkyStone if we found it, or make out it's in Pos 3
            if (skyStonePosition > 0) {

                if (isRed) {
                    axialDistance = (390 + nav.robotY);
                } else {
                    axialDistance = (390 - nav.robotY);
                }

                lateralDistance = Math.abs(nav.robotX) - 170;
                robot.driveLateralVelocity(lateralDistance, 0, 600, 3, false);
                robot.driveAxialVelocity(axialDistance, 0, -600, 3, true);
            } else {
                lateralDistance = 145;
                robot.driveLateralVelocity(lateralDistance, 0, 600, 3, false);
                skyStonePosition = 3;
            }

            sleep(100);
            robot.turnToHeading(isRed ? -45 : -135,2);
            robot.transferStone(1);
            robot.runCollector(1);

            // Collect Skystone
            robot.driveAxialVelocity(350,isRed ? -45 : -135,300,2, false);
            robot.driveAxialVelocity(350,isRed ? -45 : -135,-400,2,false);
            robot.runCollector(0.25);
            robot.turnToHeading(isRed ? 0 : -180,2);
            robot.runCollector(0);

            // Drive to Foundation
            robot.driveAxialVelocity(1150 + (200 * skyStonePosition),isRed ? 0 : -180,-1600,3,false);
            robot.grabStone(true);
            robot.stoneInGrasp = true;
            robot.turnToHeading(isRed ? 90 : -270,1.5);
            robot.transferStone(0);
            robot.driveAxialVelocity(300,isRed ? 90 : -270,-400,2,false);
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
                robot.driveAxialVelocity(600, isRed ? 90 : -270, 600, 3, false);
                robot.turnToHeading(isRed ? 0 : -180, 3);

                // Release foundation and push against back wall
                robot.grabFoundation(false);
                robot.driveAxialVelocity(200, isRed ? 0 : -180, -500, 1, false);

                // Are we parking?  Where?
                if (autoConfig.autoOptions.park) {
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        robot.driveLateralVelocity(475, isRed ? 0 : -180, -600, 1.5,true);
                        robot.driveAxialVelocity(1000, isRed ? 0 : -180, 1200, 2,false);
                    } else {
                        robot.driveLateralVelocity(100, isRed ? 0 : -180, 400, 1.5,true);
                        robot.driveAxialVelocity(1000, isRed ? 0 : -180, 1200, 2,false);
                    }
                }
            } else {
                // Do not pull the foundation.
                if (autoConfig.autoOptions.park) {
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        robot.driveAxialVelocity(650, isRed ? 90 : -270, 600, 4,false);
                    } else {
                        robot.driveAxialVelocity(150, isRed ? 90 : -270, 600, 4,false);
                    }
                    robot.turnToHeading(0, 2);
                    robot.driveAxialVelocity(1100, 0, 1000, 3, true);
                }
            }

        } else {

            //Parking under bridge if we choose not to score any SkyStones
            if (autoConfig.autoOptions.park) {
                holdForPark();
                if (autoConfig.autoOptions.parkCloseToWall) {
                    //park near Wall
                    robot.driveLateralVelocity(25, 0, 600, 1,  false);
                    robot.driveAxialVelocity(900, 0, -600, 3,  true);
                } else {
                    //park near bridge code
                    robot.driveLateralVelocity(700, 0, 800, 2,  false);
                    robot.driveAxialVelocity(900, 0, -600, 3,  true);
                }
            }
        }
    }

    private void startBuildingZone() {
        if (autoConfig.autoOptions.moveFoundation) {
            robot.beginReleaseCollectorArms();

            robot.driveLateralVelocity(300, 0, 600, 3,  false);
            robot.driveAxialVelocity(200, 0, -400, 4,  true);
            robot.turnToHeading(90, 4);
            robot.sleepAndHoldHeading(90, 0.5);
            robot.driveAxialVelocity(500, 90, -400, 4,  false);
            robot.grabFoundation(true);
            robot.sleepAndHoldHeading(90,1);
            robot.driveAxialVelocity(400, 90, 800, 2,  false);

            if (isRed) {
                robot.turnToHeading(0, 4);
                robot.grabFoundation(false);
                robot.driveAxialVelocity(200, 0, -1000, 1,  false);
                if (autoConfig.autoOptions.park) {
                    holdForPark();
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        // Park by wall
                        robot.driveLateralVelocity(550, 0, -600, 2,  false);
                        robot.driveAxialVelocity(900, 0, 800, 3,  false);
                    }
                    else {
                        // Park by Bridge
                        //robot.driveLateralVelocity(50, 0, -600, 1, false);
                        robot.driveAxialVelocity(900, 0, 800, 3,  false);
                    }
                }
            } else {
                robot.turnToHeading(180, 4);
                robot.grabFoundation(false);
                robot.driveAxialVelocity(200, 180, -1000, 1,  false);
                if (autoConfig.autoOptions.park) {
                    holdForPark();
                    if (autoConfig.autoOptions.parkCloseToWall) {
                        // Park by wall
                        robot.driveLateralVelocity(550, 180, 600, 2, false);
                        robot.driveAxialVelocity(900, 180, 800, 3, false);
                    }
                    else {
                        // Park by Bridge
                        //robot.driveLateralVelocity(50, 180, -600, 1, false);
                        robot.driveAxialVelocity(900, 180, 800, 3, false);
                    }
                }
            }
        } else if (autoConfig.autoOptions.park) {
            holdForPark();
            if (autoConfig.autoOptions.parkCloseToWall) {
                //park near Wall
                robot.driveLateralVelocity(25, 0, 600, 1, false);
                robot.driveAxialVelocity(850, 0, 800, 4, true);
            } else {
                //park near bridge code
                robot.driveLateralVelocity(700, 0, 800, 2, false);
                robot.driveAxialVelocity(850, 0, 800, 4, true);
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