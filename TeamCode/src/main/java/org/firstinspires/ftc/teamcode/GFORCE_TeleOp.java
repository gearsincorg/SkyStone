/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Teleop opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="G-FORCE Teleop", group="!Competition")
public class GFORCE_TeleOp extends LinearOpMode {

    public final double AXIAL_JS_SCALE = 0.5;
    public final double LATERAL_JS_SCALE = 0.5;
    public final double YAW_JS_SCALE = 0.25;

    private ElapsedTime neutralTime = new ElapsedTime();

    /* Declare OpMode members. */
    GFORCE_Hardware robot = new GFORCE_Hardware();

    @Override
    public void runOpMode() {
        double forwardBack;
        double rotate;
        double rightLeft;

        double axialVel;
        double yawVel;
        double lateralVel;

        double desiredHeading = 0;
        boolean neutralSticks = true;
        boolean autoHeadingOn = false;

        /* Initialize the hardware variables.
         * The init() method of the Hardware class does all the work here
         */
        robot.init(this);

        // Wait for the game to start (Driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        waitForStart();
        robot.startMotion();

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions
            controlBlockScoring();

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            forwardBack = -gamepad1.left_stick_y;
            // forwardBack = forwardBack * forwardBack * Math.signum(forwardBack);
            forwardBack *= AXIAL_JS_SCALE;

            rightLeft = gamepad1.left_stick_x;
            // rightLeft = rightLeft * rightLeft * Math.signum(rightLeft);
            rightLeft *= LATERAL_JS_SCALE;

            rotate = -gamepad1.right_stick_x;
            // rotate = rotate * rotate * Math.signum(rotate);
            rotate *= YAW_JS_SCALE;

            // Jog controls
            if (gamepad1.dpad_up) {
                forwardBack = 0.1;
            } else if (gamepad1.dpad_down) {
                forwardBack = -0.1;
            } else if (gamepad1.x) {
                rotate = 0.1;
            } else if (gamepad1.b) {
                rotate = -0.1;
            } else if (gamepad1.dpad_left) {
                rightLeft = -0.2;
            } else if (gamepad1.dpad_right) {
                rightLeft = 0.2;
            }

            //Field Centric Motion
            axialVel = -((forwardBack * Math.sin(Math.toRadians(robot.currentHeading))) +
                    (rightLeft * Math.cos(Math.toRadians(robot.currentHeading))));

            lateralVel = (forwardBack * Math.cos(Math.toRadians(robot.currentHeading))) -
                    (rightLeft * Math.sin(Math.toRadians(robot.currentHeading)));

            //Scale velocities to mm per second
            axialVel *= robot.MAX_VELOCITY_MMPS;
            lateralVel *= robot.MAX_VELOCITY_MMPS;
            yawVel = rotate * robot.MAX_VELOCITY_MMPS;

            // Control Yaw, using manual or auto correction
            if (rotate != 0) {
                // We are turning with the joystick
                autoHeadingOn = false;
            } else if (!autoHeadingOn && robot.notTurning()) {
                // We have just stopped turning, so lock in current heading
                desiredHeading = robot.currentHeading;
                autoHeadingOn = true;
            }

            robot.setAxialVelocity(axialVel);
            robot.setLateralVelocity(lateralVel);

            // Disable correction if JS are neutral for more than 2 seconds
            neutralSticks = ((forwardBack == 0) && (rightLeft == 0) &&  (rotate == 0));
            if (!neutralSticks)
                neutralTime.reset();

            if (autoHeadingOn && (neutralTime.time() < 3)) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
                desiredHeading = robot.currentHeading;
            }

            robot.moveRobotVelocity();

            //Extend the Block if either control is pressed
            robot.extendStone((gamepad1.left_trigger > 0.5) || gamepad1.left_bumper);

            //Release the capstone if both controls are pressed
            robot.releaseCapstone((gamepad1.left_trigger > 0.5) && gamepad1.left_bumper);

            //Move the SkyStone Grabbers (Temporary test code)
            if(gamepad1.right_bumper) {
                robot.setBlueSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
            } else {
                robot.setBlueSkystoneGrabber(SkystoneGrabberPositions.START);
            }

            if(gamepad1.right_trigger > 0.5) {
                robot.setRedSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
            } else {
                robot.setRedSkystoneGrabber(SkystoneGrabberPositions.START);
            }

            // Send telemetry message to signify robot running
            robot.showEncoders();
        }

    }

    //Co-pilot Controls
    //Lift Code
    private void controlBlockScoring() {
        double heightError = 0;

        if (gamepad2.back && gamepad2.start) {
            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.x) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setPower(-0.10);
            robot.rightLift.setPower(0);
        } else if (gamepad2.b) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(-0.10);

        } else if (gamepad2.y && (robot.leftLiftAngle < 42)) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftLift.setPower(1);
            robot.rightLift.setPower(1);

        } else if (gamepad2.a && (robot.leftLiftAngle > 10)) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (robot.leftLiftAngle > 15) {
                robot.leftLift.setPower(-0.9);
                robot.rightLift.setPower(-0.9);
            } else {
                robot.leftLift.setPower(-0.3);
                robot.rightLift.setPower(-0.3);
            }

        } else {
            // Try to keep the block level if it is up in the air.
            if (robot.leftLiftAngle > 12) {
                robot.leftLift.setPower(0.0);
                heightError = (robot.leftLiftAngle - robot.rightLiftAngle);

                if (heightError > 0.1) {
                    robot.rightLift.setPower(0.1);
                }
                else if (heightError <  -0.1) {
                    robot.rightLift.setPower(-0.1);
                }
                else {
                    robot.rightLift.setPower(0);
                }
            }
            else {
                robot.leftLift.setPower(0.0);
                robot.rightLift.setPower(0.0);
            }
        }

        //Run Collector and Transfer Wheels
        if (gamepad2.right_trigger > 0.5) {
            robot.runCollector(1);
            robot.transferStone(0.75);
        } else if (gamepad2.left_trigger > 0.5) {
            robot.runCollector(-0.5);
            robot.transferStone(-0.75);
        } else {
            robot.runCollector(0);
            robot.transferStone(0);
        }

        //Grab the stone
        if(gamepad2.right_bumper) {
            robot.grabStone(true);
        } else if (gamepad2.left_bumper) {
            robot.grabStone(false);
        }

    }
}
