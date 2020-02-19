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

    public final double SLOW_AXIAL_JS_SCALE = 0.2;
    public final double NORMAL_AXIAL_JS_SCALE = 0.6;
    public final double FAST_AXIAL_JS_SCALE = 0.75;

    public final double SLOW_LATERAL_JS_SCALE = 0.2;
    public final double NORMAL_LATERAL_JS_SCALE = 0.6;
    public final double FAST_LATERAL_JS_SCALE = 0.75;

    public final double SLOW_YAW_JS_SCALE = 0.15;
    public final double NORMAL_YAW_JS_SCALE = 0.25;
    public final double FAST_YAW_JS_SCALE = 0.25;


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
        robot.lockLiftInPlacet();
        robot.startMotion();

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions
            //controlBlockScoring();
            robot.runLiftControl();
            robot.craneControl();

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            forwardBack = -gamepad1.left_stick_y;
            rightLeft = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            //Three different speeds: slow, medium, and fast depending on what trigger the driver is holding

            if (gamepad1.left_trigger > 0.5) {
                forwardBack *= SLOW_AXIAL_JS_SCALE;
                rightLeft *= SLOW_LATERAL_JS_SCALE;
                rotate *= SLOW_YAW_JS_SCALE;
            } else {
                forwardBack *= NORMAL_AXIAL_JS_SCALE;
                rightLeft *= NORMAL_LATERAL_JS_SCALE;
                rotate *= NORMAL_YAW_JS_SCALE;
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

            //Grab Foundation
            robot.grabFoundation(gamepad1.right_trigger > 0.5);

            //Co-pilot Lift Controls
            if (gamepad2.y) {
                robot.setLiftSetpoint(45);
            }

            if (gamepad2.b) {
                robot.setLiftSetpoint(26);
            }

            if (gamepad2.a) {
                robot.setLiftSetpoint(10);
            }

            // Send telemetry message to signify robot running
            robot.showEncoders();
        }

    }




    //Lift Code
    /*
    private void controlBlockScoring() {
        double heightError = 0;

        if (gamepad2.back && gamepad2.start) {
            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.y) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setPower(-0.10);
            robot.rightLift.setPower(0);
        } else if (gamepad2.a) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(-0.10);

        } else if (gamepad2.dpad_up && (robot.leftLiftAngle < 42)) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftLift.setPower(1);
            robot.rightLift.setPower(1);

        } else if (gamepad2.dpad_down && (robot.leftLiftAngle > 10)) {
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

    }
    */
}
