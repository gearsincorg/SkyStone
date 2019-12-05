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

    public final double AXIAL_JS_SCALE      = 0.5;
    public final double LATERAL_JS_SCALE    = 0.5;
    public final double YAW_JS_SCALE        = 0.25;

    private ElapsedTime neutralTime = new ElapsedTime();

    /* Declare OpMode members. */
    GFORCE_Hardware robot = new GFORCE_Hardware();   // Use steve hardware

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
         * The init() method of the hardware class does all the work here
         */
        robot.init(this);

        //robot.homeArm();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        waitForStart();
        robot.startMotion();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions
            controlBlockScoring();

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

            if (gamepad1.y || gamepad1.dpad_up) {
                forwardBack = 0.1;
            } else if (gamepad1.a || gamepad1.dpad_down) {
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

            // Convert to axis velocities
            axialVel   = forwardBack * robot.MAX_VELOCITY_MMPS;
            lateralVel = rightLeft * robot.MAX_VELOCITY_MMPS;
            yawVel     = rotate * robot.MAX_VELOCITY_MMPS;

            //-------------------------
            // Determine Robot Centric motion (based on gyro heading)
            // axial = (forwardBack * Math.cos(Math.toRadians(currentHeading))) -
            //         (rightLeft * Math.sin(Math.toRadians(currentHeading)));
            // lateral = (forwardBack * Math.sin(Math.toRadians(currentHeading))) +
            //         (rightLeft * Math.cos(Math.toRadians(currentHeading)));

            neutralSticks = ((forwardBack == 0) &&
                    (rightLeft == 0) &&
                    (rotate == 0)
            );

            if (rotate != 0) {
                // We are turning with joystick.
                autoHeadingOn = false;
            } else if (!autoHeadingOn && robot.notTurning()) {
                // We have just stopped turning, so lock in current heading
                desiredHeading = robot.currentHeading;
                autoHeadingOn = true;
            }

            // Control Yaw, using manual or auto correction
            // disable correction if JS are neutral for more than 2 seconds
            if (!neutralSticks)
                neutralTime.reset();

            robot.setAxialVelocity(axialVel);
            robot.setLateralVelocity(lateralVel);

            if (autoHeadingOn && (neutralTime.time() < 4)) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
                desiredHeading = robot.currentHeading;
            }

            robot.moveRobotVelocity();

            // Send telemetry message to signify robot running;
            robot.showEncoders();
        }
    }

    private void controlBlockScoring() {

        // Check for arm up
        if (robot.armAngle < 120 && (gamepad2.dpad_up || (gamepad2.right_stick_y < -0.1))) {
            if (gamepad2.dpad_up) {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(0.2);
            } else {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(Math.abs(gamepad2.right_stick_y) * 0.3);
            }
        }

        // Check for Arm Down
        else if (robot.armAngle > -110 && (gamepad2.dpad_down || (gamepad2.right_stick_y > 0.1))) {
            if (gamepad2.dpad_down) {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(-0.2);
            } else {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(Math.abs(gamepad2.right_stick_y) * -0.3);
            }
        }

        // Hold the current position
        else {
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (robot.armAngle < -100) {
                robot.arm.setPower(0);
            } else {
                if (Math.abs(robot.armAngle) > 10) {
                    robot.arm.setPower(Math.signum(robot.armAngle) * -0.1);
                } else {
                    robot.arm.setPower(0);
                }
            }
        }

        if (gamepad2.y && (robot.liftAngle < 55))
            robot.lift.setPower(0.3);
        else if (gamepad2.a && (robot.liftAngle > 10 ))
            robot.lift.setPower(-0.2);
        else
            robot.lift.setPower(0);

        if(gamepad2.x) {
            robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_DOWN_L);
            robot.foundationGrabberRight.setPosition(robot.FOUNDATION_DOWN_R);
        }
        else if (gamepad2.b) {
            robot.foundationGrabberLeft.setPosition(robot.FOUNDATION_SAFE_L);
            robot.foundationGrabberRight.setPosition(robot.FOUNDATION_SAFE_R);
        }

        if (gamepad2.right_bumper)
            robot.stoneGrab.setPosition(robot.STONE_CLOSE);
        else if (gamepad2.left_bumper)
            robot.stoneGrab.setPosition(robot.STONE_OPEN);
        else
            robot.collect.setPower(0);

        if (gamepad2.right_trigger > 0.5) {
            robot.collect.setPower(1);

        } else if (gamepad2.left_trigger > 0.5) {
            robot.collect.setPower(-1);

        }

    }

}
