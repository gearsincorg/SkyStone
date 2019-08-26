

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode provides auto scoring for the 2019 4-H REC
 * Created by G-FORCE 4-H Team
 */

@TeleOp(name="Steve Teleop", group="!Steve")
public class SteveTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Steve robot = new Hardware_Steve();   // Use steve hardware

    @Override
    public void runOpMode() {
        double forwardBack;
        double rotate;

        double axial;
        double yaw;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this);
        robot.homeBucket();

        // Run menu while waiting to start
        robot.runMenu();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            forwardBack = -gamepad1.left_stick_y;
            forwardBack = forwardBack * forwardBack * Math.signum(forwardBack);

            rotate = -gamepad1.right_stick_x;
            rotate = rotate * rotate * Math.signum(rotate);

            forwardBack *= robot.AXIAL_SCALE;
            rotate *= robot.YAW_SCALE;

            // Provide some SLOW BUTTONS for gentle movement
            if (gamepad1.y || gamepad1.dpad_up) {
                forwardBack = 0.35;
            } else if (gamepad1.a || gamepad1.dpad_down) {
                forwardBack = -0.35;
            } else if (gamepad1.x || gamepad1.dpad_left) {
                rotate = 0.25;
            } else if ( gamepad1.b || gamepad1.dpad_right
            ) {
                rotate = -0.25;
            }

            axial = forwardBack;
            yaw = rotate;

            robot.moveRobot(axial, yaw);

            // Move Bucket based on gamepad buttons.
            // Restrict movement to within legal encoder values
            if (((gamepad1.right_trigger > 0.5) || gamepad1.right_bumper)&& (robot.arm.getCurrentPosition() > robot.DUMP_LIMIT)) {
                robot.arm.setPower(robot.ARM_DUMP);
            }
            else if (((gamepad1.left_trigger > 0.5) || gamepad1.left_bumper) && (robot.arm.getCurrentPosition() < robot.COLLECT_LIMIT)) {
                robot.arm.setPower(robot.ARM_COLLECT);
            }
            else {
                robot.arm.setPower(robot.ARM_STOP);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", robot.leftDrive.getPower());
            telemetry.addData("right", "%.2f", robot.rightDrive.getPower());
            telemetry.update();
        }
    }
}
