/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="G-FORCE Teleop", group="!Steve")
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
        double currentHeading = 0;

        boolean neutralSticks = true;
        boolean autoHeadingOn = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this);

        //robot.homeArm();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Home the Bucket");
        telemetry.update();

        waitForStart();
        robot.startMotion();

        // run until the end of the match (driver presses STOP)
        robot.startMotion();
        while (opModeIsActive()) {

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
            } else if (gamepad1.x || gamepad1.dpad_left) {
                rightLeft = 0.1;
            } else if ( gamepad1.b || gamepad1.dpad_right
            ) {
                rightLeft = -0.1;
            }

            // Convert to axis velocities
            axialVel   = forwardBack * robot.MAX_VELOCITY_IPS;
            lateralVel = rightLeft * robot.MAX_VELOCITY_IPS;
            yawVel     = rotate * robot.MAX_ROTATION_DPS;

            //-------------------------
            // Determine Robot Centric motion (based on gyro heading)
            // axial = (forwardBack * Math.cos(Math.toRadians(currentHeading))) -
            //         (rightLeft * Math.sin(Math.toRadians(currentHeading)));
            // lateral = (forwardBack * Math.sin(Math.toRadians(currentHeading))) +
            //         (rightLeft * Math.cos(Math.toRadians(currentHeading)));

            //determine desired Yaw rate
            currentHeading = robot.getHeading();

            neutralSticks = ((forwardBack == 0) &&
                    (rightLeft == 0) &&
                    (rotate == 0)
            );

            if (rotate != 0) {
                // We are turning with joystick.
                autoHeadingOn = false;
            } else if (!autoHeadingOn && robot.notTurning()) {
                // We have just stopped turning, so lock in current heading
                desiredHeading = currentHeading;
                autoHeadingOn = true;
            }

            // Control Yaw, using manual or auto correction
            // disable correction if JS are neutral for more than 2 seconds
            if (!neutralSticks)
                neutralTime.reset();

            robot.setAxialVelocity(axialVel);
            robot.setLateralVelocity(lateralVel);

            if (autoHeadingOn) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
            }

            robot.moveRobotVelocity();

            // Send telemetry message to signify robot running;
            robot.updateMotion();
            robot.showEncoders();
        }
    }

}
