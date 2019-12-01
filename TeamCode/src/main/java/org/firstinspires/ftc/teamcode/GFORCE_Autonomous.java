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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;


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

@Autonomous(name="G-FORCE Autonomous", group="!Competition")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig   autoConfig    = new AutoConfig();
    GFORCE_Hardware     robot         = new GFORCE_Hardware();
    GFORCE_Navigation   nav           = new GFORCE_Navigation();

    final double STUD_RANGE = 10;               // Plus or minus 10 millimeters
    final double BLOCK_PUSH_DISTANCE = 10;      // Extra 10mm
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

        robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
        robot.driveLateralVelocity(600, 0, 1000, 20, true);
        robot.sleepAndHoldHeading(0, 0.5);

        if (nav.waitForTarget(5)) {
            skyStonePosition = 1;
        } else {
            robot.driveAxialVelocity(200,0,380,5,true);
            if (nav.waitForTarget(5)) {
                skyStonePosition = 2;
            }
            else {
                robot.driveAxialVelocity(200,0,380,5,true);
                if (nav.waitForTarget(5)) {
                    skyStonePosition = 3;
                }
            }
        }


        if (skyStonePosition > 0) {
            driveAndGrabBlock();
            robot.driveAxialVelocity(700 + (skyStonePosition * 200), 0, -1250, 20, true); //Should be 700mm
            robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
            robot.sleepAndHoldHeading(0, 1);
            robot.driveAxialVelocity( 1500 + (skyStonePosition * 200), 0, 1250, 20, true);
            robot.sleepAndHoldHeading(0, 0.5);
            robot.driveLateralVelocity(300,0,600,2,true);

            if (nav.waitForTarget(5)) {
                driveAndGrabBlock();
                robot.driveAxialVelocity(1500 + (skyStonePosition * 200), 0, -1250, 20, true);
                robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
                robot.driveAxialVelocity(600, 0, 1250, 20, true);
                robot.sleepAndHoldHeading(0, 1);
            }
        }

        while(opModeIsActive()) {
            nav.waitForTarget(1);
            nav.showNavTelemetry(true);
        }
    }

    public void driveAndGrabBlock() {
        //Multiply the distance in mm from the target by the conversion factor for inches, flip the direction to correct for the error
        error = (BLOCK_CENTER_OFFSET - nav.robotY);

        RobotLog.ii(TAG, String.format("Error=%5.0f robotY=%5.0f ", error, nav.robotY));
        robot.driveAxialVelocity(error, 0, 380,5,true);

        //Calculate new error
        error = (-nav.robotX);
        robot.driveLateralVelocity(error + BLOCK_PUSH_DISTANCE, 0, 380, 5, true);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(400, 0, -600, 20, true);
    }

}