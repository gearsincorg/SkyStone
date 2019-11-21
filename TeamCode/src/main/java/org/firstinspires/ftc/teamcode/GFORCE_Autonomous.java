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



@Autonomous(name="G-FORCE Autonomous", group="!Steve")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig   autoConfig    = new AutoConfig();
    GFORCE_Hardware     robot         = new GFORCE_Hardware();
    GFORCE_Navigation   nav           = new GFORCE_Navigation();

    @Override
    public void runOpMode() {

        boolean driveOK;

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext,this);
        robot.init(this);
        // nav.init(this,robot);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
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
        robot.driveLateralVelocity(18,0,50,20,true);
        robot.driveLateralVelocity(12,0,15,20,true);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(12,0,-25,20,true);
        robot.driveAxialVelocity(34,0,-50,20,true);  // 6" shorter than actual move ??
        robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveAxialVelocity(64,0,50,20,true);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(12,0,15.5,20,true);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.GRAB_DOWN);
        robot.sleepAndHoldHeading(0, 1);
        robot.driveLateralVelocity(12,0,-25,20,true);
        robot.driveAxialVelocity(64,0,-50,20,true);
        robot.setSkystoneGrabber(SkystoneGrabberPositions.START);
        robot.driveAxialVelocity(6,0,50,20,true);
        robot.sleepAndHoldHeading(0, 1);
        robot.showEncoders();

        while(opModeIsActive()) {

        }
    }
}