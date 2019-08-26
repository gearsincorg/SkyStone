

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This OpMode provides auto scoring for the 2019 4-H REC
 * Created by G-FORCE 4-H Team
 */

@Autonomous(name="Steve Autonomous", group="!Steve")
public class SteveAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Steve robot = new Hardware_Steve();   // Use steve hardware

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this);
        robot.homeBucket();

        // Run menu while waiting to start
        robot.runMenu();

        //Run autonomous routine
        robot.resetHeading();

        // Run autonomous based on selections made in init.
        if (robot.scoreBucket) {
            scoreBucket();
        } else {
            scoreCorner();
        }
    }

    public void scoreCorner() {
        //Drive, turn to corner, dump rings in the corner goal
        robot.driveStraight(76,0,0.75,6);
        robot.turnToHeading(-45,2);
        robot.sleepAndHoldHeading(-45,0.5);
        robot.driveStraight(60,-45,0.75,4);
        robot.dumpRings(2);
        sleep(1000);

        //Drive back from the corner goal towards the chute
        robot.driveStraight(26,-45,-0.75,4);
        robot.homeBucket();
        robot.turnToHeading(0,2);
        robot.driveStraight(98,0,-0.75,6);
    }

    public void scoreBucket() {
        //Drive straight to the bucket, dump rings in the bucket
        robot.driveStraight(114,0,0.5,12);
        robot.prepareRings(2);
        robot.driveStraight(5,0,0.5,2);
        robot.dumpRings(2);

        //Turn the robot from side to side to shake the rings out
        sleep(1500);
        robot.turnToHeading(10,1);
        robot.turnToHeading(-10,1);
        robot.releaseRings(1);

        //Drive back from the bucket towards the chute
        robot.driveStraight(4,0,-0.25,2);
        robot.homeBucket();
        robot.driveStraight(72,20,-0.75,8);
        robot.driveStraight(42,0,-0.75,5);
    }
}