package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */

@Autonomous(name="Depot Auto", group = "linear opmode")
public class DepotAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {
        double movementNeeded = disengageAndSampleDepot(robot, this);

        robot.drivetrain.driveDistance(10, 1);
        waitForMovement(robot, this, .75);

        turnDegrees(robot, this, 135, .5);
        waitForMovement(robot, this, 1.5);

        robot.drivetrain.strafeDistance(-movementNeeded/Math.sqrt(2) +20 ,1);
        waitForMovement(robot, this, .75);

        //robot.drivetrain.driveDistance(-(movementNeeded+40)*.3, 1);
        //waitForMovement(robot, this, 1);

        robot.markerServo.moveTo(robot.markerScore);
        sleep(1000);

        robot.drivetrain.strafeDistance(-5, .6);
        waitForMovement(robot, this, 1);

        robot.drivetrain.driveDistance(72, 1);
        sleep(2000);
        robot.intake.extend();
        waitForMovement(robot, this, 10);
        sleep(1000);
        robot.intake.stopExtension();
    }
}
