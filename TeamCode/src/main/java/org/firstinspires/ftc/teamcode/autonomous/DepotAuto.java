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

        robot.drivetrain.driveDistance(20, 1);
        waitForMovement(robot, this, .75);

        robot.drivetrain.strafeDistance(movementNeeded + 12, 1);
        waitForMovement(robot, this, .75);

        turnDegrees(robot, this, -135, .6);
        waitForMovement(robot, this, 1.5);

        robot.markerServo.moveTo(robot.markerScore);
        sleep(1000);
        robot.markerServo.moveTo(robot.markerVert);

        robot.drivetrain.strafeDistance(-10, .6);
        waitForMovement(robot, this, 1);

        robot.drivetrain.driveDistance(-66, 1);
        waitForMovement(robot, this, 10);
    }
}
