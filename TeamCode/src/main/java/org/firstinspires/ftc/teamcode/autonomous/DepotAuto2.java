package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */

@Autonomous(name="Depot Auto New", group = "linear opmode")
public class DepotAuto2 extends AutonomousBasic {

    @Override
    void runAutonomous() {
        double movementNeeded = disengageAndSample(robot, this);
        robot.drivetrain.strafeDistance(-16 + movementNeeded, .75);
        waitForMovement(robot, this, 1);
        turnDegrees(robot,this, 135, .6);
        waitForMovement(robot, this, 1);
        robot.drivetrain.strafeDistance(-30, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.strafeDistance(4, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.driveDistance(-24, 1);
        waitForMovement(robot, this, 1);

        robot.markerServo.moveTo(robot.markerScore);
        sleep(1000);
        robot.markerServo.moveTo(robot.markerVert);

        robot.drivetrain.driveDistance(39, 1);
        waitForMovement(robot, this, 5);

        robot.drivetrain.strafeDistance(-12, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.strafeDistance(4, 1);
        waitForMovement(robot, this, .75);

        robot.drivetrain.driveDistance(39, 1);
        waitForMovement(robot, this, 5);

        robot.intake.extend();
        sleep(500);
        robot.intake.stopExtension();
    }
}
