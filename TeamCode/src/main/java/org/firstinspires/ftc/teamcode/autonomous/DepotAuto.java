package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */

@Autonomous(name="Depot Auto", group = "linear opmode")
public class DepotAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {
        double movementNeeded = disengageAndSample(robot, this);

        robot.drivetrain.strafeDistance(-16 + movementNeeded, 1);
        waitForMovement(robot, this, 3);
        turnDegrees(robot,this, 35, .6);
        waitForMovement(robot, this, 3);

        robot.drivetrain.strafeDistance(-5, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.strafeDistance(4, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.driveDistance(-48, 1);
        waitForMovement(robot, this, 2);

        turnDegrees(robot, this, 45, 1);
        waitForMovement(robot, this, .75);

        robot.intake.scoreMarker();
        sleep(500);
        robot.intake.openGate();


        turnDegrees(robot, this, -45, 1);
        waitForMovement(robot, this, 3);

        robot.drivetrain.driveDistance(66, 1);

    }
}
