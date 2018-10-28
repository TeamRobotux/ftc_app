package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/26/18.
 */
@Autonomous(name="Simple Crater", group = "Linear Opmode")
public class SimpleCrater extends AutonomousBasic {

    private final int actionDelay = 250;

    @Override
    void runAutonomous() {
        robot.drivetrain.strafeDistance(-24);
        waitForMovement(robot, this, 5);
        sleep(actionDelay);

        robot.drivetrain.driveDistance(-30);
        waitForMovement(robot, this, 5);
        sleep(actionDelay);

        turnDegrees(robot, this, 135, .5);
        waitForMovement(robot, this, 10);
        sleep(actionDelay);

        robot.drivetrain.driveDistance(40);
        waitForMovement(robot, this, 10);

        robot.intakeServo.move(-1);
        sleep(2);
        robot.intakeServo.move(0);

        robot.drivetrain.driveDistance(-60);
        waitForMovement(robot, this, 10);
    }
}
