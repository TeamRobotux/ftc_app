package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */
@Autonomous(name = "crater auto", group = "Linear Opmode")
public class CraterAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {

        robot.lift.raiseLift();
        waitForMovement(robot.lift, this, 10);

        robot.drivetrain.driveDistance(6);
        sleep(100);
        robot.lift.setPower(-1);
        waitForMovement(robot.drivetrain, this, 5);
        robot.lift.setPower(0);

        robot.drivetrain.strafeDistance(-10);
        waitForMovement(robot, this, 3);

    }
}
