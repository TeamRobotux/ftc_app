package org.firstinspires.ftc.teamcode;

/**
 * Created by jack on 10/27/18.
 */

public class CraterAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {

        robot.lift.setPower(-1);
        sleep(4200);
        robot.lift.setPower(0);

        robot.drivetrain.driveDistance(6);
        sleep(100);
        robot.lift.setPower(-1);
        waitForMovement(robot.drivetrain, this, 5);
        robot.lift.setPower(0);

    }
}
