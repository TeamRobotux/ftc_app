package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by jack on 10/27/18.
 */

@Autonomous(name="Depot Auto", group = "linear opmode")
public class DepotAuto extends AutonomousBasic {

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

        robot.drivetrain.strafeDistance(-10);
        waitForMovement(robot, this, 3);

        turnDegrees(robot, this, -90, .5);
        waitForMovement(robot, this, 7);

        robot.intakeLifter.moveRotation(.225, .4);
        waitForMovement(robot.intakeLifter, this, 1.5);
        robot.intakeLifter.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.drivetrain.driveDistance(34);
        waitForMovement(robot, this, 5);

        robot.intakeServo.move(1);
        sleep(2000);
        robot.intakeServo.move(0);

        robot.intakeLifter.moveRotation(-.225,6);
        waitForMovement(robot.intakeLifter, this, 1.5);

        turnDegrees(robot, this, 135, .5);
        waitForMovement(robot, this, 10);

        robot.drivetrain.strafeDistance(-25);
        waitForMovement(robot, this, 10);

        robot.drivetrain.driveDistance(50);
        waitForMovement(robot, this, 10);


    }
}
