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
        robot.lift.raiseLift();
        waitForMovement(robot.lift, this, 10);

        robot.drivetrain.driveDistance(6);
        sleep(100);
        robot.lift.setPower(-1);
        waitForMovement(robot.drivetrain, this, 5);
        robot.lift.setPower(0);

        robot.drivetrain.strafeDistance(-20);
        waitForMovement(robot, this, 3);

        turnDegrees(robot, this, -90, .5);
        waitForMovement(robot, this, 7);


        /*
        robot.intakeLifter.moveRotation(.225, .4);
        waitForMovement(robot.intakeLifter, this, 1.5);
        robot.intakeLifter.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        */

        robot.drivetrain.driveDistance(26);
        waitForMovement(robot, this, 5);

        /*
        robot.intakeServo.move(1);
        sleep(2000);
        robot.intakeServo.move(0);
        */
//
//        robot.intakeLifter.setPower(.2);
//        sleep(250);
//        robot.intakeLifter.setPower(-.2);
//        sleep(250);
//        robot.intakeLifter.setPower(0);

        turnDegrees(robot, this, 135, .5);
        waitForMovement(robot, this, 10);

        robot.drivetrain.strafeDistance(-25);
        waitForMovement(robot, this, 10);

        robot.drivetrain.driveDistance(55);
        waitForMovement(robot, this, 10);


    }
}
