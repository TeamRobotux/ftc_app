package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */
@Autonomous(name = "crater auto", group = "Linear Opmode")
public class CraterAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {
        double movementNeeded = disengageAndSample(robot, this);
        robot.drivetrain.strafeDistance(-16 + movementNeeded, .75);
        waitForMovement(robot, this, 1);
        turnDegrees(robot,this, 35, .6);
        waitForMovement(robot, this, 1);
        robot.drivetrain.strafeDistance(-30, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.driveDistance(-48, 1);
        waitForMovement(robot, this, 1);

        robot.markerServo.moveTo(robot.markerScore);
        sleep(1000);
        robot.markerServo.moveTo(robot.markerVert);

        robot.drivetrain.driveDistance(66, 1);
        robot.intake.extend();
        waitForMovement(robot, this, 10);


        //for simple crater
//
//        robot.drivetrain.driveDistance(28,1);
//        waitForMovement(robot.drivetrain, this, 1);
    }
}
