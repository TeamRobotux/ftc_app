package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by jack on 4/15/18.
 */

@TeleOp(name="Test Teleop", group="Pushbot")
public class TestTeleop extends LinearOpMode {

    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware
    double driveIn;
    double turnIn;
    double strafeIn;

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.wheels.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.gyro.composeTelemetry(telemetry);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveIn = -gamepad1.left_stick_y;
            turnIn  =  gamepad1.right_stick_x;
            strafeIn = gamepad1.left_stick_x;

            double magnitude = Math.sqrt(Math.pow(driveIn, 2) + Math.pow(strafeIn, 2));

            double angle = Math.atan2(driveIn, strafeIn); //strafing right = 0, forward = pi/2

            //robot.wheels.driveSinglePower(magnitude*Math.sin(angle + Math.PI) + turnIn,0); //back //forward
            //robot.wheels.driveSinglePower(magnitude*Math.sin(angle) - turnIn,1); //forward, forward
            //robot.wheels.driveSinglePower(magnitude*Math.sin(angle + Math.PI) - turnIn,2); //back, forward
            //robot.wheels.driveSinglePower(magnitude*Math.sin(angle) + turnIn,3); //forw, forward

            robot.wheels.driveSinglePower(driveIn + strafeIn + 2*turnIn, 0);
            robot.wheels.driveSinglePower(driveIn - strafeIn - 2*turnIn, 1);
            robot.wheels.driveSinglePower(driveIn + strafeIn - 2*turnIn, 2);
            robot.wheels.driveSinglePower(driveIn - strafeIn + 2*turnIn, 3);
        }
    }
}
