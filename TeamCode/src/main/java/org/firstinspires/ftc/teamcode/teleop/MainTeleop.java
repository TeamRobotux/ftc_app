package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Created by jack on 9/26/18.
 */

@TeleOp(name="Main Teleop", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //drivetrain controls
            //Move the wheels
            robot.drivetrain.drive360(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // lift controls
            //Lift or Lower lift
            if(gamepad1.right_bumper) {
                robot.lift.setPower(-1);
            }
            else if(gamepad1.left_bumper) {
                robot.lift.setPower(1);
            }
            else {
                robot.lift.setPower(0);
            }


            //Scoop controls
            //Rotate scoop
            if(gamepad1.left_trigger > .25) {
                robot.scoopMotor.setPower(-gamepad1.left_trigger*.55);
            }
            else if(gamepad1.right_trigger > .25) {
                robot.scoopMotor.setPower(gamepad1.right_trigger*.55);
            }
            else {
                robot.scoopMotor.setPower(0);
            }

//
//            //Intake controls
//            //Move intake vertically
//            if(gamepad1.dpad_right) {
//                robot.intake.raiseIntake();
//            }
//            else if(gamepad1.dpad_left) {
//                robot.intake.lowerIntake();
//            }
//            else {
//                robot.intake.moveIntakePerp();
//            }

            /*
            //Extra controls for intake moving up or down
            if(gamepad2.dpad_right) {
                robot.intake.;
            }
            else if(gamepad2.dpad_left) {
                robot.intakeLifter.move(-1);
            }
            else {
                robot.intakeLifter.move(0);
            }
            */

            //Move the intake in and out
            if(gamepad1.dpad_down) {
                robot.intake.retract();
            }
            else if(gamepad1.dpad_up) {
                robot.intake.extend();
            }
            else {
                robot.intake.stopExtension();
            }


            //Move the rotating surgical tubing
            if(gamepad1.a) {
                robot.intake.intake();
            }
            else if(gamepad1.b) {
                robot.intake.outtake();
            }
            else if(gamepad1.y){
                robot.intake.stopIntaking();
            }

//            if(robot.intake.getPulleyPosition() > robot.intake.extensionThreshold) {
//                robot.intake.closeGate();
//            }
//            else {
//                robot.intake.openGate();
//            }

            //Test opmode controls

            telemetry.addData("pulley position", robot.intake.getPulleyPosition());
            telemetry.update();

            sleep(50);
        }

    }

}
