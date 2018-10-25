package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            robot.drivetrain.drive360(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // lift controls
            if(gamepad1.right_trigger > .5) {
                robot.lift.setPower(-1);
            }
            else if(gamepad1.left_trigger > .5) {
                robot.lift.setPower(1);
            }
            else {
                robot.lift.setPower(0);
            }


            //Scoop controls
            if(gamepad1.left_bumper) {
                robot.scoopMotor.setPower(-.6);
            }
            else if(gamepad1.right_bumper) {
                robot.scoopMotor.setPower(.6);
            }
            else {
                robot.scoopMotor.setPower(0);
            }

            //Intake controls
            if(gamepad1.dpad_right) {
                robot.intakeLifter.setPower(.6);
            }
            else if(gamepad1.dpad_left) {
                robot.intakeLifter.setPower(-.6);
            }
            else {
                robot.intakeLifter.setPower(0);
            }

            if(gamepad1.dpad_up) {
                robot.intakePulley.setPower(.75);
            }
            else if(gamepad1.dpad_down) {
                robot.lift.setPower(-.75);
            }
            else {
                robot.lift.setPower(0);
            }

            if(gamepad1.a) {
                robot.intakeServo.move(-1);
            }
            else if(gamepad1.b) {
                robot.intakeServo.move(1);
            }
            else if(gamepad1.y){
                robot.intakeServo.move(0);
            }

            sleep(50);
        }
    }

}
