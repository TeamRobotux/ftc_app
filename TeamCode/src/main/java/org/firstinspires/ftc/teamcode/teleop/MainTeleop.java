package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.IsBusy;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Created by jack on 9/26/18.
 */

@TeleOp(name="Main Teleop", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();

    private double servoPosition = 0;

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
                robot.scoopMotor.setPower(.3);
//                robot.scoopMotor.moveToEncoderVal(-10, .3);
            }
            else if(gamepad1.right_trigger > .25) {
                robot.scoopMotor.setPower(-.3);
//                robot.scoopMotor.moveToEncoderVal(-458, .6);
            }
            else {
                robot.scoopMotor.setPower(0);
            }

            telemetry.addData("Scoop position", robot.scoopMotor.getEncoderVal());

            //Intake controls
            //Move intake vertically
            if(gamepad1.dpad_right) {
                robot.intake.raiseIntake();
            }
            else if(gamepad1.dpad_left) {
                robot.intake.lowerIntake();
            }
            else if(gamepad1.x) {
                robot.intake.moveIntakePerp();
            }

            //Code for determining good lifter positions
            if(gamepad2.y) {
                robot.intake.openGate();
            }
            else if(gamepad2.x) {
                robot.intake.closeGate();
            }


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
            else if(gamepad1.left_stick_button || gamepad2.a) {
                robot.intake.movePulley(robot.intake.scoringThreshold + 30);
            }
            else if(gamepad1.right_stick_button || gamepad2.b) {
                robot.intake.movePulley(robot.intake.clearPosition);
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

            if(robot.intake.getPulleyPosition() < robot.intake.scoringThreshold && !(robot.intake.getPulleyPosition() < -100)) {
                robot.intake.closeGate();
            }
            else {
                robot.intake.openGate();
            }



            //Test opmode controls

            telemetry.addData("heading", robot.gyro.imu.getAngularOrientation().firstAngle);
            telemetry.addData("pulley position", robot.intake.getPulleyPosition());
            telemetry.addData("lift position", robot.lift.getEncoderVal());
            telemetry.addData("gatePosition", robot.intake.gate.getPosition());
            telemetry.update();

            sleep(50);
        }

    }

    public void scoreMinerals() {
        robot.intake.movePulley(robot.intake.scoringThreshold);
        waitForMovement(robot.intake, 2);
        robot.intake.movePulley(robot.intake.clearPosition);
    }

    public void waitForMovement(IsBusy object, double seconds) {
        boolean flag = false;
        for(int i = 0; i < seconds*1000 && !flag; i+=10) {
            sleep(10);
            if(!object.isBusy())
                flag = true;
            else if(isStopRequested()) {
                break;
            }
        }
        sleep(100);
    }
}
