package org.firstinspires.ftc.teamcode.testOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Created by jack on 9/26/18.
 */
@Disabled
@TeleOp(name="Testing", group="Linear Opmode")
public class IntakeTestingTeleop extends LinearOpMode {

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
                if(gamepad1.a) {
                    robot.lift.setPower(-1);
                }
                else if(gamepad1.b) {
                    robot.lift.setPower(1);
                }
                else {
                    robot.lift.setPower(0);
                }
            }
        }

}
