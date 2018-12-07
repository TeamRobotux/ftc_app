package org.firstinspires.ftc.teamcode.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Created by jack on 10/20/18.
 */

@TeleOp(name="Encoder testing", group="Linear Opmode")
public class EncoderTesting extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            TelemetryPacket telemetry = new TelemetryPacket();

            if(gamepad1.right_bumper) {
                robot.lift.raiseLift();
                sleep(100);
            }
            else if(gamepad1.left_bumper) {
                robot.lift.lowerLift();
                sleep(100);
            }

            if(gamepad1.a) {
                robot.drivetrain.strafeDistanceCustom(24, this, dashboard);
                sleep(100);
            }
            else if(gamepad1.b) {
                robot.drivetrain.strafeDistanceCustom(-24, this, dashboard);
                sleep(100);
            }

            double[] dtError = robot.drivetrain.getError();
            for(int i = 0; i < 4; i++) {
                telemetry.put("wheel " + i, dtError[i]);
            }

            dashboard.sendTelemetryPacket(telemetry);

            dashboard.updateConfig();


            sleep(50);
        }
    }
}
