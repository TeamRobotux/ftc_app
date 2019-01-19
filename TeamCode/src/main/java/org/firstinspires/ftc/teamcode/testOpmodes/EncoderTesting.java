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

            //TelemetryPacket telemetry = new TelemetryPacket();

            double currentPos = robot.markerServo.getPosition();
            if(gamepad1.a && currentPos < 1) {
                robot.markerServo.moveTo(currentPos + .01);
            }
            else if(gamepad1.b && currentPos > 0) {
                robot.markerServo.moveTo(currentPos - .01);
            }

            telemetry.addData("servoPos", robot.markerServo.getPosition());

            //dashboard.sendTelemetryPacket(telemetry);

            //dashboard.updateConfig();

            telemetry.update();


            sleep(50);
        }
    }
}
