package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.support.annotation.NonNull;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.opencv.features2d.BOWTrainer;

import java.util.logging.Logger;

/**
 * Created by jack on 10/15/18.
 */

public abstract class AutonomousBasic extends LinearOpMode {
    protected ElapsedTime runtime = new ElapsedTime();
    protected RobotHardware robot = new RobotHardware();

    protected Bitmap frame = null;
    //protected MineralDetector mDetector = new MineralDetector();
    // protected VuforiaNavigator vNavigator = new VuforiaNavigator(robot, hardwareMap);1

    abstract void runAutonomous();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runAutonomous();
            sleep(25000);
            stop();
        }
    }

    public static void turnDegrees(RobotHardware robot, LinearOpMode opMode, int degrees, double power) {
        float originalAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
        float deltaAngle = 0;
        int adjustedDegrees = degrees;

        if(degrees == 180) {
            adjustedDegrees = 160;
        }

        while (deltaAngle > adjustedDegrees+1 || deltaAngle < adjustedDegrees-1) {
            deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

            if (deltaAngle < adjustedDegrees-1) {
                robot.drivetrain.turn(power);
            } else {
                robot.drivetrain.turn(power);
            }

            if(opMode.isStopRequested()) {
                break;
            }

            opMode.telemetry.addData("heading: ", deltaAngle);
            opMode.telemetry.update();
        }

        robot.drivetrain.turn(0);
        opMode.sleep(100);

        deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;
        if(degrees == 180) {
            adjustedDegrees = 173;
        }

        while(deltaAngle > adjustedDegrees+1 || deltaAngle < adjustedDegrees-1) {
            deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

            if (deltaAngle < adjustedDegrees-1) {
                robot.drivetrain.turn(.25);
            } else {
                robot.drivetrain.turn(-.25);
            }

            if(opMode.isStopRequested()) {
                break;
            }
        }

        robot.drivetrain.turn(0);
    }

    public static void waitForMovement(IsBusy object, LinearOpMode opMode, double seconds) {
        boolean flag = false;
        for(int i = 0; i < seconds*1000 && !flag; i+=10) {
            opMode.sleep(10);
            if(!object.isBusy())
                flag = true;
            else if(opMode.isStopRequested()) {
                break;
            }
        }
        opMode.sleep(100);
    }

    public static void engageLift(RobotHardware robot, LinearOpMode opMode) {
        robot.lift.raiseLift();
        waitForMovement(robot.lift, opMode, 5);
        //TODO find actual distance
        robot.drivetrain.driveDistance(-1);
        waitForMovement(robot.drivetrain, opMode, 2);
    }

    public static void disengageLift(RobotHardware robot, LinearOpMode opMode) {
        robot.lift.lowerLift();
        waitForMovement(robot.lift, opMode, 5);
        //TODO find actual distance
        robot.drivetrain.driveDistance(1);
        waitForMovement(robot.drivetrain, opMode, 2);
    }

    public static void grabGoldMineral(RobotHardware robot, LinearOpMode opMode, Mineral gold) {
        double sizeRatio = gold.radius/Mineral.goldSize;

        if(!isMineralCenteredX(robot, gold)) {
            robot.drivetrain.strafeDistance((gold.center.x-robot.cameraCenter.x-robot.cameraOffset)*Mineral.dpi);
            waitForMovement(robot, opMode, 10);
        }


        robot.drivetrain.driveDistance(7);

        //TODO make robot move forward; maybe add microswitches to detect when mineral is intaked?
    }

    public static boolean isMineralCenteredX(RobotHardware robot, Mineral m) {
        return m.center.x < robot.cameraCenter.x- robot.cameraOffset + 20 || m.center.x < robot.cameraCenter.x- robot.cameraOffset - 20;
    }

}
