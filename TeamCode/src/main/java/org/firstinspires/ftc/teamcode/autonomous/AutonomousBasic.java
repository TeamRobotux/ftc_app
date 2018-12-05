package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.robot.IsBusy;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Created by jack on 10/15/18.
 */

public abstract class AutonomousBasic extends LinearOpMode {
    protected ElapsedTime runtime = new ElapsedTime();
    protected RobotHardware robot = new RobotHardware();

    protected Bitmap frame = null;
    protected MineralDetectorPipeline mDetector;
    protected FtcDashboard dashboard;
    // protected VuforiaNavigator vNavigator = new VuforiaNavigator(robot, hardwareMap);1

    abstract void runAutonomous();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        mDetector = new MineralDetectorPipeline();

        mDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        mDetector.enable();

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
                robot.drivetrain.turn(-power);
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
            adjustedDegrees = 179;
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


    public double disengageAndSample(RobotHardware robot, LinearOpMode opMode) {
//        robot.lift.raiseLift();
//        waitForMovement(robot.lift, this, 10);
//
//        robot.drivetrain.driveDistance(6);
//        sleep(100);

        //TODO change
//        robot.lift.setPower(-1);
//        waitForMovement(robot.drivetrain, this, 5);
//        robot.lift.setPower(0);

        robot.drivetrain.strafeDistance(-14, .8);
        waitForMovement(robot, this, 3);

        //TODO optimize to make quicker?

        turnDegrees(robot, this, -100, .4);
        waitForMovement(robot, this, 8);

        robot.drivetrain.strafeDistance(0, .4);
        waitForMovement(robot, opMode, 5);

        if(isSampleGold(opMode, mDetector)) {
            robot.drivetrain.driveDistance(18, 1);
            waitForMovement(robot, this, 2);
            robot.drivetrain.driveDistance(-18, 1);
            waitForMovement(robot, this, 2);

            return -36;
        }
        else {
            robot.drivetrain.strafeDistance(-20, .75);
            waitForMovement(robot, opMode, 5);

            if(isSampleGold(opMode, mDetector)) {
                robot.drivetrain.driveDistance(18, 1);
                waitForMovement(robot, this, 2);
                robot.drivetrain.driveDistance(-18, 1);
                waitForMovement(robot, this, 2);

                return -20;
            }
            else {
                robot.drivetrain.strafeDistance(-20, .75);
                waitForMovement(robot, opMode, 5);

                robot.drivetrain.driveDistance(18,1);
                waitForMovement(robot, this, 2);
                robot.drivetrain.driveDistance(-18,1);
                waitForMovement(robot, this, 2);

                return 0;
            }
        }
    }

    public static boolean isMineralCenteredX(RobotHardware robot, Mineral m) {
        return m.center.x < robot.cameraCenter.x- robot.cameraOffset + 20 || m.center.x < robot.cameraCenter.x- robot.cameraOffset - 20;
    }

    public static boolean isSampleGold(LinearOpMode opMode, MineralDetectorPipeline mineralDetector) {
        opMode.sleep(1000);

        return mineralDetector.getFirstMineral().type == Mineral.Type.GOLD;
    }


}
