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

        robot.intake.moveGate(robot.intake.openGatePos - .1);

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
        double originalAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
        double deltaAngle = 0;
        double targetAngle = originalAngle + degrees;

        double angleAdd = 0;
        double lastAngle = originalAngle;

        double p = 2;

        if(Math.abs(degrees) > 180) {
            degrees %= 360;
            degrees = degrees - (degrees/Math.abs(degrees))*360;
        }

        if(degrees + originalAngle > 180) {

        }
        if(degrees > 0) {
            while(deltaAngle < degrees-1.5) {

                if(deltaAngle < -1) {
                    angleAdd = 360;
                }

                deltaAngle = robot.gyro.imu.getAngularOrientation().firstAngle - originalAngle + angleAdd;

                robot.drivetrain.turn(-power * p * (degrees-deltaAngle)/degrees);

                if(opMode.isStopRequested()) {
                    break;
                }

                opMode.telemetry.addData("angleAdd", angleAdd);
                opMode.telemetry.addData("originalAngle", originalAngle);
                opMode.telemetry.addData("DeltaAngle", deltaAngle);
                opMode.telemetry.addData("TargetAngle", degrees);
                opMode.telemetry.addData("heading", robot.gyro.imu.getAngularOrientation().firstAngle);
                opMode.telemetry.update();

                lastAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
            }
        }
        else if(degrees < 0) {
            while(deltaAngle > degrees-1.5) {

                if(deltaAngle > 1) {
                    angleAdd = 360;
                }

                deltaAngle = robot.gyro.imu.getAngularOrientation().firstAngle - originalAngle - angleAdd;

                robot.drivetrain.turn(power*p*(degrees-deltaAngle)/degrees);

                if(opMode.isStopRequested()) {
                    break;
                }

                opMode.telemetry.addData("angleAdd", angleAdd);
                opMode.telemetry.addData("originalAngle", originalAngle);
                opMode.telemetry.addData("DeltaAngle", deltaAngle);
                opMode.telemetry.addData("TargetAngle", degrees);
                opMode.telemetry.addData("heading", robot.gyro.imu.getAngularOrientation().firstAngle);
                opMode.telemetry.update();

                lastAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
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
        double retValue = disengageAndSampleDepot(robot, opMode);
        robot.drivetrain.driveDistance(-18, 1);
        waitForMovement(robot.drivetrain, this, .5);

        return retValue;
    }

    public double disengageAndSampleDepot(RobotHardware robot, LinearOpMode opMode) {
        robot.lift.raiseLift();
        waitForMovement(robot.lift, this, 7);

        robot.drivetrain.driveDistance(6);
        waitForMovement(robot.drivetrain, this, 1);

        robot.drivetrain.strafeDistance(-14, .8);
        waitForMovement(robot.drivetrain, this, 1);

        //TODO optimize to make quicker?

        turnDegrees(robot, this, 90, .6);
        waitForMovement(robot.drivetrain, this, 1.5);

        robot.drivetrain.strafeDistance(5, 1);
        waitForMovement(robot.drivetrain, opMode, .75);

        if(isSampleGold(opMode, mDetector)) {
            robot.drivetrain.strafeDistance(6, 1);
            waitForMovement(robot.drivetrain, this, .5);
            robot.drivetrain.driveDistance(18,1);
            waitForMovement(robot.drivetrain, this, .5);

            return -40;
        }
        else {
            robot.drivetrain.strafeDistance(-19, .75);
            waitForMovement(robot.drivetrain, opMode, .75);

            if(isSampleGold(opMode, mDetector)) {
                robot.drivetrain.strafeDistance(6, 1);
                waitForMovement(robot.drivetrain, this, .5);
                robot.drivetrain.driveDistance(18,1);
                waitForMovement(robot.drivetrain, this, .5);

                return -24;
            }
            else {
                robot.drivetrain.strafeDistance(-20, 1);
                waitForMovement(robot.drivetrain, opMode, .75);

                robot.drivetrain.strafeDistance(6, 1);
                waitForMovement(robot.drivetrain, this, .5);
                robot.drivetrain.driveDistance(18,1);
                waitForMovement(robot.drivetrain, this, .5);

                return -4;
            }
        }
    }

    public static boolean isSampleGold(LinearOpMode opMode, MineralDetectorPipeline mineralDetector) {
        opMode.sleep(500);

        return mineralDetector.getLowestMineral().type == Mineral.Type.GOLD;
    }


}
