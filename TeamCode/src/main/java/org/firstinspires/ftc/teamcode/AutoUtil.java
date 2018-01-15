package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Jack Lowry on 1/7/2018.
 */

public final class AutoUtil {

    //DONT INSTANTIATE
    private AutoUtil() {
        super();
    }

    public static void waitForMovement(RobotHardware robot, LinearOpMode opMode, int seconds) {
        for(int i = 0; i < seconds*100 && robot.wheels.isBusy(); i++) { opMode.sleep(10); }

        opMode.sleep(50);
    }

    public static void turnDegrees(RobotHardware robot, LinearOpMode opMode, int degrees) {
        float originalAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
        float deltaAngle = 0;

        while (deltaAngle > degrees+1 || deltaAngle < degrees-1) {
            deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

            if (deltaAngle < degrees-1) {
                robot.wheels.turn(.2);
            } else {
                robot.wheels.turn(-.2);
            }

            if(opMode.isStopRequested()) {
                break;
            }

            opMode.telemetry.addData("heading: ", deltaAngle);
            opMode.telemetry.update();
        }

        robot.wheels.turn(0);
        opMode.sleep(100);

        deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

        while(deltaAngle > degrees+1 || deltaAngle < degrees-1) {
            deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

            if (deltaAngle < degrees-1) {
                robot.wheels.turn(.2);
            } else {
                robot.wheels.turn(-.2);
            }

            if(opMode.isStopRequested()) {
                break;
            }
        }

        robot.wheels.turn(0);
    }

    public static double scanColumn(RobotHardware robot, LinearOpMode opMode) {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AaiquuH/////AAAAGa0Yq9q1+0YrjKIQl75JKMtbkCfbX1s4QuajYfob6seMwTDejdEf8WOHpi4ynOSLXdKC2tPaPTZqNCXDPbFNik7OS3eUUJGNWoCXlvax5In3QvY7HtWsnGG2KIa/AkJYeu69kYsmIEd7y9fEr1BSX5MXkkghfKAfV644TDRxntIB/YCyWaAcsmOvPuK14RxTh8PTjcX9vYPCpVh8Sq/OlERLvXkDasPo+0jFxMkPYrEauQ3bawhYt6xFuCa861gAiDgIEo3kAvcvrwYOGwJqueueKTthyG6Ydvfk5qvAs/hRbVOuAOwhCKs87TdHrx08xiUaGKxm251/WlVkPPrDUdesFJVcfXE0JXXrEJBCeOL5";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        ClosableVuforiaLocalizer vuforia = new ClosableVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        for(int i = 0; i < 20 && (vuMark == RelicRecoveryVuMark.UNKNOWN ||  vuMark == null); i++) {
            opMode.sleep(100);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(opMode.isStopRequested()) {
                opMode.stop();
            }
        }

        double distanceAdd = 0;
        if(vuMark == RelicRecoveryVuMark.LEFT) {
            distanceAdd += 9;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT) {
            distanceAdd -= 9;
        }

        opMode.telemetry.addData("VuMark", vuMark.toString());
        vuforia.close();
        return distanceAdd;



    }

    public static double knockJewels(RobotHardware robot, LinearOpMode opMode, boolean blue) {
        JewelDetector detector = new JewelDetector();
        detector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        detector.enable();
        robot.jewelR.setPosition(0);
        opMode.sleep(5000);

        for(int i = 0; i < 20 && (detector.getCurrentOrder() == JewelDetector.JewelOrder.UNKNOWN || detector.getCurrentOrder() == null); i++) {
            opMode.sleep(100);
            if(opMode.isStopRequested()) {
                opMode.stop();
            }
        }

        int jewelCompensation = 0;
        if(detector.getCurrentOrder() == JewelDetector.JewelOrder.BLUE_RED) {
            if(blue) {
                robot.wheels.driveDistance(-6);
                jewelCompensation = -6;
            }
            else {
                robot.wheels.driveDistance(6);
                jewelCompensation = 6;
            }
        }
        else if(detector.getCurrentOrder() == JewelDetector.JewelOrder.RED_BLUE) {
            if(blue) {
                robot.wheels.driveDistance(6);
                waitForMovement(robot, opMode, 1);
                robot.jewelR.setPosition(1);
                robot.wheels.driveDistance(-16);
                jewelCompensation = -6;
            }
            else {
                robot.wheels.driveDistance(-6);
                waitForMovement(robot, opMode, 1);
                robot.jewelR.setPosition(1);
                robot.wheels.driveDistance(16);
                jewelCompensation = 6;
            }
        }
        waitForMovement(robot, opMode, 2);
        robot.jewelR.setPosition(1);
        opMode.sleep(500);
        detector.disable();
        return jewelCompensation;
    }

    public static void findColumn(RobotHardware robot, LinearOpMode opMode, int column) {
        CryptoboxDetector detector = new CryptoboxDetector();
        detector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);

    }

}
