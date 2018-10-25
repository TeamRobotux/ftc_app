package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;

import java.io.File;

/**
 * Created by jack on 10/15/18.
 */

@Autonomous(name="AutoBlueCrater", group="Linear Opmode")
public class AutoBlueCrater extends AutonomousBasic implements Consumer<CameraFrame> {
    Bitmap frame = null;
    @Override
    void runAutonomous() {

        File[] fileArray = new File[] {};

        VuforiaWebcam webcam = new VuforiaWebcam(new int[] { com.qualcomm.robotcore.R.xml.teamwebcamcalibrations }, fileArray, 0, Double.MAX_VALUE, 30, robot.webcamName);
        webcam.createNativeVuforiaWebcam();

        webcam.getFrameOnce(Continuation.createTrivial(this));

        /*
        disengageLift(robot, this);
        robot.drivetrain.strafeDistance(5);
        waitForMovement(robot.drivetrain, this, 2);
        turnDegrees(robot,this, 90, .5);
        */

        //MineralDetector.goldPosition gPosition = mDetector.getPositions(vNavigator.getFrame());

    }

    @Override
    public void accept(CameraFrame value) {
        if(frame == null) {
            frame = Bitmap.createBitmap(value.getSize().getWidth(), value.getSize().getHeight(), Bitmap.Config.ARGB_8888);
        }
        value.copyToBitmap(frame);
    }
}
