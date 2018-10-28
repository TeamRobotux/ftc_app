package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.support.annotation.NonNull;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;

import java.io.File;

/**
 * Created by jack on 10/15/18.
 */

@Autonomous(name="AutoBlueCrater", group="Linear Opmode")
public class AutoBlueCrater extends AutonomousBasic implements CameraCaptureSession.StateCallback, CameraCaptureSession.StatusCallback, CameraCaptureSession.CaptureCallback, Consumer<CameraFrame>{
    Bitmap frame = null;

    CameraCaptureSession captureSession = null;
    @Override
    void runAutonomous() {

        File[] fileArray = new File[] {};

        TuxWebcam webcam = new TuxWebcam(new int[] { com.qualcomm.robotcore.R.xml.teamwebcamcalibrations }, fileArray, 0, Double.MAX_VALUE, 30, robot.webcamName);

        if(webcam.openWebcam()) {
            Camera c = webcam.getCamera();

            try {
                c.createCaptureSession(Continuation.createTrivial(this));
            } catch (CameraException e) {
                for(StackTraceElement s : e.getStackTrace()) {
                    Log.e("AutoBlueCrater", s.toString());
                }
            }
        }


//        sleep(5000);
//
//        webcam.getFrameOnce(Continuation.createTrivial(this));
//
//        sleep(2000);



        /*
        disengageLift(robot, this);
        robot.drivetrain.strafeDistance(5);
        waitForMovement(robot.drivetrain, this, 2);
        turnDegrees(robot,this, 90, .5);
        */

        //MineralDetector.goldPosition gPosition = mDetector.getPositions(vNavigator.getFrame());

    }

/*
    @Override
    public void accept(CameraFrame value) {
        if(frame == null) {
            frame = Bitmap.createBitmap(value.getSize().getWidth(), value.getSize().getHeight(), Bitmap.Config.ARGB_8888);
        }
        value.copyToBitmap(frame);
    }
*/

    @Override
    public void onConfigured(@NonNull org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession session) {
        captureSession = session;
        try {
            session.startCapture(session.getCamera().createCaptureRequest(ImageFormat.FLEX_RGBA_8888, new Size(640,480), 30), this, Continuation.createTrivial(this));
        }
        catch(Exception e) {
            Log.e("AutoBlueCrater", e.toString());
        }
    }

    @Override
    public void onClosed(@NonNull org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession session) {

    }

    @Override
    public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {

    }

    @Override
    public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
        if(frame == null) {
            frame = Bitmap.createBitmap(request.getSize().getWidth(),request.getSize().getHeight(), Bitmap.Config.ARGB_8888);
        }
        cameraFrame.copyToBitmap(frame);

        Log.i("AutoBlueCrater", frame.toString());
    }

    @Override
    public void accept(CameraFrame value) {
        if(frame == null) {
            frame = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
        }
        value.copyToBitmap(frame);

    }
}
