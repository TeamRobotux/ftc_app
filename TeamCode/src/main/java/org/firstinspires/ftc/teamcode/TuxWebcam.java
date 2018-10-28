package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraCallback;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraMode;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.FrameFormat;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;

import java.io.File;

/**
 * Created by jack on 10/26/18.
 */

public class TuxWebcam extends VuforiaWebcam {

    public TuxWebcam(int[] webcamCalibrationResources, File[] webcamCalibrationFiles, double minRatio, double maxRatio, int secondsPermissionTimeout, @NonNull CameraName cameraName) {
        super(webcamCalibrationResources, webcamCalibrationFiles, minRatio, maxRatio, secondsPermissionTimeout, cameraName);
    }

    public boolean openWebcam() {
        boolean retBool = false;

        try {
            retBool = super.openCamera();
        }
        catch(Exception e) {
            Log.e("TuxWebcam", e.toString());
        }

        return retBool;
    }

}
