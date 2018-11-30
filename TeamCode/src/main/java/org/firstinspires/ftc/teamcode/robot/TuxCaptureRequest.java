package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;

/**
 * Created by jack on 10/26/18.
 */

public class TuxCaptureRequest implements CameraCaptureRequest {
    private int format = ImageFormat.FLEX_RGBA_8888;
    Size size = new Size(640,480);
    long frameDuration = 100;
    int fps = 30;

    @Override
    public int getAndroidFormat() {
        return 0;
    }

    @Override
    public Size getSize() {
        return null;
    }

    @Override
    public long getNsFrameDuration() {
        return 0;
    }

    @Override
    public int getFramesPerSecond() {
        return 0;
    }

    @NonNull
    @Override
    public Bitmap createEmptyBitmap() {
        return Bitmap.createBitmap(size.getWidth(), size.getHeight(), Bitmap.Config.ARGB_8888);
    }
}
