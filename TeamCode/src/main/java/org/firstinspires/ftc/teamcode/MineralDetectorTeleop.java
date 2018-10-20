package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="Example: MineralDetectorTeleop")
public class MineralDetectorTeleop extends OpMode {
    private MineralDetector mDetector;
    private VuforiaNavigator vNavigator;

    private boolean openCVLoaded = false;

    private RobotHardware robot = new RobotHardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        // start the vision system
        vNavigator = new VuforiaNavigator(robot, hardwareMap);
        /*  BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch(status) {
                    case LoaderCallbackInterface.SUCCESS:
                        mDetector = new MineralDetector();
                        openCVLoaded = true;
                        break;
                    default:
                        super.onManagerConnected(status);
                        break;
                }

            }
        };

        while(!openCVLoaded) {
            try {
                wait(100);
                Log.d("Teleop", "waiting for opencv to load");
            }
            catch(Exception e) {
                Log.e("Teleop", e.toString());
            }
        }*/
    }

    @Override
    public void loop() {
        // get a list of contours from the vision system
        vNavigator.updateFrame();
        //mDetector.processFrame(vNavigator.getFrame());
    }

    public void stop() {
        // stop the vision system
    }
}