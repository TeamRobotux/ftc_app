/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * A test to check the capabilites of encoders
 */

@Autonomous(name="AutoBlueFar", group="Pushbot")
public class AutonomousBlueFar extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.grabber.close();
        waitForStart();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("heading:", robot.gyro.imu.getAngularOrientation().firstAngle);
        telemetry.update();

        while(opModeIsActive()) {

            // Wait for the game to start (driver presses PLAY)

            double distanceAdd = scanColumn(telemetry);

            //turnDegrees(45);
            sleep(200);

            robot.wheels.driveDistance(24);
            waitForMovement(7);


            robot.wheels.strafeDistance(36 + distanceAdd);
            waitForMovement(5);

            robot.wheels.driveDistance(12);
            waitForMovement(2);

            robot.grabber.open();
            sleep(500);

            robot.wheels.driveDistance(-10);
            waitForMovement(2);

            robot.grabber.close();
            sleep(400);

            robot.wheels.driveDistance(12);
            waitForMovement(2);

            robot.wheels.driveDistance(-14);
            waitForMovement(2);
            stop();

        }
    }

    private void waitForMovement(int seconds) {
        for(int i = 0; i < seconds*100 && robot.wheels.isBusy(); i++) { sleep(10); }

        sleep(50);
    }

    public void turnDegrees(int degrees) {
        float originalAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
        float deltaAngle = 0;

        while (deltaAngle > degrees+1 || deltaAngle < degrees-1) {
            deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

            if (deltaAngle < degrees-1) {
                robot.wheels.turn(.2);
            } else {
                robot.wheels.turn(-.2);
            }

            if(isStopRequested()) {
                break;
            }

            telemetry.addData("heading: ", deltaAngle);
            telemetry.update();
        }

        robot.wheels.turn(0);
        sleep(100);

        deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

        while(deltaAngle > degrees+1 || deltaAngle < degrees-1) {
            deltaAngle = originalAngle - robot.gyro.imu.getAngularOrientation().firstAngle;

            if (deltaAngle < degrees-1) {
                robot.wheels.turn(.2);
            } else {
                robot.wheels.turn(-.2);
            }

            if(isStopRequested()) {
                break;
            }
        }

        robot.wheels.turn(0);
    }

    private double scanColumn(Telemetry t) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaiquuH/////AAAAGa0Yq9q1+0YrjKIQl75JKMtbkCfbX1s4QuajYfob6seMwTDejdEf8WOHpi4ynOSLXdKC2tPaPTZqNCXDPbFNik7OS3eUUJGNWoCXlvax5In3QvY7HtWsnGG2KIa/AkJYeu69kYsmIEd7y9fEr1BSX5MXkkghfKAfV644TDRxntIB/YCyWaAcsmOvPuK14RxTh8PTjcX9vYPCpVh8Sq/OlERLvXkDasPo+0jFxMkPYrEauQ3bawhYt6xFuCa861gAiDgIEo3kAvcvrwYOGwJqueueKTthyG6Ydvfk5qvAs/hRbVOuAOwhCKs87TdHrx08xiUaGKxm251/WlVkPPrDUdesFJVcfXE0JXXrEJBCeOL5";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        for(int i = 0; i < 20 && (vuMark == RelicRecoveryVuMark.UNKNOWN ||  vuMark == null); i++) {
            sleep(100);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(isStopRequested()) {
                stop();
            }
        }

        double distanceAdd = 0;
        if(vuMark == RelicRecoveryVuMark.LEFT) {
            distanceAdd -= 13.125;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT) {
            distanceAdd += 13.125;
        }

        t.addData("VuMark", vuMark.toString());

        return distanceAdd;



    }
}


