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
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * A test to check the capabilites of encoders
 */

@Autonomous(name="AutoRedNear", group="Pushbot")
public class AutonomousRedNear extends LinearOpMode {

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

            // Wait for the game to start (driver presses PLAY;


            double columnAdd = scanColumn();



            robot.wheels.strafeDistance(2.5);
            while(robot.wheels.isBusy()) {
                sleep(100);
            }

            robot.jewelR.setPosition(0);
            for(int i = 0; i < 12; i++) {
                sleep(100);
            }

            robot.colorSensor.enableLed(true);

            int distCompensation = 5;
            if(robot.colorSensor.blue() > 12) {
                robot.wheels.driveDistance(5);
                distCompensation *= -1;
            }
            else {
                robot.wheels.driveDistance(-5);

            }

            robot.colorSensor.enableLed(false);


            while(robot.wheels.isBusy()) {
                sleep(100);
            }

            robot.jewelR.setPosition(1);
            for(int i = 0; i < 12; i++) {
                sleep(100);
            }



            robot.wheels.driveDistance(36.95  + distCompensation + columnAdd);  //38.1 rn
            while (robot.wheels.isBusy()) {
                sleep(100);
            }
            while (robot.wheels.isBusy()) {
                sleep(100);
            }
            float xAngle = robot.gyro.imu.getAngularOrientation().firstAngle;

            while (Math.abs(xAngle) > 91 || Math.abs(xAngle) < 89) {
                xAngle = robot.gyro.imu.getAngularOrientation().firstAngle;
                if (xAngle > -90) {
                    robot.wheels.turn(.5);
                } else {
                    robot.wheels.turn(-.25);
                }


                if(isStopRequested()) {
                    break;
                }

                telemetry.addData("heading: ", xAngle);
                telemetry.update();
            }

            robot.wheels.turn(0);

            robot.wheels.driveDistance(5);
            while (robot.wheels.isBusy()) {
                sleep(100);
            }
            robot.grabber.open();
            sleep(500);

            robot.wheels.driveDistance(-8);
            while(robot.wheels.isBusy()); { sleep(100); }

            robot.grabber.close();

            robot.wheels.driveDistance(10);
            while(robot.wheels.isBusy()); { sleep(100); }

            robot.wheels.driveDistance(-8);
            while(robot.wheels.isBusy()); { sleep(100); }





            stop();

        }
    }

    private double scanColumn() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaiquuH/////AAAAGa0Yq9q1+0YrjKIQl75JKMtbkCfbX1s4QuajYfob6seMwTDejdEf8WOHpi4ynOSLXdKC2tPaPTZqNCXDPbFNik7OS3eUUJGNWoCXlvax5In3QvY7HtWsnGG2KIa/AkJYeu69kYsmIEd7y9fEr1BSX5MXkkghfKAfV644TDRxntIB/YCyWaAcsmOvPuK14RxTh8PTjcX9vYPCpVh8Sq/OlERLvXkDasPo+0jFxMkPYrEauQ3bawhYt6xFuCa861gAiDgIEo3kAvcvrwYOGwJqueueKTthyG6Ydvfk5qvAs/hRbVOuAOwhCKs87TdHrx08xiUaGKxm251/WlVkPPrDUdesFJVcfXE0JXXrEJBCeOL5";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        for(int i = 0; i < 20 || vuMark == RelicRecoveryVuMark.UNKNOWN; i++) {
            sleep(100);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(isStopRequested()) {
                stop();
            }
        }

        double distanceAdd = 0;
        if(vuMark == RelicRecoveryVuMark.LEFT) {
            distanceAdd -= 7.875;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT) {
            distanceAdd += 7.875;
        }

        return distanceAdd;



    }
}


