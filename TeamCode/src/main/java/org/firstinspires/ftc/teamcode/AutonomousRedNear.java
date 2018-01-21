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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
            double columnAdd = AutoUtil.scanColumn(robot, this);
            sleep(1000);
            double jewelCompensation = AutoUtil.knockJewels(robot, this, false);

            robot.wheels.driveDistance(48  - jewelCompensation + columnAdd);  //38.1 rn
            AutoUtil.waitForMovement(robot, this, 5);

            AutoUtil.Column column = null;
            switch("" + columnAdd) {
                case "6": column = AutoUtil.Column.RIGHT;
                case "-6": column = AutoUtil.Column.LEFT;
                default: column = AutoUtil.Column.CENTER;
            }

            double[] PID = {-.2, -.2};
            AutoUtil.findColumn(robot, this, -1, column, PID);

            AutoUtil.turnDegrees(robot, this, 90);

            telemetry.addLine("Finished Turn");
            sleep(3000);

            robot.wheels.driveDistance(15);
            AutoUtil.waitForMovement(robot, this, 2);

            robot.grabber.open();
            sleep(400);

            robot.wheels.driveDistance(-12);
            AutoUtil.waitForMovement(robot, this, 2);

            robot.grabber.close();

            robot.wheels.driveDistance(14);
            AutoUtil.waitForMovement(robot, this, 2);

            robot.wheels.driveDistance(-8);
            AutoUtil.waitForMovement(robot, this, 2);

            stop();

        }
    }
}


