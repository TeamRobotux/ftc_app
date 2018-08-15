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

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Main Teleop", group="Pushbot")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double driveIn;
        double turnIn;
        double strafeIn;
        double driveOut = 0;
        double turnOut = 0;
        double strafeOut = 0;

        boolean slow;

        int armDelay = 0;

        final double DRIVE_SCALE_FACTOR = .5;
        final double TURN_SCALE_FACTOR = .7;
        final double STRAFE_SCALE_FACTOR = .5;
        final double MINIMUM_DIFFERENCE = .2;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.jewelServo.moveTo(.5);
        robot.jewelR.setPosition(1);

        robot.gyro.composeTelemetry(telemetry);

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            armDelay--;

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            driveIn = -gamepad1.left_stick_y;
            turnIn  =  gamepad1.right_stick_x;
            strafeIn = gamepad1.left_stick_x;

            //Drivetrain controls
            double difference = driveIn - driveOut;
            if(driveIn == 0) {
                driveOut = 0;
            }
            else if(difference < MINIMUM_DIFFERENCE) {
                driveOut = driveIn;
            }
            else {
                driveOut += difference*DRIVE_SCALE_FACTOR;
            }

            difference = strafeIn - strafeOut;
            if(strafeIn == 0) {
                strafeOut = 0;
            }
            else if(difference < MINIMUM_DIFFERENCE) {
                strafeOut = strafeIn;
            }
            else {
                strafeOut += difference*STRAFE_SCALE_FACTOR;
            }

            difference = turnIn - turnOut;
            if(turnIn == 0) {
                turnOut = 0;
            }
            else if(difference < MINIMUM_DIFFERENCE) {
                turnOut = turnIn;
            }
            else {
                turnOut += difference*TURN_SCALE_FACTOR;
            }

            if(gamepad1.right_trigger > .5) {
                robot.wheels.drive360(strafeOut/2, driveOut/2, turnOut/2);
            }
            else {
                robot.wheels.drive360(strafeOut, driveOut, turnOut);
            }

            //Pulley Movement
            if(gamepad1.dpad_up) {
                robot.pulley.setPower(-1);
                if(gamepad1.right_trigger > .5) {
                    robot.pulley.setPower(1);
                }
            }
            else if(gamepad1.dpad_down) {
                robot.pulley.setPower(.25);
                if(gamepad1.right_trigger > .5) {
                    robot.pulley.setPower(-1);
                }
            }
            else {
                robot.pulley.setPower(0);
            }

            //Glyph Grabber movement
            //Rotation
            if(gamepad1.dpad_right || gamepad1.right_bumper) {
                robot.grabber.rotClockwise();
            }
            else if(gamepad1.dpad_left || gamepad1.left_bumper) {
                robot.grabber.rotCounterClockwise();
            }
            else {
                robot.grabber.stopRot();
            }

            //Glyph Grabber intake control
            if(gamepad1.left_trigger > .5 && gamepad1.a) {
                robot.grabber.suckAll();
            }
            else if(gamepad1.b) {
                robot.grabber.push();
            }
            else if(gamepad1.a) {
                robot.grabber.suck();
            }
            else if(gamepad1.y) {
                robot.grabber.stop();
            }
            else if(gamepad1.x) {
                robot.grabber.pushAll();
            }


            //Jewel arm control
            if(((gamepad1.left_trigger > .5 && gamepad1.right_trigger > .5) || gamepad2.x) && Math.round(robot.jewelR.getPosition()) == 1 && armDelay < 5) {
                robot.jewelR.setPosition(0);
                armDelay = 5;
            } else if(((gamepad1.left_trigger > .5 && gamepad1.right_trigger > .5) || gamepad2.x) && armDelay < 5) {
                robot.jewelR.setPosition(1);
                armDelay = 5;
            }

            /*BEGINNING OF SECOND DRIVER CONTROLS


            Relic arm controls

            Relic arm pulley control */
            if(gamepad2.left_bumper) {
                slow = true;
            }
            else {
                slow = false;
            }
            if(gamepad2.dpad_down) {
                robot.relicArm.retract(slow);
            }
            if(gamepad2.dpad_up) {
                robot.relicArm.extend(slow);
            }
            else {
                robot.relicArm.stopPulley();
            }

            //relic arm Rotational controls
            if(gamepad2.dpad_left) {
                robot.relicArm.rotateCounterClockwise(false);
            }
            else if(gamepad2.dpad_right) {
                robot.relicArm.rotateClockwise(slow);
            }
            else {
                robot.relicArm.stopSusan();
            }

            //Relic arm wrist controls
            if(gamepad2.y) {
                robot.relicArm.raiseWrist();
            }
            else if(gamepad2.a) {
                robot.relicArm.lowerWrist();
            }
            else if(gamepad2.b) {
                robot.relicArm.moveWristParallel();
            }

            //Relic arm hand controls
            if(gamepad2.right_trigger > .5) {
                robot.relicArm.openHand();
            }
            else if(gamepad2.left_trigger > .5) {
                robot.relicArm.closeHand();
            }


            telemetry.addData("Grabber pos: ", robot.grabber.toString());
            telemetry.addData("grabberR pos: ", robot.jewelR.getPosition());
            telemetry.addData("tolerance:", robot.pulley.getTolerance());
            telemetry.addData("red data:", robot.jewelColorDistanceSensor.getRed());
            telemetry.update();
            sleep(50);
        }
    }
}