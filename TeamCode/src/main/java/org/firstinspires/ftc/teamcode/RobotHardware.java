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

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;




public class RobotHardware
{
    /* Public OpMode members. */

    //Pulley ticks from top to bottom 3880, length = 29.75, ticks per block = 784
    public TuxMotor pulley;
    public Drivetrain wheels;


    public GlyphGrabber grabber;

    public Servo jewelL;
    public Servo jewelR;

    public TuxGyro gyro;

    public LynxI2cColorRangeSensor colorSensor;

    /* local OpMode members. */

    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Define and Initialize Motors
        //Pulley motor lift length = 35.2
        pulley = new TuxMotor("pulley", hwMap, 130, 1);

        wheels = new Drivetrain(hwMap);

        grabber = new GlyphGrabber(hwMap);

        //gyro = hwMap.get(GyroSensor.class, "imu 1");
       // gyro.calibrate();

        jewelL  = hwMap.get(Servo.class, "jewelL");
        jewelR  = hwMap.get(Servo.class, "jewelR");

        gyro = new TuxGyro(hwMap);

        //colorSensor = (LynxI2cColorRangeSensor) hwMap.get("colorSensor");

        jewelL.setDirection(Servo.Direction.FORWARD);
        jewelR.setDirection(Servo.Direction.REVERSE);

//        jewelL.setPosition(0);
      //  jewelL.setPosition(0);
    }

}

