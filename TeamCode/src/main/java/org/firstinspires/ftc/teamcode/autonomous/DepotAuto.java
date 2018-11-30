package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */

@Autonomous(name="Depot Auto", group = "linear opmode")
public class DepotAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {
        disengageAndSample(robot, this);


    }
}
