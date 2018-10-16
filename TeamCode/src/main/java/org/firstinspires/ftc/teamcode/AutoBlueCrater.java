package org.firstinspires.ftc.teamcode;

/**
 * Created by jack on 10/15/18.
 */

public class AutoBlueCrater extends AutonomousBasic {
    @Override
    void runAutonomous() {
        vNavigator.updateFrame();
        disengageLift(robot, this);
        robot.drivetrain.strafeDistance(5);
        waitForMovement(robot.drivetrain, this, 2);
        turnDegrees(robot,this, 90, .5);

        MineralDetector.goldPosition gPosition = mDetector.getPositions(vNavigator.getFrame());

    }
}
