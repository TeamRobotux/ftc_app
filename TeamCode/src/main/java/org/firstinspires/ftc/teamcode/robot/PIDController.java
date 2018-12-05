package org.firstinspires.ftc.teamcode.robot;

/**
 * Created by jack on 12/4/18.
 */

public class PIDController {

    private double kp;
    private double ki;
    private double kd;
    private double tolerance;

    private double goal = 0;
    private double error = 0;
    private double lastError = 0;

    private double integral = 0;

    private long lastTime = System.currentTimeMillis();

    public PIDController(double p, double i, double d, double t) {
        kp = p;
        ki = i;
        kd = d;
        tolerance = t;
    }

    public void setGoal(double newGoal, double currentValue) {
        goal = newGoal;

        error = goal - currentValue;
        lastError = error;
        integral = 0;

        lastTime = System.currentTimeMillis();
    }

    public double getOutput(double currentValue) {
        long dTime = (System.currentTimeMillis() - lastTime)/1000;
        error = goal - currentValue;

        double derivative = (error - lastError)/dTime;
        integral += error*dTime;

        double output = kp*error + ki*integral + kd*derivative;

        return output;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getError() {
        return error;
    }

}
