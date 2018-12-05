package org.firstinspires.ftc.teamcode.robot;

/**
 * Created by jack on 12/4/18.
 */

public class PIDController {
    private TuxMotor motor;

    private double kp;
    private double ki;
    private double kd;
    private int tolerance;

    private int goal = 0;
    private int error = 0;
    private int lastError = 0;

    private double integral = 0;

    private long lastTime = System.currentTimeMillis();

    public PIDController(TuxMotor m, double p, double i, double d, int t) {
        motor = m;

        kp = p;
        ki = i;
        kd = d;
        tolerance = t;
    }

    public void setGoal(int newGoal) {
        goal = newGoal;

        error = goal - motor.getEncoderVal();
        lastError = error;
        integral = 0;

        lastTime = System.currentTimeMillis();
    }

    public double getOutput() {
        long dTime = (System.currentTimeMillis() - lastTime)/1000;
        error = goal - motor.getEncoderVal();

        double derivative = (error - lastError)/dTime;
        integral += error*dTime;

        double output = kp*error + ki*integral + kd*derivative;

        return output;
    }

    public int getTolerance() {
        return tolerance;
    }

    public double getError() {
        return error;
    }

}
