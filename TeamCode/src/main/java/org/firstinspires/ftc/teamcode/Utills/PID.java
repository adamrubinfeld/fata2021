package org.firstinspires.ftc.teamcode.Utills;

public class PID {
    private double Kp;
    private double Ki;
    private double Kd;

    private double lastTime = 0;
    private double lastError = 0;
    private double integral = 0;

    public PID (double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double calculate(double setPoint, double currentPoint, double time){
        double error = setPoint - currentPoint;
        integral += error;
        double derivative = (error - lastError)/(time - lastTime);

        lastTime = time;
        lastError = error;
        return  error * Kp + integral * Ki + derivative * Kd;
    }

    public void reset(){
        lastError = 0;
        lastTime = 0;
        integral = 0;
    }
}
