package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Pipelines.RingPipeline;
import org.firstinspires.ftc.teamcode.Utills.PID;
import org.openftc.easyopencv.OpenCvPipeline;

public class DriveTrain {
    private DcMotorEx lf, rf, lb, rb;
    private BNO055IMU imu;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private PID xVisionController = new PID(Constant.K_dt_xKpVision, Constant.K_dt_xKiVision, Constant.K_dt_xKdVision);
    private PID yVisionController = new PID(Constant.K_dt_yKpVision, Constant.K_dt_yKiVision, Constant.K_dt_yKdVision);
    private PID rotateController = new PID(Constant.K_dt_rotationKp, Constant.K_dt_rotationKi, Constant.K_dt_rotationKd);

    private double lastTime = 0;
    private double lastAngle = 0;

    public DriveTrain(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init(boolean auto){
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(auto){
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setPower(0, 0, 0, 0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        telemetry.addData("drive", "ready to go");
    }

    private void setPower(double lfp, double lbp, double rfp, double rbp){
        lf.setPower(lfp);
        lb.setPower(lbp);
        rf.setPower(rfp);
        rb.setPower(rbp);

        telemetry.addData("lfp, rfp, lbp, rbp", "%.1 %.1 %.1 %.1", lfp, lbp, rfp, rbp);
    }

    private void calculatePower(double x, double y, double r){
        double lfp = y + x + r;
        double lbp = y - x + r;
        double rfp = y - x - r;
        double rbp = y + x - r;

        setPower(lfp, lbp, rfp, rbp);
    }

    private void fieldCentric(double x, double y, double r, ElapsedTime time){
        double stick180 = Math.toDegrees(Math.atan2(y, x));
        double stick360 = (stick180 + 360) % 360;

        double robot180 = anglePrediction(time) + Constant.K_dt_angleOffset;
        double robot360 = (robot180 + 360) % 360;

        double finalAngle180 = stick360 + robot360;
        double finalAngle360 = (finalAngle180 + 360) % 360;

        double vectorLength = Math.hypot(x, y);
        x = Math.sin(Math.toRadians(finalAngle360)) * vectorLength;
        y = -Math.cos(Math.toRadians(finalAngle360)) * vectorLength;

        calculatePower(x, y, r);
    }

    public void driver(double x, double y, double r, ElapsedTime time){
        fieldCentric(x, y, r, time);
    }

    public void vision(RingPipeline pipeline, ElapsedTime time){
        if (pipeline.x < Constant.K_dt_ringXmin || Constant.K_dt_ringXmax > pipeline.x &&
            pipeline.y < Constant.K_dt_ringYmin || Constant.K_dt_ringYmax > pipeline.y){
            double x = xVisionController.calculate(Constant.K_dt_ringXcenter, pipeline.x, time.milliseconds());
            double y = yVisionController.calculate(Constant.K_dt_ringYcenter, pipeline.y, time.milliseconds());
            calculatePower(x, y, 0);
        }
    }

    private double getAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double anglePrediction(ElapsedTime time){
        double dt = time.milliseconds() - lastTime;
        double dx = getAngle() - lastAngle;
        double v = dx / dt;
        dx = v * dt;
        return getAngle() + (dx*dt); //TODO maybe its only dx without multiply at dt
    }
}
