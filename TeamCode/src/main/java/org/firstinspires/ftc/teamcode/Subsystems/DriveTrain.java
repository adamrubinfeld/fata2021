package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constant;

public class DriveTrain {
    private DcMotorEx lf, rf, lb, rb;
    private BNO055IMU imu;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

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

    public void teleop(double x, double y, double r){
        fieldCentric(x, y, r);
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

    private void fieldCentric(double x, double y, double r){
        double stick180 = Math.toDegrees(Math.atan2(y, x));
        double stick360 = (stick180 + 360) % 360;

        double robot180 = anglePrediction() + Constant.K_dt_angleOffset;
        double robot360 = (robot180 + 360) % 360;

        double finalAngle180 = stick360 + robot360;
        double finalAngle360 = (finalAngle180 + 360) % 360;

        double vectorLength = Math.hypot(x, y);
        x = Math.sin(Math.toRadians(finalAngle360)) * vectorLength;
        y = -Math.cos(Math.toRadians(finalAngle360)) * vectorLength;

        calculatePower(x, y, r);
    }

    private double getAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    //TODO build that function
    private double anglePrediction(){
        return getAngle();
    }
}
