package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Utills.PID;

public class Wobble {
    private DcMotorEx wobble;
    private Servo clipper;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private double lastOutput = 0;
    private int lastTarget = 0;

    private PID wobblePosController = new PID(Constant.K_wobble_Kp, Constant.K_wobble_Ki, Constant.K_wobble_Kd);

    public Wobble(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init(boolean auto){
        wobble = hardwareMap.get(DcMotorEx.class, "wobble");
        clipper = hardwareMap.get(Servo.class, "clipper");

        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(auto){
            wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        wobble.setPower(0);
        clipper.setPosition(Constant.K_wobble_clipperOpen);
    }

    public void SetWobblePos(int ticks, ElapsedTime time){
        if (ticks != lastTarget){
            wobblePosController.reset();
        }else {
            double output = wobblePosController.calculate(ticks, wobble.getCurrentPosition(), time.milliseconds());

            double outputAcc = output - lastOutput;

            if (Math.abs(outputAcc) > Constant.K_wobble_wobbleMaxAcc){
                //TODO fill that part
                telemetry.addData("WOBBLE WARNING", "THE ACCELERATION IS MORE THE IS ALLOW");
            }else{
                wobble.setPower(output);
            }

            telemetry.addData("wobble acc", outputAcc);

            lastOutput = output;
        }

        telemetry.addData("wobble velocity", wobble.getVelocity());
        lastTarget = ticks;
    }

    public void setClipperOpen(){
        clipper.setPosition(Constant.K_wobble_clipperOpen);
    }

    public void setClipperClose(){
        clipper.setPosition(Constant.K_wobble_clipperClose);
    }
}
