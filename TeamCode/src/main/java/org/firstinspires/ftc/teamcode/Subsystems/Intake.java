package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor intake;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Intake(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");

        stati();

        telemetry.addData("intake", "ready to go");
    }

    public void intake(){
        intake.setPower(1);
        telemetry.addData("intake power", 1);
    }

    public void stati(){
        intake.setPower(0);
        telemetry.addData("intake power", 0);
    }

    public void emission(){
        intake.setPower(-1);
        telemetry.addData("intake power", -1);
    }

}
