package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;

enum RobotState {
    travel,
    intake,
    wobble,
    intakeEmission
}

enum IntakeState {
    stati,
    intake,
    emission
}

enum WobbleState {
    up,
    down
}

public class Robot {
    public DriveTrain dt;
    public Intake intake;
    public Wobble wobble;

    private Gamepad driver;
    private Gamepad operator;
    private Telemetry telemetry;

    private RobotState robotState = RobotState.travel;
    private IntakeState intakeState = IntakeState.stati;
    private WobbleState wobbleState = WobbleState.down;

    private ElapsedTime time = new ElapsedTime();


    public Robot(OpMode opMode){
        driver = opMode.gamepad1;
        operator = opMode.gamepad2;

        telemetry = opMode.telemetry;

        dt = new DriveTrain(opMode);
        intake = new Intake(opMode);
        wobble = new Wobble(opMode);
    }

    public void init(boolean auto) {
        dt.init(auto);
        intake.init();
        wobble.init(auto);
        telemetry.addData("robot", "ready to go");
    }

    public void start(){
        time.startTime();
    }

    public void teleop(){
        dt.teleop(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x);

        if (operator.a)robotState = RobotState.travel;
        if (operator.x)robotState = RobotState.intake;
        if (operator.b) robotState = RobotState.wobble;
        if (operator.dpad_left) robotState = RobotState.intakeEmission;

        switch (robotState){
            case travel:
                intakeState = IntakeState.stati;
                wobbleState = WobbleState.up;
                break;

            case intake:
                intakeState = IntakeState.intake;
                wobbleState = WobbleState.up;
                break;

            case wobble:
                intakeState = IntakeState.stati;
                wobbleState = WobbleState.down;
                break;

            case intakeEmission:
                intakeState = IntakeState.emission;
                wobbleState = WobbleState.up;
                break;
        }

        switch (intakeState){
            case stati:
                intake.stati();
                break;

            case intake:
                intake.intake();
                break;

            case emission:
                intake.emission();
                break;
        }

        switch (wobbleState){
            case up:
                wobble.setClipperClose();
                wobble.SetWobblePos(Constant.K_wobble_wobbleUp, time);
                break;

            case down:
                wobble.setClipperOpen();
                wobble.SetWobblePos(Constant.K_wobble_wobbleDown, time);
                break;
        }
    }
}
