package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Pipelines.RingPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

enum RobotState {
    travel,
    intake,
    shoot,
    wobble,
    intakeEmission,
}

enum DriveTrainState {
    vision,
    driver
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

enum VisionState {
    open,
    close
}

public class Robot {
    public DriveTrain dt;
    public Intake intake;
    public Wobble wobble;
    public Vision vision;

    private Gamepad driver;
    private Gamepad operator;
    private Telemetry telemetry;

    private RobotState robotState = RobotState.travel;
    private DriveTrainState dtState = DriveTrainState.driver;
    private IntakeState intakeState = IntakeState.stati;
    private WobbleState wobbleState = WobbleState.down;
    private VisionState visionState = VisionState.close;

    private ElapsedTime time = new ElapsedTime();


    public Robot(OpMode opMode){
        driver = opMode.gamepad1;
        operator = opMode.gamepad2;

        telemetry = opMode.telemetry;

        dt = new DriveTrain(opMode);
        intake = new Intake(opMode);
        wobble = new Wobble(opMode);
        vision = new Vision(opMode, new RingPipeline());
    }

    public void init(boolean auto) {
        dt.init(auto);
        intake.init();
        wobble.init(auto);
        vision.init();
        telemetry.addData("robot", "ready to go");
    }

    public void start(){
        time.startTime();
    }

    public void teleop(){
        if (operator.a)robotState = RobotState.travel;
        if (operator.x)robotState = RobotState.intake;
        if (operator.b) robotState = RobotState.wobble;
        if (operator.dpad_left) robotState = RobotState.intakeEmission;

        switch (robotState){
            case travel:
                dtState = DriveTrainState.driver;
                intakeState = IntakeState.stati;
                wobbleState = WobbleState.up;
                visionState = VisionState.close;
                break;

            case intake:
                dtState = DriveTrainState.driver;
                intakeState = IntakeState.intake;
                wobbleState = WobbleState.up;
                visionState = VisionState.close;
                break;

            case shoot:
                dtState = DriveTrainState.vision;
                intakeState = IntakeState.stati;
                wobbleState = WobbleState.up;
                visionState = VisionState.open;
                break;

            case wobble:
                dtState = DriveTrainState.driver;
                intakeState = IntakeState.stati;
                wobbleState = WobbleState.down;
                visionState = VisionState.close;
                break;

            case intakeEmission:
                dtState = DriveTrainState.driver;
                intakeState = IntakeState.emission;
                wobbleState = WobbleState.up;
                visionState = VisionState.close;
                break;
        }

        switch (dtState){
            case driver:
                dt.driver(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x, time);
                break;

            case vision:
                dt.vision(vision.getPipeline(), time);
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

        switch (visionState){
            case open:
                vision.startCam();
                break;

            case close:
                vision.stopCam();
                break;
        }
    }
}
