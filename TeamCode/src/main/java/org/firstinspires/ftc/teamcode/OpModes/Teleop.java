package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class Teleop extends OpMode {

    Robot robot = new Robot(this);

    @Override
    public void init() {
        robot.init(false);
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        robot.start();
    }

    @Override
    public void loop() {
        robot.teleop();
        telemetry.update();
    }
}
