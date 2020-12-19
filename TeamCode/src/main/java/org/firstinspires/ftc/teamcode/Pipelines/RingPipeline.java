package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingPipeline extends OpenCvPipeline {

    public double x = 0; //pixels left right
    public double y = 0; //pixels up down
    public double z = 0; //pixels size

    @Override
    public Mat processFrame(Mat input) {

        return input;
    }

}
