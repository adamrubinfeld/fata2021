package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Pipelines.RingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision {
    private OpenCvCamera camera;
    private RingPipeline pipeline;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    int camView = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
            "id", hardwareMap.appContext.getPackageName());


    public Vision(OpMode opMode, RingPipeline pipeline){
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        this.pipeline = pipeline;
    }

    public void init(){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camView);

        telemetry.addData("vision", "ready to go");
    }

    public void startCam(){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(Constant.K_vision_width, Constant.K_vision_height);
            }

        });
    }

    public void stopCam(){
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {

            @Override
            public void onClose() {
                camera.closeCameraDevice();
            }
        });
    }

    public RingPipeline getPipeline(){
        return pipeline;
    }

}
