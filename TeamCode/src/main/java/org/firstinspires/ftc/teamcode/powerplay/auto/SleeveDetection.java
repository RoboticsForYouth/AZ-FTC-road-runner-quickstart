package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.powerplay.pipeline.PowerPlayPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class SleeveDetection extends LinearOpMode {
    private LinearOpMode opMode;
    OpenCvWebcam webcam;
    PowerPlayPipeline pipeline;

    public SleeveDetection(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public SleeveDetection() {
    }

    public void setup() {

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new PowerPlayPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam Error Code;", errorCode);
                telemetry.update();
            }
        });


    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;

        setup();
        waitForStart();

        while (opModeIsActive()) {
            int pos = pipeline.getAnalysis();

            telemetry.addData("Position: ", pos);
            telemetry.addData("Avg: ", pipeline.getAvg());
            telemetry.update();
            sleep(1000);
        }
    }


}
