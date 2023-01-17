package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.powerplay.pipeline.PowerPlayRGBPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class SleeveDetection extends LinearOpMode {
    private LinearOpMode opMode;
    private Point topLeft;
    private int width;
    private int height;
    OpenCvWebcam webcam;
    PowerPlayRGBPipeline rgbPipeline;

    public SleeveDetection(LinearOpMode opMode, Point topLeft, int width, int height) {
        this.opMode = opMode;
        this.topLeft = topLeft;
        this.width = width;
        this.height = height;
    }

    public  SleeveDetection(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public SleeveDetection() {
    }

    public void setup() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
//        bluePipeline = new PowerPlayBluePipeline();
        rgbPipeline = new PowerPlayRGBPipeline();
        if(  topLeft != null) {
            rgbPipeline.setBoundingBox(topLeft, width, height);
        }
        webcam.setPipeline(rgbPipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam Error Code;", errorCode);
                telemetry.update();
            }
        });


    }

    public int getPos() {
        //run it 3 times, sometimes takes time
        for (int i = 0; i < 3; i++) {
            rgbPipeline.getAnalysis();
        }
        return rgbPipeline.getAnalysis();
    }

    public int[] getAvgs(){
        return rgbPipeline.getAvg();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        topLeft = new Point(320, 149);
        width = 60;
        height = 90;

        setup();
        waitForStart();

        while (opModeIsActive()) {
            int pos = rgbPipeline.getAnalysis();

            telemetry.addData("Position: ", pos);
            int[] avg = rgbPipeline.getAvg();
            telemetry.addData("Red: ", avg[0]);
            telemetry.addData("Green: ", avg[1]);
            telemetry.addData("Blue: ", avg[2]);
            telemetry.update();
            sleep(1000);
        }
    }


}
