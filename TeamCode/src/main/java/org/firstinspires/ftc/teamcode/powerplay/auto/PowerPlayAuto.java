package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.powerplay.pipeline.PowerPlayPipeline;

@Autonomous
public class PowerPlayAuto extends LinearOpMode {
    PowerPlayPipeline cam;

    @Override
    public void runOpMode() throws InterruptedException {
        cam = new PowerPlayPipeline();

        waitForStart();

        int pos = cam.getAnalysis();
        telemetry.addData("Position", pos);
        telemetry.update();

        sleep(10000);

    }
}
