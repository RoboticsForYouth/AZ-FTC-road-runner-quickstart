package org.firstinspires.ftc.teamcode.sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Saurav Test Auto", group="Saurav Test Auto")
public class SauravTestAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private SauravTestAutoBase base = null;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        base = new SauravTestAutoBase(this);

        base.init();
        waitForStart();
        runtime.reset();

        base.setGrip(1);
        sleep(2000);

        base.moveForwardEncoder(3500, 0.5);
        sleep(1000);
        base.turnRight90();
        sleep(1000);
//        base.moveForwardEncoder(500, 0.5);
//        sleep(1000);

        base.setArm(0.375);
        sleep(2000);
        base.setGrip(0);
        sleep(2000);
        base.setArm(0.85);
        sleep(2000);

        base.stop();
    }
}