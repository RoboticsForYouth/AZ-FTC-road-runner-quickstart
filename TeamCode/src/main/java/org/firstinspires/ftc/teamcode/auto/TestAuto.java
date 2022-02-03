package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.FreightTool;

@TeleOp
public class TestAuto extends LinearOpMode {
    Trajectory sharedHubDrop;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FreightTool freightTool = new FreightTool(this);
        waitForStart();
        freightTool.setupPos();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean run = true;
        while(opModeIsActive()) {
            if(run) {

                autoSharedHubDrop(drive, freightTool);


                //sleep(1000);
                run = false;
            }
            telemetry.addLine(freightTool.getDisplayValues());
            telemetry.update();
        }


    }

    private void autoSharedHubDrop(SampleMecanumDrive drive, FreightTool freightTool) {
        sharedHubDrop = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-12, 0, 0)).build();

        Trajectory home = drive.trajectoryBuilder(sharedHubDrop.end()).
                lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0))).build();
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                freightTool.prepSharedHubDrop();

            }
        });
        drive.followTrajectory(sharedHubDrop);
        freightTool.waitUntilBusy();
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                freightTool.moveTo0();
            }
        });
        drive.followTrajectory(home);
    }
}
