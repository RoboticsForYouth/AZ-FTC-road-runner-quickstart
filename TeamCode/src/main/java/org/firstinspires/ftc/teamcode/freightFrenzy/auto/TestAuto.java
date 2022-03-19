package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous (name = "1TestAuto")
public class TestAuto extends LinearOpMode {
    Trajectory sharedHubDrop;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FreightTool freightTool = new FreightTool(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        freightTool.setupPos();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-9, 33, Math.toRadians(150)))
                .addDisplacementMarker(()->{ freightTool.drop();})
                .back(2)
                .lineToSplineHeading(startPose)
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->{freightTool.intake();})
                .forward(30)
                .build();
        waitForStart();
        AZUtil.runInParallel(()->{freightTool.setAllianceHubDrop();});
        drive.followTrajectorySequence(trajectorySequence);

        //loop to start collection
        boolean freightDetected = false;


        sleep(5000);

        telemetry.addLine(freightTool.getDisplayValues());
        telemetry.update();


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
