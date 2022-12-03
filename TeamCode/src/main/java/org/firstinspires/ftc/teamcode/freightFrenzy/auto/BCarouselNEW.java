package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Carousel;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;
import org.firstinspires.ftc.teamcode.freightFrenzy.pipeline.FFDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Autonomous(name = "2BlueCarouselAuto")
public class BCarouselNEW extends LinearOpMode {
    Trajectory sharedHubDrop;
    private LinearOpMode opMode;
    private FreightTool freightTool;
    private SampleMecanumDrive drive;
    private FFDetection cam;
    private Carousel duckyTool;


    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;

        drive = new SampleMecanumDrive(hardwareMap);
        freightTool = new FreightTool(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cam = new FFDetection(this);
        duckyTool = new Carousel(this);
        cam.setup();
        duckyTool.setup();
        freightTool.setupPos();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        int pos = cam.getPos();
        sleep(4000);
        telemetry.addLine("Status: Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        carouselAuto(startPose);

        sleep(5000);

        telemetry.addLine(freightTool.getDisplayValues());
        telemetry.update();


    }

    private void carouselAuto(Pose2d startPose) {

        final AutoUtil.AutoVars vars = getAutoVars(cam.getPos());
        telemetry.addData("Vars", vars);
        telemetry.update();
        TrajectorySequence toHub = getToHubTrajectorySequence(startPose, vars.initX, vars.initY);

        AZUtil.runInParallel(() -> {
            freightTool.setAllianceHubDropAuto(vars);
        });
        drive.followTrajectorySequence(toHub);
        //wait until trajectory completed
        drive.waitForIdle();

        //wait until arm moved to drop position
        freightTool.waitUntilBusy();
        sleep(100);
        freightTool.dropFreightTeleOp();
        sleep(1500);
        AZUtil.runInParallel(() -> {
            freightTool.reset();
        });
        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(toHub.end())
                .lineToSplineHeading(new Pose2d(22, 7, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(toCarousel);
        drive.waitForIdle();
        duckyTool.blueDuckyDrop();


        Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(28, 28, Math.toRadians(180)))
                .build();
        drive.followTrajectory(toPark);


    }

    private TrajectorySequence getToHubTrajectorySequence(Pose2d startPose, int x, int y) {
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(0)))
                .build();
        return toHub;
    }

    private AutoUtil.AutoVars getAutoVars(int pos) {
        if( pos == 1){
            return AutoUtil.AutoVars.BC_LEVEL1;
        } else if (pos == 2){
            return AutoUtil.AutoVars.BC_LEVEL2;
        } else {
            return AutoUtil.AutoVars.BC_LEVEL3;
        }
    }



}
