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

@Autonomous(name = "2RedCarouselAuto")
public class RCarouselNEW extends LinearOpMode {
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
        sleep(3000);
        AZUtil.runInParallel(() -> {
            freightTool.reset();
        });
        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(toHub.end())
                .lineToSplineHeading(new Pose2d(-22, 15, 0))
                .strafeRight(9)
                .build();
        drive.followTrajectorySequence(toCarousel);
        drive.waitForIdle();
        duckyTool.redDuckyDrop();

        //NEW STUFF***
//        AZUtil.runInParallel(() -> {
//            freightTool.intakeWithAngle(0);
//        });
//        Trajectory toDuckyPickup = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(-90)))
//                .build();
//
//        drive.followTrajectory(toDuckyPickup);
//        drive.waitForIdle();
//        freightTool.waitUntilBusy();
//        autoIntake(-90);
//        freightTool.setWristSecured();
//        sleep(500);
//        TrajectorySequence toHub2 = getToHubTrajectorySequence(drive.getPoseEstimate(), AutoUtil.AutoVars.RC_LEVEL3.initX, AutoUtil.AutoVars.RC_LEVEL3.initY);
//        AZUtil.runInParallel(() -> {
//            freightTool.setDuckHupDropAuto();
//        });
//        drive.followTrajectorySequence(toHub2);
//        drive.waitForIdle();
//        freightTool.wristLevel1Pos();
//        //wait until arm moved to drop position
//        freightTool.waitUntilBusy();
//        sleep(100);
//        freightTool.dropFreightTeleOp();
//        sleep(1500);
//        AZUtil.runInParallel(() -> {
//            freightTool.reset();
//        });
        //^^^^ NEW STUFF****


        Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-30, 29, 0))
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
            return AutoUtil.AutoVars.RC_LEVEL1;
        } else if (pos == 2){
            return AutoUtil.AutoVars.RC_LEVEL2;
        } else {
            return AutoUtil.AutoVars.RC_LEVEL3;
        }
    }

    public void autoIntake(int deg) {
        //double distance = sensor.getSensorDistance();

        //AZUtil.print(telemetry, "Object distance: ", distance);
        final Pose2d originalPos = drive.getPoseEstimate();
        Pose2d pos = originalPos;
        //sensorDistance = sensor.getSensorDistance();
        freightTool.startDetection();

        TrajectorySequence move;
        double xPos = pos.getX();
        double yPos = pos.getY();
        int count = 0;
        while (!freightTool.isFreightDetected() && opModeIsActive() && count < 3 ) {
            //sleep(1000);
            double distance1 = 13.0;
            int degChange = 10;

            //turnTable.turnTo(TurnTable.Direction.CLOCKWISE, deg - 5, false);
            move = drive.trajectorySequenceBuilder(pos)
                    .lineToSplineHeading(new Pose2d(xPos, yPos-distance1, Math.toRadians(deg)))
                    .lineToLinearHeading(originalPos)
                    .build();

            if(count == 0) deg += degChange;
            if(count == 1) deg -= degChange * 2.5;

            drive.followTrajectorySequence(move);
            AZUtil.print(telemetry, "Pos", drive.getPoseEstimate());
            pos = move.end();
            //yPos -= distance1;
            count++;

        }
        if(!pos.epsilonEquals(originalPos)) {
            Trajectory home = drive.trajectoryBuilder(pos)
                    .lineToLinearHeading(originalPos)
                    .build();
            drive.followTrajectory(home);
        }
        freightTool.stopDetection();
        freightTool.stopIntake();

    }



}
