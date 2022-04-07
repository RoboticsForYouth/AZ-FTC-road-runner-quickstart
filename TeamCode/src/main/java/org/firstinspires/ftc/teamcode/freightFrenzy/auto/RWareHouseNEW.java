package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Arm;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;
import org.firstinspires.ftc.teamcode.pipeline.FFDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "1RedWareHouseAuto")
public class RWareHouseNEW extends LinearOpMode {
    Trajectory sharedHubDrop;
    private LinearOpMode opMode;
    private FreightTool freightTool;
    private SampleMecanumDrive drive;
    private FFDetection cam;


    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;

        drive = new SampleMecanumDrive(hardwareMap);
        freightTool = new FreightTool(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cam = new FFDetection(this);
        cam.setup();
        freightTool.setupPos();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        int pos = cam.getPos();
        sleep(4000);
        telemetry.addData("Status: Initialized", pos);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        redAuto(startPose);
        //loop to start collection
        boolean freightDetected = false;


        sleep(5000);

        telemetry.addLine(freightTool.getDisplayValues());
        telemetry.update();


    }

    private void redAuto(Pose2d startPose) {

        final AutoUtil.AutoVars vars = getAutoVars(cam.getPos());
        telemetry.addData("Vars", vars);
        telemetry.update();
        TrajectorySequence toHub0 = getToHubTrajectorySequence(startPose, vars.initX, vars.initY);

        for(int i = 0; i < 3; i++) {
            TrajectorySequence toHub = toHub0;

            //second and third cycles drop freight at level 3
            if(i != 0) {
                AZUtil.runInParallel(() -> {
                    freightTool.setAllianceHubDropAuto(AutoUtil.AutoVars.RW_LEVEL3);
                });
                TrajectorySequence toHub12 = getToHubTrajectorySequence(startPose, AutoUtil.AutoVars.RW_LEVEL3.initX, AutoUtil.AutoVars.RW_LEVEL3.initY-3);
                toHub = toHub12;
            } else {
                //level based on detection for the first cycle
                AZUtil.runInParallel(() -> {
                    freightTool.setAllianceHubDropAuto(vars);
                });
            }
            drive.followTrajectorySequence(toHub);
            //wait until trajectory completed
            drive.waitForIdle();

            //wait until arm moved to drop position
            freightTool.waitUntilBusy();
            sleep(100);
            freightTool.dropFreightTeleOp();
            sleep(1500);

            //last angle should be zero so that teleop cn start at correct angle
            int angle = 0;
            if( i== 0) angle = 10;
            else if (i==1) angle = -10;
            final int intakeAngle = angle;

            AZUtil.runInParallel(() -> {
                freightTool.intakeWithAngle(intakeAngle);
//                freightTool.setTurnTablePos(intakeAngle);
            });
            TrajectorySequence toWarehouse = drive.trajectorySequenceBuilder(toHub.end())
                    .lineToSplineHeading(new Pose2d(0, -2, 0))
                    .forward(26+(i*2))
                    .build();
            drive.followTrajectorySequence(toWarehouse);
            drive.waitForIdle();
            freightTool.waitUntilBusy();
            autoIntake(angle);
            if( i< 2) {
                Trajectory toHome = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, -2, 0))
                        .build();
                drive.followTrajectory(toHome);
            }
        }
    }

    private TrajectorySequence getToHubTrajectorySequence(Pose2d startPose, int x, int y) {
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(0)))
                .build();
        return toHub;
    }

    private AutoUtil.AutoVars getAutoVars(int pos) {
        if( pos == 1){
            return AutoUtil.AutoVars.RW_LEVEL1;
        } else if (pos == 2){
            return AutoUtil.AutoVars.RW_LEVEL2;
        } else {
            return AutoUtil.AutoVars.RW_LEVEL3;
        }
    }

    public void autoIntake(int deg) {
        //double distance = sensor.getSensorDistance();

        //AZUtil.print(telemetry, "Object distance: ", distance);
        final Pose2d originalPos = drive.getPoseEstimate();
        Pose2d pos = originalPos;
        //sensorDistance = sensor.getSensorDistance();
        freightTool.startDetection();

        Trajectory move;
        double xPos = pos.getX();
        int count = 0;
        while (!freightTool.isFreightDetected() && opModeIsActive() && count < 3 ) {
            //sleep(1000);
            double distance1 = 3.0;


            // turnTable.turnTo(TurnTable.Direction.CLOCKWISE, deg - 5, false);
            move = drive.trajectoryBuilder(pos)
                    .lineToSplineHeading(new Pose2d(xPos + distance1, 0, 0))
                    .build();


            drive.followTrajectory(move);
            AZUtil.print(telemetry, "Pos", drive.getPoseEstimate());
            pos = move.end();
            xPos += distance1;
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
