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
public class RWareHouseAuto extends LinearOpMode {
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


//         testPath();

        redAuto(startPose);
        //loop to start collection
        boolean freightDetected = false;


        sleep(5000);

        telemetry.addLine(freightTool.getDisplayValues());
        telemetry.update();


    }

    private void redAuto(Pose2d startPose) {

        int pos = cam.getPos();
        telemetry.addData("Pos, wait 4 seconds", pos);
        telemetry.update();
        waitForStart();

        final AutoVars vars = getAutoVars(cam.getPos());
        telemetry.addData("Vars", vars);
        telemetry.update();
        TrajectorySequence toHub0 = getToHubTrajectorySequence(startPose, vars.initX, vars.initY);

        for(int i = 0; i < 3; i++) {
            TrajectorySequence toHub = toHub0;
            if(i != 0) {
                AZUtil.runInParallel(() -> {
                    freightTool.setAllianceHubDropAuto(AutoVars.LEVEL3.tsePos,
                            AutoVars.LEVEL3.turnTableAngle,
                            AutoVars.LEVEL3.level);
                });
                TrajectorySequence toHub12 = getToHubTrajectorySequence(startPose, AutoVars.LEVEL3.initX, AutoVars.LEVEL3.initY-3);
                toHub = toHub12;
            } else {
                AZUtil.runInParallel(() -> {
                    freightTool.setAllianceHubDropAuto(vars.tsePos, vars.turnTableAngle, vars.getLevel());
                });
            }

            drive.followTrajectorySequence(toHub);
            freightTool.dropFreightTeleOp();
            sleep(1500);
            AZUtil.runInParallel(() -> {
                sleep(800);
                freightTool.intake();
            });
            TrajectorySequence toWarehouse = drive.trajectorySequenceBuilder(toHub.end())
                    .lineToSplineHeading(new Pose2d(0, -2, 0))
                    .forward(26)
                    .build();
            drive.followTrajectorySequence(toWarehouse);
            int angle = 0;
            if( i== 1) angle = 5;
            else if (i==2) angle = -5;
            autoIntake(angle);
            Trajectory toHome = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(0, -2, 0))
                    .build();
            drive.followTrajectory(toHome);
        }
    }

    private TrajectorySequence getToHubTrajectorySequence(Pose2d startPose, int x, int y) {
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(0)))
                .build();
        return toHub;
    }

    private AutoVars getAutoVars(int pos) {
        if( pos == 1){
            return AutoVars.LEVEL1;
        } else if (pos == 2){
            return AutoVars.LEVEL2;
        } else {
            return AutoVars.LEVEL3;
        }
    }

    private void autoSharedHubDrop(SampleMecanumDrive drive, FreightTool freightTool) {
        sharedHubDrop = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-12, 0, 0)).build();

        Trajectory home = drive.trajectoryBuilder(sharedHubDrop.end()).
                lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0))).build();
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

    public enum AutoVars{
        LEVEL1(1, 0, 30, 150, Arm.ArmLevel.LEVEL1),
        LEVEL2(1, -4, 34, 150, Arm.ArmLevel.LEVEL2),
        LEVEL3(1, -6, 35, 150, Arm.ArmLevel.LEVEL3);
        public int turnTableAngle;
        private Arm.ArmLevel level;
        public int tsePos;
        public int initX;
        public int initY;

        AutoVars(int tsePos, int initX, int initY, int turnTableAngle, Arm.ArmLevel level) {

            this.tsePos = tsePos;
            this.initX = initX;
            this.initY = initY;
            this.turnTableAngle = turnTableAngle;
            this.level = level;
        }

        public int getTurnTableAngle() {
            return turnTableAngle;
        }

        public int getTsePos() {
            return tsePos;
        }

        public int getInitX() {
            return initX;
        }

        public Arm.ArmLevel getLevel() {
            return level;
        }

        public int getInitY() {
            return initY;
        }
    }

}
