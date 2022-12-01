package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;
import org.firstinspires.ftc.teamcode.freightFrenzy.pipeline.FFDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="1BlueWareHouseAuto")
public class BWareHouseNEW extends LinearOpMode {
    Trajectory sharedHubDrop;
    private LinearOpMode opMode;
    private SampleMecanumDrive drive;
    private FreightTool freightTool;
    private FFDetection cam;
    private Pose2d startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        int pos = cam.getPos();
        sleep(4000);
        telemetry.addLine("Status: Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        blueAutoNew();
    }

    private void blueAutoNew(){

        Pose2d orgPose = new Pose2d(0, 0, Math.toRadians(180));
        Pose2d startPose = orgPose;
        drive.setPoseEstimate(startPose);
        waitForStart();
        final AutoUtil.AutoVars vars = getAutoVars(cam.getPos());

        for(int i=0; i<3; i++) {
            if(i!=0){
                AZUtil.runInParallel(() -> {
                    freightTool.setBlueAllianceHubDropAuto1(AutoUtil.AutoVars.BW2_LEVEL3);
                });
            } else {
                AZUtil.runInParallel(() -> {
                    freightTool.setBlueAllianceHubDropAuto1(vars);
                });
            }

            //drive to hub and drop
            TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(vars.initX,
                            vars.initY, startPose.getHeading()))
                    .build();
            drive.followTrajectorySequence(toHub);
            drive.waitForIdle();
            freightTool.waitUntilBusy();
            //drop cube
            sleep(100);
            freightTool.dropFreightTeleOp();
            sleep(1500);


            //last angle should be zero for teleop
            int angle = 0;
            if( i== 0) angle = 10;
            else if (i==1) angle = -10;
            final int intakeAngle = angle;
            AZUtil.runInParallel(() -> {
                freightTool.intakeWithAngle(intakeAngle);
            });

            //to warehouse
            TrajectorySequence toWarehouse = drive.trajectorySequenceBuilder(toHub.end())
                    .lineToSplineHeading(new Pose2d(0, 2, Math.toRadians(0)))
                    .forward(26+(i*2))
                    .build();
            drive.followTrajectorySequence(toWarehouse);
            freightTool.waitUntilBusy();
            //autoIntake

            autoIntake();

            if( i!= 2) {
                TrajectorySequence toHome = drive.trajectorySequenceBuilder(toWarehouse.end())
                        .lineToSplineHeading(new Pose2d(startPose.getX(),
                                startPose.getY() - 1,
                                Math.toRadians(0)))
                        .build();
                drive.followTrajectorySequence(toHome);
                startPose = toHome.end();
            }
        }
    }


    private TrajectorySequence getToHubTrajectorySequence(Pose2d startPose, int x, int y) {
            TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(x, y, startPose.getHeading()))
                    .build();
            return toHub;
        }

        private AutoUtil.AutoVars getAutoVars(int pos) {
            if( pos == 1){
                return AutoUtil.AutoVars.BW_LEVEL1;
            } else if (pos == 2){
                return AutoUtil.AutoVars.BW_LEVEL2;
            } else {
                return AutoUtil.AutoVars.BW_LEVEL3;
            }
        }

    public void autoIntake() {
            //double distance = sensor.getSensorDistance();

            //AZUtil.print(telemetry, "Object distance: ", distance);
            final Pose2d originalPos = drive.getPoseEstimate();
            Pose2d pos = originalPos;
            //sensorDistance = sensor.getSensorDistance();
            freightTool.startDetection();
//            freightTool.intake();
            Trajectory move;
            double xPos = pos.getX();
            int count = 0;

            while (!freightTool.isFreightDetected() && opModeIsActive() && count < 1 ) {
                //sleep(1000);
                double distance1 = 3.0;

                move = drive.trajectoryBuilder(pos)
                        .lineToSplineHeading(new Pose2d(xPos + distance1, 0, pos.getHeading()))
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
//            freightTool.setTurnTablePos(0);
            freightTool.stopDetection();
            freightTool.stopIntake();
        }


    public void setup(){
        this.opMode = this;

        drive = new SampleMecanumDrive(hardwareMap);
        freightTool = new FreightTool(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cam = new FFDetection(this);
        cam.setup();
        freightTool.setupPos();
        startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

    }
}
