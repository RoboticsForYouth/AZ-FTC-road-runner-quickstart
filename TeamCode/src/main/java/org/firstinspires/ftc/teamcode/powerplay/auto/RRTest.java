package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aztools.AZUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRTest extends LinearOpMode {


    private SampleMecanumDrive drive;
    private Pose2d origin;
    private TrajectorySequence dropConeTrajectory;
    private ConeTool coneTool;
    private TrajectorySequence grabTrajectorySequence;
    private Trajectory dropTrajectory;
    private ElapsedTime runTime = new ElapsedTime();
    private MultipleTelemetry multiTelemetry;

    public void initAuto() {
        coneTool = new ConeTool(this);

        drive = new SampleMecanumDrive(hardwareMap);
        multiTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.addData("Status", "Initialized");
        multiTelemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        origin = new Pose2d(-64, 36, Math.toRadians(0));
        Pose2d highJuncPos = new Pose2d(-7, 32, Math.toRadians(300));
        Pose2d grabConePos = new Pose2d(-15 , 66, Math.toRadians(90));
        drive.setPoseEstimate(origin);

        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                })
                .splineToConstantHeading(new Vector2d(-30, 36), Math.toRadians(0))
                .splineToSplineHeading(highJuncPos, Math.toRadians(0))
                .build();

        drive.setPoseEstimate(highJuncPos);

        grabTrajectorySequence =  drive.trajectorySequenceBuilder(highJuncPos)
                .splineToConstantHeading(new Vector2d(-15 , 36), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-15 , 55, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(()-> {grabConeWhenDetectedAsync();})
                .lineToConstantHeading(new Vector2d(-15 , 66))
                .build();
/*
        dropTrajectorySequence = drive.trajectorySequenceBuilder(grabTrajectorySequence.end())
                .lineToConstantHeading(new Vector2d( -12, 60))
                .splineToLinearHeading(new Pose2d(-12, 40, Math.toRadians(0)), Math.toRadians(0))
//1.                .lineToLinearHeading(new Pose2d(-16, 40, Math.toRadians(60)))
                .splineToLinearHeading(highJuncPos, Math.toRadians(0))
                .build();

 */
        dropTrajectory = drive.trajectoryBuilder(highJuncPos)
                .lineToLinearHeading(new Pose2d(-14, 65.5, Math.toRadians(0)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        runTime.reset();

        drive.followTrajectory(dropTrajectory);
        multiTelemetry.addData("ElapsedTime:", runTime.milliseconds());
        multiTelemetry.update();
        sleep(3000);
    }
    public void grabConeWhenDetectedAsync() {
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                while(!coneTool.isConeDetected()) {
                    Thread.yield();
                }
                coneTool.grabCone();
                multiTelemetry.addLine("Cone Detected");
                multiTelemetry.update();
            }
        });

    }
}
