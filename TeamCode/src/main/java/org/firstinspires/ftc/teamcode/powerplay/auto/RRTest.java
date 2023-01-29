package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aztools.AZUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRTest extends LinearOpMode {


    private SampleMecanumDrive drive;
    private Pose2d origin;
    private TrajectorySequence dropConeTrajectory, goToConeStack, grabConeStackTrajectorySequence, dropConeStackTrajectorySequence;
    private ConeTool coneTool;
    private TrajectorySequence grabTrajectorySequence;
    private Trajectory dropTrajectory, pos2Trajectory, pos3Trajectory, pos1Trajectory;
    private ElapsedTime runTime = new ElapsedTime();
    private MultipleTelemetry multiTelemetry;
    int slowVelocity = 33;
    int slowAcc = 33;
    private Lift.LiftLevel[] levels = new Lift.LiftLevel[]{
            Lift.LiftLevel.CONE_5,
            Lift.LiftLevel.CONE_4,
            Lift.LiftLevel.CONE_3,
            Lift.LiftLevel.CONE_2,
            Lift.LiftLevel.CLEAR
    };

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
        Pose2d mediumJuncPos = new Pose2d(-28, 30.5, Math.toRadians(300));
        Pose2d grabConePos = new Pose2d(-15 , 66, Math.toRadians(90));
        drive.setPoseEstimate(origin);
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(slowVelocity, slowAcc, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(slowAcc);
//        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
//                //move to high junction
//                .addDisplacementMarker(() -> {
//                    coneTool.liftTo(Lift.LiftLevel.HIGH);
//                })
//                .splineToConstantHeading(new Vector2d(-30, 36), Math.toRadians(0))
//                .splineToSplineHeading(highJuncPos, Math.toRadians(0))
//                .build();
//
//        drive.setPoseEstimate(highJuncPos);
//
//        grabTrajectorySequence =  drive.trajectorySequenceBuilder(highJuncPos)
//                .splineToConstantHeading(new Vector2d(-15 , 36), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-15 , 55, Math.toRadians(90)), Math.toRadians(0))
//                .addDisplacementMarker(()-> {grabConeWhenDetectedAsync();})
//                .lineToConstantHeading(new Vector2d(-15 , 66))
//                .build();
/*
        dropTrajectorySequence = drive.trajectorySequenceBuilder(grabTrajectorySequence.end())
                .lineToConstantHeading(new Vector2d( -12, 60))
                .splineToLinearHeading(new Pose2d(-12, 40, Math.toRadians(0)), Math.toRadians(0))
//1.                .lineToLinearHeading(new Pose2d(-16, 40, Math.toRadians(60)))
                .splineToLinearHeading(highJuncPos, Math.toRadians(0))
                .build();

 */
        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.MEDIUM);
                })
                .splineToConstantHeading(new Vector2d(-45, 36), Math.toRadians(0), velocityConstraint, accelerationConstraint)
                .splineToSplineHeading(mediumJuncPos, Math.toRadians(0), velocityConstraint, accelerationConstraint)
                .build();
        goToConeStack = drive.trajectorySequenceBuilder(dropConeTrajectory.end())
                .back(8)
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.CONE_5);
                })
                .lineToLinearHeading(new Pose2d(-12.5, 38, Math.toRadians(0)), velocityConstraint, accelerationConstraint)
                .lineToLinearHeading(new Pose2d(-12.5, 42, Math.toRadians(90)), velocityConstraint, accelerationConstraint)
                .lineToConstantHeading(new Vector2d(-12.8, 63), velocityConstraint, accelerationConstraint)
                .build();

        dropConeStackTrajectorySequence = drive.trajectorySequenceBuilder(goToConeStack.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(-13, 55, Math.toRadians(208)), velocityConstraint, accelerationConstraint)
                .forward(5)
                .build();

        grabConeStackTrajectorySequence = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(-12.8, 55, Math.toRadians(90)), velocityConstraint, accelerationConstraint)
                //.splineToLinearHeading(new Pose2d(-16, 55, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    grabConeWhenDetectedAsync();
                })
                .lineToConstantHeading(new Vector2d(-12.8, 63), velocityConstraint, accelerationConstraint)
                .build();

        pos1Trajectory = drive.trajectoryBuilder(dropConeStackTrajectorySequence.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-15, 63, Math.toRadians(180)), velocityConstraint, accelerationConstraint)
                .build();
        pos2Trajectory = drive.trajectoryBuilder(dropConeStackTrajectorySequence.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-14, 37, Math.toRadians(180)), velocityConstraint, accelerationConstraint)
                .build();
        pos3Trajectory = drive.trajectoryBuilder(dropConeStackTrajectorySequence.end(), true)
                .back(4)
                .lineToLinearHeading(new Pose2d(-16, 12, Math.toRadians(180)), velocityConstraint, accelerationConstraint)
                .build();
        coneTool.grabCone();
        waitForStart();

        if(isStopRequested()) return;

        runTime.reset();

        drive.followTrajectorySequence(dropConeTrajectory);
        coneTool.dropCone();
        drive.followTrajectorySequence(goToConeStack);
        coneTool.grabCone();
        sleep(250);
        coneTool.liftTo(Lift.LiftLevel.LOW);
        drive.followTrajectorySequence(dropConeStackTrajectorySequence);
        coneTool.dropCone();
        for (int i = 1; i < 3; i++) {
            final int index = i;
            AZUtil.runInParallel(new Runnable() {
                @Override
                public void run() {
                    coneTool.liftTo(Lift.LiftLevel.LOW);
                    sleep(500);
                    coneTool.liftTo(levels[index]);
                }
            });

            drive.followTrajectorySequence(grabConeStackTrajectorySequence);
            coneTool.grabCone();
            sleep(250);
            coneTool.liftTo(Lift.LiftLevel.LOW);
            drive.followTrajectorySequence(dropConeStackTrajectorySequence);
            coneTool.dropCone();
        }
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
