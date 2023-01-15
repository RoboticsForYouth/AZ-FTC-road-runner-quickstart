package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.aztools.AZUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ConeTool coneTool;
    SampleMecanumDrive drive;
    SleeveDetection sleeveDetection;
    int pos;
    Trajectory dropCone, dropCone2, dropCone3, park;
    private TrajectorySequence[] grabTrajectorySequenceList = new TrajectorySequence[5];
    private TrajectorySequence[] dropTrajectorySequenceList = new TrajectorySequence[5];
    Pose2d origin;
    private Vector2d backUp;
    private Trajectory coneStackTrajectoryDropSequence;
    private TrajectorySequence dropConeStackTrajectorySequence;

    public enum FieldPos {
        LEFT,
        RIGHT
    }

    FieldPos fieldPos = FieldPos.LEFT;

    int originX = 0;
    int originY = 0;
    int originHeading = 0;
    private TrajectorySequence dropConeTrajectory;
    private TrajectorySequence grabConeStackTrajectorySequence;

    private Lift.LiftLevel[] levels = new Lift.LiftLevel[]{
            Lift.LiftLevel.CONE_5,
            Lift.LiftLevel.CONE_4,
            Lift.LiftLevel.CONE_3,
            Lift.LiftLevel.CONE_2,
            Lift.LiftLevel.CLEAR
    };


    public void initAuto() {
        coneTool = new ConeTool(this);
        sleeveDetection = new SleeveDetection(this);
        sleeveDetection.setup();
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
    }


    public void setupPos() {
        coneTool.grabCone();
    }

    public void setFieldPos(FieldPos fieldPos) {
        this.fieldPos = fieldPos;
    }

    public void detection() {
        pos = sleeveDetection.getPos();
        telemetry.addData("Position", pos);
        telemetry.update();
    }

    public void setUpLeftSideTrajectory() {
        origin = new Pose2d(-64, 36, Math.toRadians(0));
        drive.setPoseEstimate(origin);
        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                })
                .splineTo(new Vector2d(-12, 36), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-5, 32, Math.toRadians(300)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(-30, 36), Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(-7, 32, Math.toRadians(300)), Math.toRadians(0))
                .build();

        grabConeStackTrajectorySequence = drive.trajectorySequenceBuilder(dropConeTrajectory.end())
                //.splineToConstantHeading(new Vector2d(-18, 36), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-14.5, 36))
                //.turn(Math.toRadians(150))
                //.splineToLinearHeading(new Pose2d(-18, 66, Math.toRadians(90)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-14, 55, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-13, 65))
                .build();
        dropConeStackTrajectorySequence = drive.trajectorySequenceBuilder(grabConeStackTrajectorySequence.end())
                .lineToLinearHeading(new Pose2d(-16, 40, Math.toRadians(60)))
                .lineToLinearHeading(dropConeTrajectory.end()).build();
        for(int i = 0; i < 5; i++) {
            double factor = 1.5;
            grabTrajectorySequenceList[i] = drive.trajectorySequenceBuilder(dropConeTrajectory.end())
                    .lineToConstantHeading(new Vector2d(-14.5 + (i/factor), 36))
                    .lineToLinearHeading(new Pose2d(-15 + (i/factor), 55, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-13 + (i/factor), 65))
                    .build();
            dropTrajectorySequenceList[i] = drive.trajectorySequenceBuilder(grabTrajectorySequenceList[i].end())
                    .lineToLinearHeading(new Pose2d(-16, 40, Math.toRadians(60)))
                   .lineToLinearHeading(new Pose2d(-4 + (i/factor), 32 + (i/4), Math.toRadians(300)))
                    .build();
        }
    }


    private void dropLiftToZero() {
        coneTool.liftTo(Lift.LiftLevel.ZERO);
    }

    private void dropCone() {
        coneTool.dropCone();
    }

    public void setUpRightSideTrajectory() {
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        switch (fieldPos) {
            case LEFT:
                setUpLeftSideTrajectory();
                break;
            case RIGHT:
                setUpRightSideTrajectory();
                break;
        }
        telemetry.addData("Trajectory", "Created");
        telemetry.update();
        coneTool.grabCone();
        waitForStart();

//        AZUtil.runInParallel(new Runnable() {
//            @Override
//            public void run() {
//                coneTool.liftTo(Lift.LiftLevel.HIGH);
//            }
//        });
//        sleep(1000);
        drive.followTrajectorySequence(dropConeTrajectory);
        coneTool.dropCone();
        for (int i = 0; i < 5; i++) {
            final int index = i;
            AZUtil.runInParallel(new Runnable() {
                @Override
            public void run() {
                sleep(500);
                coneTool.liftTo(levels[index]);
            }
            });

            drive.followTrajectorySequence(grabTrajectorySequenceList[i]);
            coneTool.grabCone();
            sleep(250);
            AZUtil.runInParallel(new Runnable() {
                @Override
                public void run() {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                }
            });

            drive.followTrajectorySequence(dropTrajectorySequenceList[i]);
            coneTool.dropCone();
        }


        //TODO: need to accurately determine the time to ensure < 30 secs
        sleep(5000);


    }

}
