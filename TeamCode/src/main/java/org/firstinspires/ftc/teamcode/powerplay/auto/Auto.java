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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

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
    private Trajectory pos2Trajectory;
    private Trajectory pos1Trajectory;
    private Trajectory pos3Trajectory;
    private TrajectorySequence grabConeStackTrajectorySequence;
    private TrajectorySequence dropConeStackTrajectorySequence;
    private TrajectorySequence dropConeTrajectory;


    public Rect getSleeveDetectionBoundingBox() {
        return new Rect(new Point(320, 149), new Size(60, 90));
    }

    public enum FieldPos {
        LEFT,
        RIGHT
    }

    FieldPos fieldPos = FieldPos.LEFT;

    int originX = 0;
    int originY = 0;
    int originHeading = 0;


    private Lift.LiftLevel[] levels = new Lift.LiftLevel[]{
            Lift.LiftLevel.CONE_5,
            Lift.LiftLevel.CONE_4,
            Lift.LiftLevel.CONE_3,
            Lift.LiftLevel.CONE_2,
            Lift.LiftLevel.CLEAR
    };


    public void initAuto() {
        coneTool = new ConeTool(this);
        sleeveDetection = new SleeveDetection(this, getSleeveDetectionBoundingBox().tl(),
                getSleeveDetectionBoundingBox().width, getSleeveDetectionBoundingBox().height);
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
        Pose2d highJuncPos = new Pose2d(-7, 31, Math.toRadians(300));

        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                })
                .splineToConstantHeading(new Vector2d(-30, 36), Math.toRadians(0))
                .splineToSplineHeading(highJuncPos, Math.toRadians(0))
                .build();

        pos2Trajectory = drive.trajectoryBuilder(highJuncPos)
                .lineToLinearHeading(new Pose2d(-14, 37, Math.toRadians(0)))
                .build();
        pos1Trajectory = drive.trajectoryBuilder(highJuncPos)
                .lineToLinearHeading(new Pose2d(-14, 65.5, Math.toRadians(0)))
                .build();
        pos3Trajectory = drive.trajectoryBuilder(highJuncPos, true)
                .splineToLinearHeading(new Pose2d(-16, 10, Math.toRadians(0)), Math.toRadians(45))
                .build();

        grabConeStackTrajectorySequence = drive.trajectorySequenceBuilder(highJuncPos)
                .back(3)
                .lineToLinearHeading(new Pose2d(-14.5, 36, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(-16, 55, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    grabConeWhenDetectedAsync();
                })
                .lineToConstantHeading(new Vector2d(-14.5, 64))
                .build();
        dropConeStackTrajectorySequence = drive.trajectorySequenceBuilder(grabConeStackTrajectorySequence.end())
                .lineToLinearHeading(new Pose2d(-14.5, 40, Math.toRadians(60)))
                .splineToLinearHeading(new Pose2d(highJuncPos.getX(), highJuncPos.getY()-1, highJuncPos.getHeading()), Math.toRadians(0))
                .build();
    }


    private void dropLiftToZero() {
        coneTool.liftTo(Lift.LiftLevel.ZERO);
    }

    private void dropCone() {
        coneTool.dropCone();
    }

    public void setUpRightSideTrajectory() {
        origin = new Pose2d(-64, -36, Math.toRadians(0));
        drive.setPoseEstimate(origin);
        Pose2d highJuncPos = new Pose2d(-3.5, -32, Math.toRadians(60));

        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                })
                .splineToConstantHeading(new Vector2d(-23, -36), Math.toRadians(0))
                .splineToSplineHeading(highJuncPos, Math.toRadians(0))
                .build();

        pos2Trajectory = drive.trajectoryBuilder(highJuncPos)
                .lineToLinearHeading(new Pose2d(-19, -37, Math.toRadians(0)))
                .build();
        pos3Trajectory = drive.trajectoryBuilder(highJuncPos)
                .lineToLinearHeading(new Pose2d(-14, -65.5, Math.toRadians(0)))
                .build();
        pos1Trajectory = drive.trajectoryBuilder(highJuncPos, true)
                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(0)), Math.toRadians(-45))
                .build();

        grabConeStackTrajectorySequence = drive.trajectorySequenceBuilder(highJuncPos)
                //.splineToConstantHeading(new Vector2d(-16, -36), )
                .back(7)
                .lineToLinearHeading(new Pose2d(-13, - 36, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(-13, -63))
                .addDisplacementMarker(() -> {
                    grabConeWhenDetectedAsync();
                })
                .lineToConstantHeading(new Vector2d(-12, -62))
                .build();
        dropConeStackTrajectorySequence = drive.trajectorySequenceBuilder(grabConeStackTrajectorySequence.end())
                .lineToLinearHeading(new Pose2d(-12, -40, Math.toRadians(-60)))
                .splineToLinearHeading(new Pose2d(highJuncPos.getX(), highJuncPos.getY(), highJuncPos.getHeading()), Math.toRadians(0))
                .build();
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
        for (int i = 0; i < 3; i++) {
            getSleevePos();
        }
        telemetry.addData("Trajectory", "Created");
        telemetry.update();
        coneTool.grabCone();
        waitForStart();
        for (int i = 0; i < 3; i++) {
            getSleevePos();
        }
        telemetry.addData("sleeve pos:", pos);
        telemetry.update();
        drive.followTrajectorySequence(dropConeTrajectory);
        coneTool.dropCone();
        for (int i = 0; i < 2; i++) {
            final int index = i;
            AZUtil.runInParallel(new Runnable() {
                @Override
                public void run() {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                    sleep(500);
                    coneTool.liftTo(levels[index]);
                }
            });

            drive.followTrajectorySequence(grabConeStackTrajectorySequence);
            coneTool.grabCone();
            sleep(250);
            AZUtil.runInParallel(new Runnable() {
                @Override
                public void run() {
                    coneTool.liftTo(Lift.LiftLevel.HIGH);
                }
            });

            drive.followTrajectorySequence(dropConeStackTrajectorySequence);
            coneTool.dropCone();
        }
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                coneTool.liftTo(Lift.LiftLevel.HIGH);
                sleep(500);
                //lower fast
                coneTool.liftTo(Lift.LiftLevel.ZERO, Lift.UP_POWER);
            }
        });
        telemetry.addData("sleeve pos:", pos);
        telemetry.update();
        switch (pos) {
            case 1:
                drive.followTrajectory(pos1Trajectory);
                break;
            case 2:
                drive.followTrajectory(pos2Trajectory);
                break;
            case 3:
                drive.followTrajectory(pos3Trajectory);
                break;
            default:
                drive.followTrajectory(pos2Trajectory);
        }

        //TODO: need to accurately determine the time to ensure < 30 secs

        sleep(5000);


    }

    public void grabConeWhenDetectedAsync() {
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                while (!coneTool.isConeDetected()) {
                    Thread.yield();
                }
                coneTool.grabCone();
                telemetry.addLine("Cone Detected");
                telemetry.update();
            }
        });

    }

    private void getSleevePos() {
        pos = sleeveDetection.getPos();
        telemetry.addData("Pos", pos);
        int[] avg = sleeveDetection.getAvgs();
        telemetry.addData("Red: ", avg[0]);
        telemetry.addData("Green: ", avg[1]);
        telemetry.addData("Blue: ", avg[2]);
        telemetry.update();
    }
}
