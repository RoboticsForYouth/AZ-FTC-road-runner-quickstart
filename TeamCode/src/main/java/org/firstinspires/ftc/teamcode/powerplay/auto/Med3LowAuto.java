package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.aztools.AZUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

//@Disabled
@Autonomous
public class Med3LowAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ConeTool coneTool;
    SampleMecanumDrive drive;
    SleeveDetection sleeveDetection;
    int slowVelocity = 35;
    int slowAcc = 33;
    int pos;
    Trajectory dropCone, dropCone2, dropCone3, park;
    private TrajectorySequence[] grabTrajectorySequenceList = new TrajectorySequence[5];
    private TrajectorySequence[] dropTrajectorySequenceList = new TrajectorySequence[5];
    Pose2d origin;
    private Vector2d backUp;
    private TrajectorySequence pos2Trajectory;
    private TrajectorySequence pos1Trajectory;
    private TrajectorySequence pos3Trajectory;
    private TrajectorySequence grabConeStackTrajectorySequence, goToConeStack;
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
        coneTool.setConeThreshold();
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
        Pose2d mediumJuncPos = new Pose2d(-28, 30.5, Math.toRadians(300));

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(slowVelocity, slowAcc, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(slowAcc);
        TrajectoryVelocityConstraint parkingVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(47, 44, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint parkingAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(44);
        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.MEDIUM);
                })
                .splineToConstantHeading(new Vector2d(-44.25, 36), Math.toRadians(0), parkingVelocityConstraint, parkingAccelerationConstraint)
                .splineToSplineHeading(mediumJuncPos, Math.toRadians(0), velocityConstraint, accelerationConstraint)
                .build();
        goToConeStack = drive.trajectorySequenceBuilder(dropConeTrajectory.end())
                .back(8)
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.CONE_5);
                })
                .lineToLinearHeading(new Pose2d(-12.8, 38, Math.toRadians(0)), velocityConstraint, accelerationConstraint)
                .lineToLinearHeading(new Pose2d(-12.8, 42, Math.toRadians(90)), velocityConstraint, accelerationConstraint)
                .lineToConstantHeading(new Vector2d(-13.2, 62.5), velocityConstraint, accelerationConstraint)
                .build();

        dropConeStackTrajectorySequence = drive.trajectorySequenceBuilder(goToConeStack.end())
                .back(7.5)
                .lineToLinearHeading(new Pose2d(-12.5, 54.25, Math.toRadians(205)), velocityConstraint, accelerationConstraint)
                .forward(5.5)
                .build();

        grabConeStackTrajectorySequence = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(-12.8, 55, Math.toRadians(90)), velocityConstraint, accelerationConstraint)
                //.splineToLinearHeading(new Pose2d(-16, 55, Math.toRadians(90)), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
//                    grabConeWhenDetectedAsync();
//                })
                .lineToConstantHeading(new Vector2d(-12.8, 62), velocityConstraint, accelerationConstraint)
                .build();

        pos1Trajectory = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(7)
                .lineToLinearHeading(new Pose2d(-15, 63, Math.toRadians(180)), parkingVelocityConstraint, parkingAccelerationConstraint)
                .build();
        pos2Trajectory = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(5)
                .turn(Math.toRadians(-115))
                .lineToConstantHeading(new Vector2d(-14, 34), parkingVelocityConstraint, parkingAccelerationConstraint)
                .build();
        pos3Trajectory = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(5)
                .turn(Math.toRadians(65))
                .lineToConstantHeading(new Vector2d(-10, 12), parkingVelocityConstraint, parkingAccelerationConstraint)
                //.forward(4)
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
        Pose2d mediumJuncPos = new Pose2d(-27.5, -33, Math.toRadians(60));

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(slowVelocity, slowAcc, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(slowAcc);
        TrajectoryVelocityConstraint parkingVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(47, 44, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint parkingAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(44);
        dropConeTrajectory = drive.trajectorySequenceBuilder(origin)
                //move to high junction
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.MEDIUM);
                })
                .splineToConstantHeading(new Vector2d(-44.5, -37.5), Math.toRadians(0), velocityConstraint, accelerationConstraint)
                .splineToSplineHeading(mediumJuncPos, Math.toRadians(0), velocityConstraint, accelerationConstraint)
                .build();
        goToConeStack = drive.trajectorySequenceBuilder(dropConeTrajectory.end())
                .back(8)
                .addDisplacementMarker(() -> {
                    coneTool.liftTo(Lift.LiftLevel.CONE_5);
                })
                .lineToLinearHeading(new Pose2d(-12.2, -39.5, Math.toRadians(0)), velocityConstraint, accelerationConstraint)
                .lineToLinearHeading(new Pose2d(-12.8, -43.5, Math.toRadians(-90)), velocityConstraint, accelerationConstraint)
                .lineToConstantHeading(new Vector2d(-12.8, -63.5), velocityConstraint, accelerationConstraint)
                .build();

        dropConeStackTrajectorySequence = drive.trajectorySequenceBuilder(goToConeStack.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(-12.8, -56.5, Math.toRadians(-214)), velocityConstraint, accelerationConstraint)
                .forward(5)
                .build();

        grabConeStackTrajectorySequence = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(-12.5, -56.5, Math.toRadians(-90)), velocityConstraint, accelerationConstraint)
                //.splineToLinearHeading(new Pose2d(-16, 55, Math.toRadians(90)), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
//                    grabConeWhenDetectedAsync();
//                })
                .lineToConstantHeading(new Vector2d(-12.5, -63.5), velocityConstraint, accelerationConstraint)
                .build();

        pos3Trajectory = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(7)
                .lineToLinearHeading(new Pose2d(-12.5, -66, Math.toRadians(180)), parkingVelocityConstraint, parkingAccelerationConstraint)
                .build();
        pos2Trajectory = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(7)
                .turn(Math.toRadians(-65))
                .lineToConstantHeading(new Vector2d(-14, -39.5), parkingVelocityConstraint, parkingAccelerationConstraint)
                //.forward(4)
                .build();
        pos1Trajectory = drive.trajectorySequenceBuilder(dropConeStackTrajectorySequence.end())
                .back(5)
                .turn(Math.toRadians(-65))
                .lineToConstantHeading(new Vector2d(-12, -14), parkingVelocityConstraint, parkingAccelerationConstraint)
                //.forward(4)
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
        //telemetry.addData("Trajectory", "Created");
        //telemetry.update();
        coneTool.grabCone();
        waitForStart();
        for (int i = 0; i < 3; i++) {
            getSleevePos();
        }
        telemetry.addData("sleeve pos:", pos);
        telemetry.update();
        //drop to medium
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
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                coneTool.liftTo(Lift.LiftLevel.LOW);
                sleep(500);
                //lower fast
                coneTool.liftTo(Lift.LiftLevel.ZERO, Lift.UP_POWER);
            }
        });
        telemetry.addData("sleeve pos:", pos);
        telemetry.update();
        switch (pos) {
            case 1:
                drive.followTrajectorySequence(pos1Trajectory);
                break;
            case 2:
                drive.followTrajectorySequence(pos2Trajectory);
                break;
            case 3:
                drive.followTrajectorySequence(pos3Trajectory);
                break;
            default:
                drive.followTrajectorySequence(pos1Trajectory);
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
