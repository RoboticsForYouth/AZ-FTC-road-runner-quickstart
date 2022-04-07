package org.firstinspires.ftc.teamcode.freightFrenzy.teleOp;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Carousel;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightInIntakeSensor;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.TurnTable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


public class FFTeleOpStates extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backL;
    private DcMotor frontL;
    private DcMotor backR;
    private DcMotor frontR;
    Carousel duckyTool;
    FreightTool freightTool;
    FreightInIntakeSensor intakeSensor;
    LinearOpMode opMode;
    private boolean isDuckyToolStopped;
    private boolean isIntakeOn;
    private boolean warehouseToHub = false;
    SampleMecanumDrive drive;
    TrajectorySequence allianceToWarehouseTrajectorySequence;
    boolean isSharedHubAubAuto = false;
    boolean sharedHubComplete;

    private enum GamePad2Mode {
        NONE, TAPE_DRIVE, RESET_ARM, RESET_INTAKE
    }

    GamePad2Mode gamepad2Mode = GamePad2Mode.NONE;

    boolean duckyToolClockwise = true;

    public void setup() {
        freightTool = new FreightTool(this);

        duckyTool = new Carousel(this);
//        tapeDrive = new TapeDrive(this, freightTool);
    }

    public double motor_fast_speed = 1;
    public double motor_slow_speed = 0.5;
    boolean fast = true;

    boolean intakeOn = false;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

        setup();
        freightTool.setupPos();
        waitForStart();
        runtime.reset();
        freightTool.move();

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .5,
                            -gamepad1.left_stick_x * .5,
                            -gamepad1.right_stick_x * .5
                    )
            );

            drive.update();

            orgLoop();
            Pose2d myPose = drive.getPoseEstimate();

            AZUtil.print(telemetry, Arrays.asList(
                    new Pair<>("X", myPose.getX()),
                    new Pair<>("Y", myPose.getY()),
                    new Pair<>("Heading", myPose.getHeading()),
                    new Pair<>("gamepad", gamepad1)
            ));
        }
    }

    private void orgLoop() {
        if (gamepad1.right_bumper && !freightTool.isFreightToolBusy()) {
            AZUtil.runInParallel(() -> {
                freightTool.intakeTeleOp();
            });
        }
        if (gamepad1.left_bumper && !freightTool.isFreightToolBusy()) {
            autoSharedHubDrop();
        }
        if (gamepad1.a && !intakeOn) { //drop freight
            freightTool.dropFreightTeleOp();
            isIntakeOn = true;
        } else if (isIntakeOn) {
            freightTool.stopIntake();
            isIntakeOn = false;
        }

        if (gamepad1.x) { //move pos
            AZUtil.runInParallel(() -> {
                freightTool.move();
            });
        }

        if (gamepad1.y) {
            if (!warehouseToHub) {
                warehouseToHub = true;
                moveToAllianceHub();
            } else {
                warehouseToHub = false;
                moveToWarehouse();
            }
        }

        if (gamepad1.dpad_up && !freightTool.isFreightToolBusy()) { //move to alliance hub pos
            AZUtil.runInParallel(() -> {
                freightTool.allianceHub();
            });
        }

        if (gamepad1.dpad_down && !freightTool.isFreightToolBusy()) { //move to shared hub pos
            AZUtil.runInParallel(() -> {
                freightTool.sharedHub();
            });
        }

        if (gamepad1.dpad_left) {
            freightTool.turnInc(TurnTable.Direction.COUNTER_CLOCKWISE);
        }
        if (gamepad1.dpad_right) {
            freightTool.turnInc(TurnTable.Direction.CLOCKWISE);
        }

        if (gamepad1.left_stick_button && !freightTool.isFreightToolBusy()) {
            autoSharedHubOverBarrierAuto();
        }

        if (gamepad1.right_trigger > 0.1) {

            duckyTool.blueDuckyDrop(gamepad1.right_trigger);
            isDuckyToolStopped = false;
        } else if (gamepad1.left_trigger > 0.1) {
            duckyTool.redDuckyDrop(gamepad1.left_trigger);
            isDuckyToolStopped = false;
        } else {
            if (!isDuckyToolStopped) {
                duckyTool.stopDuckyTool();
                isDuckyToolStopped = true;
            }
        }

        if(gamepad2.dpad_up){
            freightTool.moveUp();
        }
        if(gamepad2.dpad_down){
            freightTool.moveDown();
        }

        if(gamepad2.dpad_left){
            freightTool.moveLeft();
        }
        if(gamepad2.dpad_right){
            freightTool.moveRight();
        }
    }

    void autoSharedHubOverBarrierAuto() {
        //Set initial to zero
        if (!isSharedHubAubAuto)
            isSharedHubAubAuto = true;
        AZUtil.runInParallel(() -> {
            freightTool.turn180ToSharedHub();
            sharedHubComplete = true;
        });
    }


     void autoSharedHubDrop() {

        AZUtil.runInParallel(() -> {
            freightTool.prepRedSharedHubDrop();
        });
    }

     void moveToAllianceHub() {
        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(0, 10, Math.toRadians(0)));
        AZUtil.runInParallel(() -> {
            freightTool.allianceHub();
        });
        Pose2d poseEstimate = drive.getPoseEstimate();
        Trajectory trajectory = getMoveToAllianceHubTrajectory(poseEstimate);

        drive.followTrajectory(trajectory);
    }

    @NonNull
    Trajectory getMoveToAllianceHubTrajectory(Pose2d poseEstimate) {
        Trajectory trajectory = drive.trajectoryBuilder(poseEstimate, true)
                .splineToSplineHeading(new Pose2d(poseEstimate.getX() - 30, poseEstimate.getY() + 21,
                                Math.toRadians(poseEstimate.getHeading() + 90)), Math.toRadians(30),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();
        return trajectory;
    }

     void moveToWarehouse() {
        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        Pose2d poseEstimate = drive.getPoseEstimate();
        allianceToWarehouseTrajectorySequence = getAllianceToWarehouseTrajectorySequence(poseEstimate);
        drive.followTrajectorySequence(allianceToWarehouseTrajectorySequence);
        AZUtil.runInParallel(() -> {
            freightTool.intake();
        });
    }

     TrajectorySequence getAllianceToWarehouseTrajectorySequence(Pose2d poseEstimate) {
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(poseEstimate)
                .lineToSplineHeading(new Pose2d(0, 0,
                                Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToSplineHeading(new Pose2d(24,0, Math.toRadians(0)))
//                .forward(24)
                .addTemporalMarker(1, () -> {
                    AZUtil.runInParallel(() -> {
                        freightTool.intake();
                    });
                })
                .build();
        return trajectory;
    }

}
