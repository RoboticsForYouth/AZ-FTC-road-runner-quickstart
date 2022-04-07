package org.firstinspires.ftc.teamcode.freightFrenzy.teleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class BlueFFTeleOpStates extends FFTeleOpStates {

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
            freightTool.prepBlueSharedHubDrop();
        });
    }

    void moveToAllianceHub() {
        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
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
                .lineToSplineHeading(new Pose2d(poseEstimate.getX() - 30, poseEstimate.getY() - 21,
                                Math.toRadians(poseEstimate.getHeading() - 90)),
//                        Math.toRadians(30),
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
    }

    TrajectorySequence getAllianceToWarehouseTrajectorySequence(Pose2d poseEstimate) {
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(poseEstimate)
                .lineToSplineHeading(new Pose2d(0, 2,
                                Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(20)
                .addTemporalMarker(2, () -> {
                    AZUtil.runInParallel(() -> {
                        freightTool.intakeTeleOp();
                    });
                })
                .build();
        return trajectory;
    }
}
