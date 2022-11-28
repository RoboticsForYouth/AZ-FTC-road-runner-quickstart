package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.pipeline.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;

public class AutoBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ConeTool coneTool;
    SampleMecanumDrive drive;
    PowerPlayPipeline cam;
    int pos;
    Trajectory dropCone, dropCone2, dropCone3, park;

    int originX = 0;
    int originY = 0;
    int originHeading = 0;

    public void initAuto() {
        coneTool = new ConeTool(this);
        cam = new PowerPlayPipeline();
        //cam.setup();
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }
    public void setupPos() {
       coneTool.grabCone();
    }

    public void detection() {
        pos = cam.getAnalysis();
        telemetry.addData("Position", pos);
        telemetry.update();
    }

    public void dropCone(boolean left) {
        if(left) {
            dropCone = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX, originY + 32, originHeading)).build();
            dropCone2 = drive.trajectoryBuilder(dropCone.end())
                    .lineToSplineHeading(new Pose2d(originX + 50, originY + 34, originHeading-45)).build();
        } else {
            dropCone = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX + 30, originY, originHeading+90)).build();
            dropCone2 = drive.trajectoryBuilder(dropCone.end())
                    .lineToSplineHeading(new Pose2d(originX + 42, originY - 24, originHeading+180)).build();
        }
//        AZUtil.runInParallel(
//                new Runnable() {
//                    @Override
//                    public void run() {
//                        sleep(500);
//                        coneTool.liftTo(Lift.LiftLevel.HIGH);
//
//                    }
//                }
//        );
        drive.followTrajectory(dropCone);
        drive.followTrajectory(dropCone2);
        sleep(1000);
        coneTool.dropCone();
//        AZUtil.runInParallel(
//                new Runnable() {
//                    @Override
//                    public void run() {
//                        sleep(1000);
//                        coneTool.liftTo(Lift.LiftLevel.ZERO);
//
//                    }
//                }
//        );

    }

    public void park(int pos, boolean left) {
        if(left) {

            if(pos == 1) {

            } else if(pos == 2) {

            } else {

            }

        } else {

            if(pos == 1) {

            } else if(pos == 2) {

            } else {

            }

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
