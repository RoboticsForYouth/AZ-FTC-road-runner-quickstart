package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Arm;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Carousel;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightTool;
import org.firstinspires.ftc.teamcode.pipeline.FFDetection;

public abstract class RedAutoBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive;
    FFDetection cam;
    FreightTool freightTool;
    Carousel duckyTool;
    LinearOpMode opMode;
    Arm.ArmLevel pos;
    Trajectory drop, drop2, drop3, duck, duck2, duck3, park, park2, park3, move, back;
    boolean carousel = false;
    int originX = 0;
    int originY = 0;
    int originHeading = 0;
    int dropX, dropY, dropHeading, alignX, alignY, duckX, duckY, parkX, parkY;


    public void initAuto() {
        freightTool = new FreightTool(this);
        duckyTool = new Carousel(this);
        cam = new FFDetection(this);
        cam.setup();
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    public void setupPos() {
        freightTool.setupPos();
    }

    public void detection() {
        pos = Arm.ArmLevel.LEVEL2;
        telemetry.addData("Position", pos);
        telemetry.update();
    }

    public void moveToDrop(String side) {
        if (side.equals("left")) { //LEFT
//            drop = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
//                    .lineToSplineHeading(new Pose2d(originX + 22, originY + 12, originHeading)).build();

            if (pos == Arm.ArmLevel.LEVEL3) {
                drop2 = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                        .lineToSplineHeading(new Pose2d(originX + 25, originY + 22, originHeading)).build();
            } else if (pos == Arm.ArmLevel.LEVEL1) {
                drop2 = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                        .lineToSplineHeading(new Pose2d(originX + 24, originY + 16, originHeading)).build();
            } else {
                drop2 = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                        .lineToSplineHeading(new Pose2d(originX + 24, originY + 17, originHeading)).build();
            }
            drop3 = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX + 23, originY + 10, originHeading)).build();


        } else { //RIGHT
//            drop = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
//                    .lineToSplineHeading(new Pose2d(originX - 24, originY + 12, originHeading)).build();

            if (pos == Arm.ArmLevel.LEVEL3) {
                dropX = -14;
                dropY = 22;
                dropHeading = 135;
            } else if( pos == Arm.ArmLevel.LEVEL2) {
                dropX = -8;
                dropY = 24;
                dropHeading = 135;
            } else {
                dropX = -5;
                dropY = 36;
                dropHeading = 180;
                drop2 = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                        .lineToSplineHeading(new Pose2d(originX, originY, originHeading))
                        .lineToSplineHeading(new Pose2d())
                        .build();
                //drive.followTrajectory(drop2);

            }
            drop3 = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX + dropX, originY + dropY, originHeading + Math.toRadians(dropHeading))).build();


        }

        AZUtil.runInParallel(
                new Runnable() {
                    @Override
                    public void run() {
                        freightTool.prepForDrop(pos); //insert pos HERE!!
                        sleep(1000);
                    }
                }
        );
        drive.followTrajectory(drop3);
       //drive.followTrajectory(drop);
        //drive.followTrajectory(drop2);
        sleep(1000);
        freightTool.drop();
        AZUtil.runInParallel(
                new Runnable() {
                    @Override
                    public void run() {
                        sleep(1000);
                        freightTool.reset();
                    }
                }
        );

    }

    public void moveToCarousel(String side) {
        carousel = true;
        if (side.equals("right")) { //RIGHT
            duck = drive.trajectoryBuilder(drop3.end())
                    .lineToSplineHeading(new Pose2d(originX - 76, originY + 15, originHeading)).build();
            duck2 = drive.trajectoryBuilder(duck.end())
                    .lineToSplineHeading(new Pose2d(originX - 68, originY + 15, originHeading)).build();
            duck3 = drive.trajectoryBuilder(duck2.end())
                    .lineToSplineHeading(new Pose2d(originX - 68, originY + 5, originHeading)).build();
        } else { //LEFT
            duck = drive.trajectoryBuilder(drop3.end())
                    .lineToSplineHeading(new Pose2d(originX - 29, originY + 15, originHeading)).build();
            duck2 = drive.trajectoryBuilder(duck.end())
                    .lineToSplineHeading(new Pose2d(originX - 23, originY + 15, originHeading)).build();
            duck3 = drive.trajectoryBuilder(duck2.end())
                    .lineToSplineHeading(new Pose2d(originX - 23, originY + 5, originHeading)).build();
        }
        drive.followTrajectory(duck);
        drive.followTrajectory(duck2);
        drive.followTrajectory(duck3);
        duckyTool.redDuckyDrop();
    }

    public void moveToPark(String side, String where) {
        if (side.equals("right")) {
            if (where.equals("warehouse")) {
                if (carousel) { //start on right - carousel - park in warehouse
                    //park = drive.trajectoryBuilder(duck3.end())
                    // .lineToSplineHeading(new Pose2d(originX-24, originY+10, originHeading)).build();
                    park2 = drive.trajectoryBuilder(duck3.end())
                            .lineToSplineHeading(new Pose2d(originX + 24, originY - 5, originHeading)).build();
                    park3 = drive.trajectoryBuilder(park2.end())
                            .lineToSplineHeading(new Pose2d(originX + 30, originY + 20, originHeading)).build();
                    //drive.followTrajectory(park);

                } else { //start on right - park in warehouse
                    park = drive.trajectoryBuilder(drop3.end())
                            .lineToSplineHeading(new Pose2d(originX - 10, originY + 2, originHeading)).build();
                    park2 = drive.trajectoryBuilder(park.end())
                            .lineToSplineHeading(new Pose2d(originX + 24, originY - 2, originHeading)).build();
                    park3 = drive.trajectoryBuilder(park2.end())
                            .lineToSplineHeading(new Pose2d(originX + 36, originY + 20, originHeading)).build();
                    drive.followTrajectory(park);
                }
                drive.followTrajectory(park2);
                drive.followTrajectory(park3);
            } else { //start on right - park in storage area
                park = drive.trajectoryBuilder(duck3.end())
                        .lineToSplineHeading(new Pose2d(originX + 68, originY + 23, Math.toRadians(90))).build();
                drive.followTrajectory(park);
            }
            freightTool.moveTo0();
        } else { //start on left
            if (where.equals("warehouse")) {
                if (carousel) { //start on left - carousel - park in warehouse
                    park = drive.trajectoryBuilder(duck3.end())
                            .lineToSplineHeading(new Pose2d(originX + 72, originY - 5, originHeading)).build();
                    park2 = drive.trajectoryBuilder(park.end())
                            .lineToSplineHeading(new Pose2d(originX + 72, originY + 24, originHeading)).build();

                } else { //start on left - no carousel - park in warehouse
                    park = drive.trajectoryBuilder(drop3.end())
                            .lineToSplineHeading(new Pose2d(originX + 72, originY, originHeading)).build();
                    park2 = drive.trajectoryBuilder(park.end())
                            .lineToSplineHeading(new Pose2d(originX + 72, originY + 24, originHeading)).build();

                }
                drive.followTrajectory(park);
                drive.followTrajectory(park2);
            } else {
                if (carousel) { //start on left - carousel - park in storage
                    park = drive.trajectoryBuilder(duck3.end())
                            .lineToSplineHeading(new Pose2d(originX - 27, originY + 23, originHeading)).build();
                } else { //start on left - no carousel - park in storage
                    park = drive.trajectoryBuilder(drop3.end())
                            .lineToSplineHeading(new Pose2d(originX - 27, originY + 23, originHeading)).build();
                }
                drive.followTrajectory(park);

            }
            freightTool.moveTo0();
        }
    }

}
