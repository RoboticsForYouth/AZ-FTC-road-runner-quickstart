package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.BlueAutoBase;
import org.firstinspires.ftc.teamcode.freightFrenzy.Detection;

@Config
@Autonomous(group = "sample")
public class SampleAuto extends BlueAutoBase {
    @Override
    public void runOpMode() throws  InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int originX=0;
        int originY=0;
        int originHeading=0;

        initAuto();
        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(originX,originY,Math.toRadians(originHeading)))
                .strafeRight(35).build();
        Trajectory drop = drive.trajectoryBuilder(strafe.end())
                .forward(20).build();
        Trajectory duck = drive.trajectoryBuilder(drop.end())
                .lineToSplineHeading(new Pose2d(originX+10, originY+39, Math.toRadians(-150))).build();
        Trajectory park = drive.trajectoryBuilder(duck.end())
                .lineToSplineHeading(new Pose2d(originX+56, originY+36, Math.toRadians(-100))).build();

        drive.followTrajectory(strafe);
        drive.followTrajectory(drop);
        drive.followTrajectory(duck);
        drive.followTrajectory(park);


        waitForStart();
        if (isStopRequested()) return;


//        drive.followTrajectory(park);
//        drive.followTrajectory(park2);
//        drive.followTrajectory(t2);
    }
}
