package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Autonomous(group = "sample")

public class RRTest extends LinearOpMode {
    Trajectory dropcone, park1, park2, park3;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // auto path
        //grip cone
        //detect
        //lift arm in parallel
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
         dropcone = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(70, 0), Math.toRadians(0))
                .build();
         park1 = drive.trajectoryBuilder(dropcone.end())
                 .splineTo(new Vector2d(-40, -20), Math.toRadians(-90))
                 .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(dropcone);
        drive.followTrajectory(park1);
     }
}
