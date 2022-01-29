package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.RedAutoBase;
import org.firstinspires.ftc.teamcode.freightFrenzy.Detection;

@Autonomous(group = "sample")

public class RRightWarehouse extends RedAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        detection();
        sleep(4000);
        moveToDrop("right");
        moveToPark("right", "warehouse");

    }
}
