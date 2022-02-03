package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "sample")
public class Test extends BlueAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) return;
        setupPos();
        //moveToDrop("warehouse");
        freightTool.arm.moveToLevel(0);
        sleep(2000);
        freightTool.arm.moveToLevel(1);
        sleep(2000);
        freightTool.arm.moveToLevel(2);
        sleep(2000);
        freightTool.arm.moveToLevel(3);
        sleep(2000);
        freightTool.arm.moveToLevel(4);
        sleep(2000);




    }
}

