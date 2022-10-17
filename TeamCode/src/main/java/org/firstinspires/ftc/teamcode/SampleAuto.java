package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.auto.BlueAutoBase;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;

@Config
@Autonomous(group = "sample")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws  InterruptedException{

        DcMotorEx leftFront;
        DcMotorEx leftRear;
        DcMotorEx rightRear;
        DcMotorEx rightFront;

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);



//        initAuto();
        AZUtil.resetMotor(leftFront);
        AZUtil.resetMotor(rightFront);
        AZUtil.resetMotor(leftRear);
        AZUtil.resetMotor(rightRear);

        waitForStart();
        if (isStopRequested()) return;

        AZUtil.setMotorTargetPostion(leftFront, 1000, 1);
        sleep(5000);
  AZUtil.setMotorTargetPostion(rightFront, 1000, 1);
        sleep(5000);
  AZUtil.setMotorTargetPostion(leftRear, 1000, 1);
        sleep(5000);
  AZUtil.setMotorTargetPostion(rightRear, 1000, 1);
        sleep(5000);



//        drive.followTrajectory(park);
//        drive.followTrajectory(park2);
//        drive.followTrajectory(t2);
    }
}
