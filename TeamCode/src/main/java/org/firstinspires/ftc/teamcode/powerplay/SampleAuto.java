package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;

@Autonomous(group = "sample")
public class SampleAuto extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx[] motors = { leftFront, leftRear, rightRear, rightFront };

        for (DcMotorEx motor : motors) {
            AZUtil.resetMotor(this, motor);
        }

//        initAuto();
        /*AZUtil.resetMotor(leftFront);
        AZUtil.resetMotor(rightFront);
        AZUtil.resetMotor(leftRear);
        AZUtil.resetMotor(rightRear);*/

        waitForStart();
        if (isStopRequested()) return;

        AZUtil.setMotorTargetPosition(leftFront, 1000, 1);
        sleep(1000);
        AZUtil.setMotorTargetPosition(rightFront, 1000, 1);
        sleep(1000);
        AZUtil.setMotorTargetPosition(leftRear, 1000, 1);
        sleep(1000);
        AZUtil.setMotorTargetPosition(rightRear, 1000, 1);
        sleep(1000);


//        drive.followTrajectory(park);
//        drive.followTrajectory(park2);
//        drive.followTrajectory(t2);
    }
}
