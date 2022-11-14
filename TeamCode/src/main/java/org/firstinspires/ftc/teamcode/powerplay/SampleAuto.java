package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;

@Autonomous(group = "sample")
public class SampleAuto extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {


        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

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

        AZUtil.setMotorTargetPosition(leftFront, 5000, 1);
        sleep(5000);
        AZUtil.setMotorTargetPosition(rightFront, 5000, 1);
        sleep(5000);
        AZUtil.setMotorTargetPosition(leftRear, 5000, 1);
        sleep(5000);
        AZUtil.setMotorTargetPosition(rightRear, 5000, 1);
        sleep(5000);


//        drive.followTrajectory(park);
//        drive.followTrajectory(park2);
//        drive.followTrajectory(t2);
    }
}
