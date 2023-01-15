package org.firstinspires.ftc.teamcode.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.aztools.AZUtil;

//@Autonomous
public class SliderTestAuto extends LinearOpMode {

    private DcMotorEx leftSlider;
    private DcMotorEx rightSlider;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = AZUtil.getMultiTelemetry(this);
        telemetry.addLine("Initialized");
        telemetry.update();

        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");

        rightSlider.setDirection(DcMotor.Direction.REVERSE);
        AZUtil.resetMotor(this, leftSlider);
        AZUtil.resetMotor(this, rightSlider);

        waitForStart();
/**
        leftSlider.setPower(1);
        sleep(5000);
        leftSlider.setPower(0);
        rightSlider.setPower(1);
        sleep(5000);
        rightSlider.setPower(0);
        sleep(5000);
        */
/**

        while(opModeIsActive()){
            telemetry.addData("Left Slider Pos", leftSlider.getCurrentPosition());
            telemetry.addData("Right Slider Pos", rightSlider.getCurrentPosition());
            telemetry.update();
        }
 */

        AZUtil.setMotorTargetPosition(leftSlider, 700, 0.25);
        AZUtil.setMotorTargetPosition(rightSlider, 700, 0.25);
        sleep(5000);

        telemetry.addLine("Top pos");
        telemetry.addData("Left Slider Pos", leftSlider.getCurrentPosition());
        telemetry.addData("Right Slider Pos", rightSlider.getCurrentPosition());
        telemetry.update();
        AZUtil.setMotorTargetPosition(leftSlider, 0, 0.1);
        AZUtil.setMotorTargetPosition(rightSlider, 0, 0.1);
        sleep(5000);
        telemetry.addLine("Bottom pos");
        telemetry.addData("Left Slider Pos", leftSlider.getCurrentPosition());
        telemetry.addData("Right Slider Pos", rightSlider.getCurrentPosition());
        telemetry.update();
        sleep(15000);

    }
}
