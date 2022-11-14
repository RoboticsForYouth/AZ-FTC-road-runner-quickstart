package org.firstinspires.ftc.teamcode.powerplay.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.auto.AutoUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.auto.RWareHouseNEW;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Arm;

@Autonomous(name = "LiftAuto")
public class Lift extends LinearOpMode {

    private DcMotorEx leftSlider;
    private DcMotorEx rightSlider;
    LinearOpMode opMode;
    SampleMecanumDrive drive;
    public enum LiftLevel {

        ZERO(0),
        GROUND(0),
        LOW(0),
        MEDIUM(0),
        HIGH(700);

        private int value;

        LiftLevel(int val) {
            this.value = val;
        }

        public int getValue() {
            return this.value;
        }
    }


    public void setup() {
        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");
        setupPos();
    }

    public void setupPos() {
        leftSlider.setDirection(DcMotor.Direction.REVERSE);
        AZUtil.resetMotor(this, leftSlider);
        AZUtil.resetMotor(this, rightSlider);
    }

    public Lift() {
        opMode = this;
    }

    public Lift(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }

    public void liftTo(LiftLevel level) {
        AZUtil.setMotorTargetPosition(leftSlider, level.getValue(), 0.25);
        AZUtil.setMotorTargetPosition(rightSlider, level.getValue(), 0.25);

    }

    public void setTo0() {
        AZUtil.setMotorTargetPosition(leftSlider, LiftLevel.ZERO.getValue(), 0.1);
        AZUtil.setMotorTargetPosition(rightSlider, LiftLevel.ZERO.getValue(), 0.1);
    }


    @Override
    public void runOpMode() {
        setup();
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry telemetry;
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        waitForStart();
        int count = 0;
        while(opModeIsActive() & count < 1) {
            liftTo(LiftLevel.HIGH);
            sleep(5000);

            telemetry.addLine("Top pos");
            telemetry.addData("Left Slider Pos", leftSlider.getCurrentPosition());
            telemetry.addData("Right Slider Pos", rightSlider.getCurrentPosition());
            telemetry.update();
            setTo0();
            sleep(5000);
            telemetry.addLine("Bottom pos");
            telemetry.addData("Left Slider Pos", leftSlider.getCurrentPosition());
            telemetry.addData("Right Slider Pos", rightSlider.getCurrentPosition());
            telemetry.update();
            sleep(15000);
            count++;
        }



    }

}