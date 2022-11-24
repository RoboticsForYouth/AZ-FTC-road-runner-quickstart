package org.firstinspires.ftc.teamcode.powerplay.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;

@Autonomous(name = "LiftAuto")
public class Lift extends LinearOpMode {

    public static final double UP_POWER = 0.95;
    public static final double DOWN_POWER = 0.1;
    public static final int DROP_HEIGHT = 200;
    private DcMotorEx leftSlider;
    private DcMotorEx rightSlider;
    LinearOpMode opMode;
    SampleMecanumDrive drive;

    public void lowerWithoutEncoder() {
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlider.setPower(-1);
        rightSlider.setPower(-1);
    }

    public void stopLoweringWithoutEncoder(){
        leftSlider.setPower(0);
        rightSlider.setPower(0);
    }


    public enum LiftLevel {

        ZERO(0),
        GROUND(0),
        CLEAR(500),
        LOW(3500),
        MEDIUM(6000),
        HIGH(8500);

        private int value;

        LiftLevel(int val) {
            this.value = val;
        }

        public int getValue() {
            return this.value;
        }
    }


    public void setup() {
        leftSlider = opMode.hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = opMode.hardwareMap.get(DcMotorEx.class, "rightSlider");
        setupPos();
    }

    public void setupPos() {
        rightSlider.setDirection(DcMotor.Direction.REVERSE);
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
        AZUtil.setMotorTargetPosition(leftSlider, level.getValue(), UP_POWER);
        AZUtil.setMotorTargetPosition(rightSlider, level.getValue(), UP_POWER);

    }

    public void setTo0() {
        AZUtil.setMotorTargetPosition(leftSlider, LiftLevel.ZERO.getValue(), DOWN_POWER);
        AZUtil.setMotorTargetPosition(rightSlider, LiftLevel.ZERO.getValue(), DOWN_POWER);
    }

    public void lowerToDrop(){
        int leftSliderCurrentPosition = leftSlider.getCurrentPosition();
        int rightSliderCurrentPosition = rightSlider.getCurrentPosition();
        if( leftSliderCurrentPosition > 0) {
            AZUtil.setMotorTargetPosition(leftSlider, leftSliderCurrentPosition - DROP_HEIGHT, DOWN_POWER);
            AZUtil.setMotorTargetPosition(rightSlider, rightSliderCurrentPosition - DROP_HEIGHT, DOWN_POWER);
        }
    }

    public void raiseAfterDrop(){
        int leftSliderCurrentPosition = leftSlider.getCurrentPosition();
        int rightSliderCurrentPosition = rightSlider.getCurrentPosition();
        AZUtil.setMotorTargetPosition(leftSlider, leftSliderCurrentPosition + DROP_HEIGHT, DOWN_POWER);
        AZUtil.setMotorTargetPosition(rightSlider, rightSliderCurrentPosition + DROP_HEIGHT, DOWN_POWER);
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