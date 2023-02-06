package org.firstinspires.ftc.teamcode.powerplay.tools;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.aztools.AZUtil;

//@Autonomous(name = "LiftAuto")
@TeleOp
public class Lift extends LinearOpMode {

    public static final double UP_POWER = .95;
    public static final double DOWN_POWER = .6;
    public static int DROP_HEIGHT = 270;
    public static final int INC = 25;
    private DcMotorEx leftSlider;
    private DcMotorEx rightSlider;
    LinearOpMode opMode;
    SampleMecanumDrive drive;
    private LiftLevel currentState = LiftLevel.ZERO;

    public void lowerWithoutEncoder(double power) {
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlider.setPower(-power);
        rightSlider.setPower(-power);
    }

    public void raiseWithoutEncoder(double power) {
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlider.setPower(1);
        rightSlider.setPower(1);
    }

    public void stopLift(){
        leftSlider.setPower(0);
        rightSlider.setPower(0);
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum LiftLevel {

        //Cone Stack Levels
        CONE_5(420),
        CONE_4(312),
        CONE_3(240),
        CONE_2(110),

        ZERO(0),
        GROUND(0),
        SECOND_CONE(300),
        CLEAR(100),
        FOURTH_CONE(800),
        LOW(1250),
        MEDIUM(1950),
        HIGH(2630);

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
        double power = UP_POWER;
        Pair<Integer, Integer> currentPosition = getCurrentPosition();
        //lower speed while sliding down
        if( currentState.getValue() > level.getValue()){
            power = DOWN_POWER;
        }
        liftTo(level, power);
    }

    public void liftTo(LiftLevel level, double power) {
        AZUtil.setMotorTargetPosition(leftSlider, level.getValue(), power);
        AZUtil.setMotorTargetPosition(rightSlider, level.getValue(), power);
        currentState = level;
    }

    public LiftLevel getCurrentState() {
        return currentState;
    }

    public void setTo0() {
        AZUtil.setMotorTargetPosition(leftSlider, LiftLevel.ZERO.getValue(), DOWN_POWER);
        AZUtil.setMotorTargetPosition(rightSlider, LiftLevel.ZERO.getValue(), DOWN_POWER);
    }

    public void lowerToDrop(){
        int leftSliderCurrentPosition = leftSlider.getCurrentPosition();
        int rightSliderCurrentPosition = rightSlider.getCurrentPosition();
        if( leftSliderCurrentPosition > DROP_HEIGHT) {
            AZUtil.setMotorTargetPosition(leftSlider, leftSliderCurrentPosition - DROP_HEIGHT, UP_POWER);
            AZUtil.setMotorTargetPosition(rightSlider, rightSliderCurrentPosition - DROP_HEIGHT, UP_POWER);
        }
    }

    //Left - first, Right - Second
    public Pair<Integer, Integer> getCurrentPosition(){
        return new Pair<>(leftSlider.getCurrentPosition(), rightSlider.getCurrentPosition());
    }

    @Override
    public void runOpMode() {
        testLiftPosition();
    }

    private void testLiftPosition() {
        testTeleOp();
    }

    private void testTeleOp() {
        setup();
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry telemetry;
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        waitForStart();
        int count = 0;
        while(opModeIsActive()) {
            if( gamepad1.right_trigger > 0){
                AZUtil.setMotorTargetPosition(leftSlider, leftSlider.getCurrentPosition()+ INC, UP_POWER);
                AZUtil.setMotorTargetPosition(leftSlider, rightSlider.getCurrentPosition()+ INC, UP_POWER);
            } else if ( gamepad1.left_trigger > 0 &&
                    //prevent from going below zero
                    ( getCurrentPosition().first > 0 && getCurrentPosition().second > 0)){
                AZUtil.setMotorTargetPosition(leftSlider, leftSlider.getCurrentPosition()- INC, DOWN_POWER);
                AZUtil.setMotorTargetPosition(leftSlider, rightSlider.getCurrentPosition()- INC, DOWN_POWER);
            }

            if( gamepad1.dpad_up){
                liftTo(LiftLevel.MEDIUM);
            }
            else if ( gamepad1.dpad_down ){
                liftTo(LiftLevel.ZERO);
            } else if (gamepad1.dpad_left){
                liftTo(LiftLevel.LOW);
            } else if (gamepad1.dpad_right){
                liftTo(LiftLevel.HIGH);
            }
//            telemetry.addLine("Pos");
//            telemetry.addData("Gamepad", gamepad1);
            telemetry.addData("Left Pos", leftSlider.getCurrentPosition());
            telemetry.addData("Right Slider Pos", rightSlider.getCurrentPosition());
            telemetry.addData("Expected Pos", getCurrentState().getValue());
            telemetry.update();
        }
    }

    public void setDropHeight(int height) {
        DROP_HEIGHT = height;
    }
}