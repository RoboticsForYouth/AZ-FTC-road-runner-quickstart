package org.firstinspires.ftc.teamcode.sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class SauravTestAutoBase {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotorEx lFront, rFront, lBack, rBack;
    Servo arm, grip;
    LinearOpMode linearOpMode;
    double armPos, gripPos;
    double MIN_POS = 0, MAX_POS = 1;
    MultipleTelemetry telemetry;

    public SauravTestAutoBase(LinearOpMode opMode) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        this.linearOpMode = opMode;

        lFront = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        rFront = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        lBack = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rBack = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);

        arm = opMode.hardwareMap.servo.get("wobblearm");
        grip = opMode.hardwareMap.servo.get("wobblegrip");

        this.runtime.reset();
    }

    private void setupMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void init() {
        setupMotor(lFront);
        setupMotor(rFront);
        setupMotor(lBack);
        setupMotor(rBack);
        stop();
    }

    public void stop() {
        lFront.setPower(0.0);
        lBack.setPower(0.0);
        rFront.setPower(0.0);
        rBack.setPower(0.0);
    }

    public void setPowers(double power) {
        lFront.setPower(power);
        rFront.setPower(power);
        lBack.setPower(power);
        rBack.setPower(power);
    }

    public void setArm(double a) {
        armPos = a;
        arm.setPosition(Range.clip(armPos, MIN_POS, MAX_POS));
    }

    public void setGrip(double a) {
        gripPos = a;
        grip.setPosition(Range.clip(gripPos, MIN_POS, MAX_POS));
    }

    public void setMotorForRunToPos(DcMotor motor, double power, int pos){
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition() + pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void moveForwardEncoder(int pos, double power) {
        stop();
        double defaultPower = 0.5;
        setMotorForRunToPos(lFront, defaultPower * .85, pos);
        setMotorForRunToPos(rFront, defaultPower, pos);
        setMotorForRunToPos(lBack, defaultPower * .85, pos);
        setMotorForRunToPos(rBack, defaultPower, pos);

        waitUntilMotorNotBusy();

        stop();
    }

    public void turnRight90() {
        double defaultPower = 0.5;
        int pos = 667;

        setMotorForRunToPos(lFront, defaultPower, -pos);
        setMotorForRunToPos(lFront, defaultPower, pos);
        setMotorForRunToPos(lFront, defaultPower, -pos);
        setMotorForRunToPos(lFront, defaultPower, pos);

        waitUntilMotorNotBusy();

        stop();
    }

    private void waitUntilMotorNotBusy() {
        while (linearOpMode.opModeIsActive() && (lFront.isBusy() || rFront.isBusy() || lBack.isBusy() || rBack.isBusy())) {
            telemetry.addData("lFront", lFront.getCurrentPosition());
            telemetry.addData("rFront", rFront.getCurrentPosition());
            telemetry.addData("lBack", lBack.getCurrentPosition());
            telemetry.addData("rBack", rBack.getCurrentPosition());
            telemetry.update();
        }
    }

    public void turn(double d, int t) {
        lFront.setVelocity(-t);
        rFront.setVelocity(t);
        lBack.setVelocity(-t);
        rBack.setVelocity(t);
    }
}