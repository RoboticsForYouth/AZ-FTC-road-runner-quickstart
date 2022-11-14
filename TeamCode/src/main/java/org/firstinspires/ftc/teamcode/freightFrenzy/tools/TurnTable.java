package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.text.MessageFormat;

@Autonomous(name = "TurnTableAuto")
public class TurnTable extends LinearOpMode {

    DcMotorEx turnMotor;
    Arm arm;
    LinearOpMode opMode;
    private static final int INC = 100;
    private static final int MAX = 300;
    private static final int MIN = -300;
    private static final int TICKS_PER_ROTATION = 1120;

    private static final double P = 3;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 0.0;


    PIDFCoefficients pidfCoefficients = new PIDFCoefficients();
    private boolean isIntakeMode;

    public enum Direction {
        CLOCKWISE, COUNTER_CLOCKWISE
    }

    public void setup() {

        //opMode = this;
        if (arm == null) {
            arm = new Arm(opMode);
        }
        turnMotor = opMode.hardwareMap.get(DcMotorEx.class, "turnTable");

        turnMotor.setPositionPIDFCoefficients(P);
        setupPos();
        //set encoder to zero

    }

    public void setupPos() {
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AZUtil.setMotorTargetPosition(turnMotor, 0, 0.2);
        arm.setupPos();
    }

    public TurnTable(LinearOpMode opMode, Arm arm) {
        this.opMode = opMode;
        this.arm = arm;
        setup();
    }


    public void turnLeft() {
        int pos = turnMotor.getCurrentPosition() + INC;
        if (pos < MIN)
            return;
        turnToPos(pos, 1, false);

    }

    public void turnRight() {
        int pos = turnMotor.getCurrentPosition() - INC;
        if (pos >  MAX)
            return;
        turnToPos(pos, 1, false);
    }

    public void turnTo0() {
        double position = turnMotor.getCurrentPosition();
        if (!(position < 3 && position > -3)) {
            Direction direction = Direction.COUNTER_CLOCKWISE;
            if( position < 0) {
                direction = Direction.CLOCKWISE;
            }
            turnTo(direction, 0, true);
        }
    }

    public TurnTable() {
        super();
    }

    public void turnRamp() {
        double power = 0.2;
        int pos = turnMotor.getCurrentPosition() + 10;
        AZUtil.setMotorTargetPosition(turnMotor, 350, power);
        while (!AZUtil.isAtPos(turnMotor, 350)) {
            sleep(500);
            if (power >= 0.25)
                power -= 0.1;
            turnMotor.setPower(power);
        }
    }

    public synchronized void turnToPos(int pos, double power, boolean armLevelCheck) {
        if(armLevelCheck) {
            arm.moveToMinLevel(Arm.ArmLevel.MOVE);
            sleep(350);
        }

        AZUtil.setMotorTargetPosition(turnMotor, pos, power);
        AZUtil.waitUntilMotorAtPos(opMode, turnMotor, pos);
    }
    public void turnToPos(int pos, double power) {
         turnToPos(pos, power, true);
    }


    public void turnTo(Direction direction, int deg) {
      turnTo(direction, deg, true);
    }
    public void setup0() {
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AZUtil.setMotorTargetPosition(turnMotor, 0, 0.2);
    }
    public void moveLeftManual() {
        turnMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        turnMotor.setPower(-0.2);
        opMode.sleep(1000);
        turnMotor.setPower(0);
    }

    public void moveRightManual() {
        turnMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        turnMotor.setPower(0.2);
        opMode.sleep(1000);
        turnMotor.setPower(0);
    }

    void turnTo(Direction direction, int deg, boolean armLevelCheck) {
        double degPerTick = 0.105;
        int ticks = (int) (deg / degPerTick);
        //int ticks = 90/3

        opMode.telemetry.addData("TurnTable Ticks ", ticks);
        opMode.telemetry.update();

        if (direction == Direction.COUNTER_CLOCKWISE) {
            ticks = -ticks;
        }

        turnToPos(ticks, 1, armLevelCheck);
    }

    public void turnInc(Direction direction){
        int sign =1;
        if (direction == Direction.COUNTER_CLOCKWISE) {
            sign = -sign;
        }
        int pos = turnMotor.getCurrentPosition() + (INC*sign);
        turnToPos( pos, 1,false);
    }


    public String getDisplayValues() {
        return MessageFormat.format("TurnTable Position; {0}", turnMotor.getCurrentPosition());

    }

    public void stopIntakeMode() {
        isIntakeMode = false;
    }

    public boolean isBusy() {
        return turnMotor.isBusy();
    }

    @Override
    public void runOpMode() {
        opMode = this;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Initialized");
        telemetry.update();
        setup();

        waitForStart();

        turnTo(Direction.CLOCKWISE, 90);
        turnTo0();
        AZUtil.print(telemetry, "Turn Table Position 100", turnMotor.getCurrentPosition());
        turnTo(Direction.COUNTER_CLOCKWISE, 150);
        turnTo0();
        AZUtil.print(telemetry, "Turn Table Position 0", turnMotor.getCurrentPosition());
        sleep(5000);


    }
}