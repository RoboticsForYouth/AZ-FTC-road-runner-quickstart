package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.text.MessageFormat;

@TeleOp
public class TurnTable extends LinearOpMode{
    
    DcMotorEx turnMotor;
    Arm arm;
    LinearOpMode opMode;
    private static final int INC = 30;
    private static final int MAX = 235; 
    private static final int MIN = -235;
    private static final int TICKS_PER_ROTATION = 1120;

    private static final double P = 2;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 0.0;



    PIDFCoefficients pidfCoefficients = new PIDFCoefficients();

    public enum Direction{
        CLOCKWISE, COUNTER_CLOCKWISE
    }

    public void setup(){

        //opMode = this;
        if(arm == null) {
            arm = new Arm(opMode);
        }
        turnMotor = opMode.hardwareMap.get(DcMotorEx.class, "turnTable");

        turnMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(P,I,D,F));
        //set encoder to zero

    }
    public void setupPos() {
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AZUtil.setMotorTargetPostion(turnMotor, 0, 0.2);
        arm.setupPos();
    }

    public TurnTable(LinearOpMode opMode, Arm arm) {
        this.opMode = opMode;
        this.arm = arm;
        setup();
    }
    
    
     public void turnLeft() {
        int pos = turnMotor.getCurrentPosition() + INC;
        if(pos > MAX)
            return;
        arm.moveToLevel(4);
         turnToPos(pos, 0.5);

     }
    public void turnRight() {
        int pos = turnMotor.getCurrentPosition() - INC;
        if(pos < MIN) 
            return;
        arm.moveToLevel(4);
        turnToPos(pos, 0.5);
    } 
    public void turnTo0() {
        arm.moveToLevel(4);
        turnToPos(0, 0.2);
    }

    public TurnTable() {
        super();
    }
    public void turnRamp() {
        double power = 0.2;
        int pos = turnMotor.getCurrentPosition() + 10;
        AZUtil.setMotorTargetPostion(turnMotor, 350, power);
        while(!AZUtil.isAtPos(turnMotor, 350)) {
            sleep(500);
            if(power >= 0.25)
                power -= 0.1;
            turnMotor.setPower(power);
        }
    }

    public void turnToPos(int pos, double power) {
        AZUtil.setMotorTargetPostion(turnMotor, pos, power);
        AZUtil.waitUntilMotorAtPos(opMode, turnMotor, pos);
    }

    public void turnTo(Direction direction, int deg){


        double ticksPerDeg = 0.3;
        int ticks = (int) (deg / ticksPerDeg);
        //int ticks = 90/3

        opMode.telemetry.addData("TurnTable Ticks ", ticks);
        opMode.telemetry.update();

        if(direction == Direction.CLOCKWISE) {
            ticks = -ticks;
        }

        turnToPos(ticks, 1);


    }

    public String getDisplayValues() {
        return MessageFormat.format("TurnTable Position; {0}", turnMotor.getCurrentPosition());

    }

    public boolean isBusy() {
        return turnMotor.isBusy();
    }
    @Override
    public void runOpMode(){
        opMode = this;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Initialized");
        telemetry.update();
        setup();

        waitForStart();
        /*
        for(int i = 0; i < 8; i++) {
            turnRight();
            sleep(200);
        }
        turnTo0();
        sleep(1000);

        for(int i = 0; i < 8; i++) {
            turnLeft();
            sleep(200);
        }
        turnTo0();
        sleep(1000);
        */
        arm.moveToLevel(Arm.ArmLevel.ARMROTATE);
        turnTo(Direction.CLOCKWISE, 90);
        //telemetry.addData("PIDFCoefficients", turnMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        //telemetry.update();
        while(isBusy()) {
            //sleep(100);
            telemetry.addLine(getDisplayValues());
            telemetry.update();
        }
        sleep(5000);

        turnTo(Direction.CLOCKWISE, 0);
        arm.moveToLevel(Arm.ArmLevel.MOVE);
        sleep(10000);



    }
}