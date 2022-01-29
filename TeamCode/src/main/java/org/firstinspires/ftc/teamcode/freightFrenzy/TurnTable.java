package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TurnTable extends LinearOpMode{
    
    DcMotor turnMotor;
    Arm arm;
    LinearOpMode opMode;
    private static final int INC = 30;
    private static final int MAX = 235; 
    private static final int MIN = -235;
    private static final int rampInc = 50;

    double power = 0.8;

    
    public void setup(){
        turnMotor = opMode.hardwareMap.get(DcMotor.class, "turnTable");
        //set encoder to zero

    }
    public void setupPos() {
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AZUtil.setMotorTargetPostion(turnMotor, 0, 0.2);
        arm.setupPos();
    }
    public TurnTable() {
        opMode = this;
        arm = new Arm(opMode);
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
        AZUtil.setMotorTargetPostion(turnMotor, pos, 0.5);
        AZUtil.waitUntilMotorAtPos(opMode, turnMotor, pos);
      
    } 
    public void turnRight() {
        int pos = turnMotor.getCurrentPosition() - INC;
        if(pos < MIN) 
            return;
        arm.moveToLevel(4);
        AZUtil.setMotorTargetPostion(turnMotor, pos, 0.5);
        AZUtil.waitUntilMotorAtPos(opMode, turnMotor, pos);
    } 
    public void turnTo0() {
        arm.moveToLevel(4);
        AZUtil.setMotorTargetPostion(turnMotor, 0, 0.2);
        AZUtil.waitUntilMotorAtPos(opMode, turnMotor, 0);
    }
    public void turnRamp() {
        int pos = turnMotor.getCurrentPosition() + rampInc;
        AZUtil.setMotorTargetPostion(turnMotor, 350, power);
        while(!AZUtil.isAtPos(turnMotor, 350)) {
            sleep(500);
            if(power >= 0.25)
                power -= 0.1;
            turnMotor.setPower(power);
        }
    }

    public void turnToPos(int pos) {
        AZUtil.setMotorTargetPostion(turnMotor, pos, 0.1);
        AZUtil.waitUntilMotorAtPos(opMode, turnMotor, pos);
    }

    @Override
    public void runOpMode(){
        setup();

        waitForStart();
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



    }
}