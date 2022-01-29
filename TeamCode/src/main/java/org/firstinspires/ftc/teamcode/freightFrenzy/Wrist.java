package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



public class Wrist extends LinearOpMode{
    
    Servo wrist;
    LinearOpMode opMode;
    
    private static final double INTAKE_POS=0.9;
    private static final double LEVEL_1_DROP_POS=0.82;
    private static final double LEVEL_2_DROP_POS=0.81;
    private static final double LEVEL_3_DROP_POS=0.97;
    private static final double HOME_POS=0.5;
    private static final double SECURED_POS=0.3;

    public void setup() {
        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        
    }
    public void setupPos() {
        wrist.setPosition(HOME_POS);
    }

    public Wrist() {
        opMode = this;
    }
    public Wrist(LinearOpMode opMode) {
        this.opMode = opMode;
        setup();
    }
    
    public void intakePos() {
        wrist.setPosition(INTAKE_POS);
    }
    
    public void setLevel1DropPos() {
        wrist.setPosition(LEVEL_1_DROP_POS);
    }

    public void setLevel2DropPos() { wrist.setPosition(LEVEL_2_DROP_POS); }

    public void setLevel3DropPos() { wrist.setPosition(LEVEL_3_DROP_POS); }

    public void homePos() { wrist.setPosition(HOME_POS); }

    public void setSecuredPos() { wrist.setPosition(SECURED_POS); }

    @Override
    public void runOpMode(){
      setup();
      waitForStart();
      setupPos();
      
      intakePos();
      sleep(4000);
      //dropPos();
      sleep(4000);
      homePos();
      telemetry.addLine("done" );
      telemetry.update();
      sleep(4000);
    }
        
    
}
