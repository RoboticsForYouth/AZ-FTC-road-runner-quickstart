package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx intake;
    LinearOpMode linearOpMode;
    final static double VELOCITY= 900;
    final static double SLOW_VELOCITY= 800;
    private Servo   intakePush;

    public Intake(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        intake = linearOpMode.hardwareMap.get(DcMotorEx.class, "Intake");
        intakePush = linearOpMode.hardwareMap.get(Servo.class, "intakePush");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
        intakePush.setPosition(0.0);
        
        this.runtime.reset();
    }
    
    public void collect() {
        intake.setVelocity(SLOW_VELOCITY);
        // linearOpMode.telemetry.addData("position: ", intake.getCurrentPosition());
        // linearOpMode.telemetry.update();
    }
    public void slow() {
        intake.setVelocity(SLOW_VELOCITY);
        // linearOpMode.telemetry.addData("position: ", intake.getCurrentPosition());
        // linearOpMode.telemetry.update();
    }
     public void push() {
         
         
        //  intakePush.setPosition(0.69);
        //  linearOpMode.sleep(1000);
        //  intakePush.setPosition(0.6);
        //  linearOpMode.sleep(200);
         intakePush.setPosition(0.69);
         linearOpMode.sleep(1000);
        //  intakePush.setPosition(0.5);
        //  linearOpmode.sleep(500);
        //  intakePush.setPosition(0.7);
         
         intakePush.setPosition(0.0);
 
     }
    public void stop() {
        intake.setPower(0);
    }
    public void reverse() {
        intake.setVelocity(-VELOCITY);
    }
    public void odometer(){
        linearOpMode.telemetry.addData("position: ", intake.getCurrentPosition());
        linearOpMode.telemetry.update();
    }
    
}