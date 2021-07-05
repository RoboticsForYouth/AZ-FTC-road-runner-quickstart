package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

public abstract class AutoBase extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Base base;
    IMU imu;
    WobbleTool wobbleTool;
    Launcher launcher;
    RingDetector detector;
    boolean first;
    String numRings;
   

    public void initAuto(String webcam) {
        
        base = new Base(this);
        base.init();
        
        wobbleTool = new WobbleTool(this);
        
        detector = new RingDetector(this, webcam);
        
        imu = new IMU(this, base);
        
        launcher = new Launcher(this); 
        
        first = true;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        launcher.init();
    }
    
    @Override
    public abstract void runOpMode(); 

    public void detect(boolean powershot) {
        
        //Init Start
        if(powershot) {
            launcher.startWithVelocityEx(950);
        } else {
            double vel = 1500;
            launcher.rightMotor.setVelocity(vel);
            launcher.leftMotor.setVelocity(vel*0.5);
            //launcher.startWithVel(850);

        }
        wobbleTool.grip();
        base.runToPosition(-500, 0.25);
        
        numRings = "Zero";
        telemetry.addLine("numRings: " + numRings);
        telemetry.update();
        

        
        
        
    }
    
    public void moveToShoot(boolean isRight, boolean delay) {
        

                
                //Move to Shoot
                //parameter = left/right
                if(isRight) {
                    base.moveLeft(700, 0.5);
                } else {
                    base.moveRight(700, 0.5);
                }
                
                rotateTo0(2);
                if (delay) {
                    sleep(6000);
                } 
                
                base.runToPosition(-2050, 0.5);
                
                sleep(200);
                
                if(isRight) {
                    imu.rotate(18, 0, 0.25);
                } else {
                    imu.rotate(342, 0, 0.25);

                }
                sleep(200);
                    
                launcher.testLaunch(1);
                sleep(100);
               
                imu.rotate(0, 0, 0.3);
                sleep(100);
                imu.rotate(0, 0, 0.25);
                sleep(100);
                imu.rotate(0, 0, 0.3);
                sleep(100);
                imu.rotate(0, 0, 0.25);
                //Move to shoot Done 
    }
    
    public void powerShot(boolean right) {
        double shot1Vel = 950;
        double shot2Vel = 930;
        double shot3Vel = 920; 
        
        base.runToPosition(-2000, 0.7);
       // if(right) {
        //    base.moveLeft(400, 0.75);
       // } else {
        //    base.moveRight(400, 0.75);
      //  }
        
        //sleep(500);
        imu.rotate(0, 0, 0.35);
        sleep(100);
        imu.rotate(0, 0, 0.25);
        sleep(400);
        
        launcher.autoPowerShot(shot1Vel);
        
        base.moveRight(500, 0.8);
       // base.turnLeft(0.25);
       // sleep(100);
       // base.stopMotors();
        imu.rotate(0, 0, 0.25);
        sleep(100);
        
        launcher.autoPowerShot(shot2Vel);
        
        base.moveRight(540, 0.8);
        //base.turnLeft(0.25);
        //sleep(100);
        //base.stopMotors();
        imu.rotate(0, 0, 0.25);
        sleep(100);

    
        launcher.autoPowerShot(shot3Vel);
        
        launcher.stopLauncher();
        rotateTo0(3);

    }
    
    public void rotateTo0(int tryCount){
        for(int i=0; i< tryCount; i++){
            sleep(500);
            imu.rotate(0, 0, 0.25);
        }
        sleep(500);
    }
                
}
