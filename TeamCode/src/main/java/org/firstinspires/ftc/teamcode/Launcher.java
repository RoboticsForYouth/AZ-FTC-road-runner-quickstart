package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Launcher {

    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private Base base;
    private Servo   rightLauncher;
    private Servo   leftLauncher, trigger;
    DcMotorEx leftMotor, rightMotor;
    private boolean launcherStarted = false;
    private final double GOAL_POWER = 0.43;
    private final double POWER_SHOT_POWER = 0.38;
    private double launcherPower = POWER_SHOT_POWER;
    public final static double AUTO_POWER = 0.37;
    private final static double SPIN_POWER = 0.99;

    private final double TRIGGER_INIT_POS = 0.0;
    private final double TRIGGER_SHOOT_POS = 0.3;
    LinearOpMode linearOpMode;
    


    //LEFT: max: 0.75, min: 0.5
    public Launcher(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        /*rightLauncher = linearOpMode.hardwareMap.get(Servo.class, "rightLauncher");
        leftLauncher = linearOpMode.hardwareMap.get(Servo.class, "leftLauncher"); */
        trigger = linearOpMode.hardwareMap.get(Servo.class, "trigger");
        leftMotor = (DcMotorEx)linearOpMode.hardwareMap.get(DcMotor.class, "LeftMotor");
        rightMotor = (DcMotorEx)linearOpMode.hardwareMap.get(DcMotor.class, "RightMotor");
        
        
        this.runtime.reset();
    }
    
    
    
    public void init() {
        //rightLauncher.setPosition(0);
       // leftLauncher.setPosition(1);
       MotorConfigurationType motorConfigurationType = leftMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        leftMotor.setMotorType(motorConfigurationType);
        rightMotor.setMotorType(motorConfigurationType);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        trigger.setPosition(TRIGGER_INIT_POS);
    }
    
    public void shoot(){
        trigger.setPosition(TRIGGER_SHOOT_POS);
        // linearOpMode.sleep(200);
        linearOpMode.sleep(100);
       trigger.setPosition(TRIGGER_INIT_POS);
        // linearOpMode.sleep(500);
         linearOpMode.sleep(50);
   }
    
    //test
    public void plus() {
        double factor = 0.955;
        double pos_r = rightLauncher.getPosition(); 
        double pos_l = leftLauncher.getPosition();
        
        rightLauncher.setPosition(pos_r + 0.1);
        leftLauncher.setPosition(pos_l - (0.1*factor));
        linearOpMode.telemetry.addData("right position: ", rightLauncher.getPosition());
        linearOpMode.telemetry.addData("left position: ", leftLauncher.getPosition());
        linearOpMode.telemetry.update();
        
    }
    
    //test
    public void minus() {
        double factor = 0.955;
        double pos_r = rightLauncher.getPosition(); 
        double pos_l = leftLauncher.getPosition();
        
        rightLauncher.setPosition(pos_r - 0.1);
        leftLauncher.setPosition(pos_l + (0.1*factor));
        linearOpMode.telemetry.addData("right position: ", rightLauncher.getPosition());
        linearOpMode.telemetry.addData("left position: ", leftLauncher.getPosition());
        linearOpMode.telemetry.update();
        
    }
    public void startLauncher() {
        if(!launcherStarted){
        rightMotor.setPower(launcherPower);
        leftMotor.setPower(launcherPower);
        launcherStarted = true;
        }
    }
    public void stopLauncher() {
        if(launcherStarted){
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        launcherStarted = false;
        }
    }
    
    public void toggleLaunchPower(){
        if( launcherPower == GOAL_POWER ){
            launcherPower = POWER_SHOT_POWER;
        } else {
            launcherPower = GOAL_POWER;
        }
    }
    public void setLauncherPower(double power){
        launcherPower = power; 
    }
    
    public void startWithVel(double vel){
        rightMotor.setVelocity(vel);
        leftMotor.setVelocity(vel*0.25);
        //leftMotor.setVelocity(vel*0.83);

        launcherStarted = true;
        
    }
    
    public void startWithVelocityEx(double vel){
        double leftMotorVelRatio = 0.4;
        leftMotor.setVelocity(vel);
        rightMotor.setVelocity(vel*leftMotorVelRatio); 
        launcherStarted = true;
    }
    
    public void testLaunch(double vel){
        
        double primeUpVel = 1200;
        double shot1Vel = 980;
        double shot2Vel = 1000;
        double shot3Vel = 990;
        // double shot2Vel = 1045;
        // double shot3vel = 1075;
        
        // startWithVelocityEx(primeUpVel);
     
        startWithVelocityEx(shot1Vel);
        
        linearOpMode.sleep(1250);
        // waitUntilElaspedTimeSecs(1);
        // waitUntilVelAndSecs(shot1Vel, 1);
        shoot();
        runtime.reset();
        // waitUntilMotorVelocity(vel); 
        startWithVelocityEx(shot2Vel);
        linearOpMode.sleep(1000);
        // waitUntilElaspedTimeSecs(1);
        shoot();
        startWithVelocityEx(shot3Vel);
        runtime.reset();
        // waitUntilMotorVelocity(vel);
        linearOpMode.sleep(1000);
        // waitUntilElaspedTimeSecs(1);
        shoot();
        stopLauncher();
        
    }
    public void autoPowerShot(double vel){
        
        
        // double shot2Vel = 1045;
        // double shot3vel = 1075;
        
        // startWithVelocityEx(primeUpVel);
     
        startWithVelocityEx(vel);
        
        linearOpMode.sleep(750);
       
        shoot();
        runtime.reset();
        
        
        
        
    }
    public void autoTestLaunch(){
        launcherStarted = true;

        double vel = 1500;
        shoot();
        waitUntilVelAndSecs(vel, 1.5);
        shoot();
        waitUntilVelAndSecs(vel, 1.5);
        shoot();
        stopLauncher();
        
    }
    
    public void waitUntilVelAndSecs(double vel, double secs){
        runtime.reset();
        waitUntilMotorVelocity(vel); 
        waitUntilElaspedTimeSecs(secs);
    }
    
    private void waitUntilElaspedTimeSecs(double secs){
        while(runtime.milliseconds() < (secs*1000)){
               linearOpMode.sleep(100);
        }
    }
    
    public String printVelocity(){
        return String.format("%f : %f", leftMotor.getVelocity(), rightMotor.getVelocity());
    }
    public void launch2() {
        shoot();
        startWithVel(781);
        linearOpMode.sleep(1000);
        shoot();
        stopLauncher();
    }
    public void launch3() {
        /*
        double vel = 890;
        //startWithVel(vel);
        //waitUntilMotorVelocity(vel);
        startWithVel(vel); //860
        linearOpMode.sleep(300);
        waitUntilMotorVelocity(vel); //860
        
        shoot();

        waitUntilMotorVelocity(vel); 
        // startWithVel(850); //850
        // linearOpMode.sleep(300);
        
        shoot();
        
        //waitUntilMotorVelocity(vel);
        //linearOpMode.sleep(1000);
        
        //startWithVel(807);
        //waitUntilMotorVelocity(vel);
        //linearOpMode.sleep(1000);
        // linearOpMode.sleep(500);
        waitUntilMotorVelocity(vel); 
        shoot();
        stopLauncher();
        */
        linearOpMode.sleep(500);
        shoot();
        linearOpMode.sleep(500);
        shoot();
        linearOpMode.sleep(500);
        shoot();
        stopLauncher();

    }
    

    public void autolaunch3() {
        double vel = 860;
        //startWithVel(vel);
        //waitUntilMotorVelocity(vel);
        startWithVel(860);
        waitUntilMotorVelocity(860);
        shoot();
        
        linearOpMode.sleep(100);

        shoot();
        
        //waitUntilMotorVelocity(vel);
        //linearOpMode.sleep(1000);
        
        //startWithVel(807);
        //waitUntilMotorVelocity(vel);
        //linearOpMode.sleep(1000);
        linearOpMode.sleep(100);
        shoot();
        stopLauncher();
    }
    
    public void waitUntilMotorVelocity(double vel){
       int count = 0;
       double currVel = rightMotor.getVelocity();
        while(!(currVel > (vel-0.5) &&
                currVel < (vel+0.5) ) && count < 30) {
            linearOpMode.sleep(100);
            count++;
            currVel = rightMotor.getVelocity();
        } 
        
        linearOpMode.telemetry.addData("vel: ", currVel);
        linearOpMode.telemetry.addData("ivel: ", vel);
        linearOpMode.telemetry.addData("count: ", count);
        linearOpMode.telemetry.update();
    }
    
    
    
}