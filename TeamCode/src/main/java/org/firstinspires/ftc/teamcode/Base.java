package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math;


public class Base{
    private ElapsedTime runtime = new ElapsedTime();
    DcMotorEx backL, frontL, backR, frontR;
    LinearOpMode linearOpMode;
    
    public Base(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        backL  = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        frontL = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        backR  = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");
        frontR = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");

        backL.setDirection(DcMotor.Direction.REVERSE);
        backR.setDirection(DcMotor.Direction.FORWARD);
        frontL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);

        this.runtime.reset();
    }
    
    public void init(){
        setupMotor(backL);
        setupMotor(frontL);
        setupMotor(backR);
        setupMotor(frontR);
        
        stopMotor(backL);
        stopMotor(frontL);
        stopMotor(backR);
        stopMotor(frontR);
    }
    
    public void stopMotors(){
        frontL.setPower(0.0);
        backL.setPower(0.0);
        frontR.setPower(0.0);
        backR.setPower(0.0);
        
        //disableEncoder();
    }
    public void stopMotorsForward()
    {
        backL.setPower(0.0);
        backR.setPower(0.0);
        frontL.setPower(0.0);
        frontR.setPower(0.0);
        
        //disableEncoder();
    }
    public void stopMotorsBackward()
    {
        frontR.setPower(0.0);
        frontL.setPower(0.0);
        backR.setPower(0.0);
        backL.setPower(0.0);
        
        //disableEncoder();
    }
    
    private void setupMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void stopMotor(DcMotor motor) {
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0);
    }
    
    public void enableEncoder(){
        stopMotor(backR);
        stopMotor(backL);
        stopMotor(frontR);
        stopMotor(frontL);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void disableEncoder(){
        backR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void setFloatMode(){
        
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        disableEncoder();
    }
    void setPower(double bl, double fl, double br, double fr){
        backL.setPower(bl);
        frontL.setPower(fl);
        backR.setPower(br);
        frontR.setPower(fr);
       
    }
    public void testMotor() {
        init();
        double factor = 1;
        
        //backL.setPower(0.25);
        //frontL.setPower(0.25);
        //backR.setPower(0.25);
        //frontR.setPower(0.25);
        
        //backL.setTargetPosition(1000);
        //frontL.setTargetPosition(1000);
        //backR.setTargetPosition(1000);
        //frontR.setTargetPosition(1000);
    }
    private void setPositionForward(int position, double drivePower){
        init();
        double factor = 0.665;
        
        backR.setPower(drivePower);
        frontR.setPower(drivePower);
        backL.setPower(drivePower);
        frontL.setPower(factor*drivePower);
        
        
        backL.setTargetPosition(position);
        frontL.setTargetPosition(position);
        backR.setTargetPosition(position);
        frontR.setTargetPosition(position);
    }
    
     void setPowerForward(double drivePower){
        //double factor = 1;
        
        backR.setPower(drivePower);
        frontR.setPower(drivePower);
        backL.setPower(drivePower);
        frontL.setPower(drivePower);
        
    }
    
    private void setPositionBackward(int position, double drivePower){
        init();
        
        double flfactor = 0.665;
        double brfactor = 1;
        double frfactor = 1;
        //-1462, -1802, -1991, 1885
        backL.setPower(drivePower);
        frontL.setPower(flfactor*drivePower);
        backR.setPower(brfactor*drivePower);
        frontR.setPower(frfactor*drivePower);
        
        backL.setTargetPosition(position);
        frontL.setTargetPosition(position);
        backR.setTargetPosition(position);
        frontR.setTargetPosition(position);
    }
     void setPowerBackward(double drivePower){
         //original value was 0.965 -Dinesh
        //double factor = 1;
        
        backL.setPower(drivePower);
        frontL.setPower(drivePower);
        backR.setPower(drivePower);
        frontR.setPower(drivePower);
    }
    public void runToPosition(int position, double drivePower){
        if(position<0){
            setPositionBackward(position, drivePower);
        }
        else{
            setPositionForward(position, drivePower);
        }
        
        int difference = Math.abs(backL.getCurrentPosition() - backL.getTargetPosition());
        
        linearOpMode.telemetry.addData("motor", backL.getCurrentPosition());
        linearOpMode.telemetry.update();
        while(difference > 20 && linearOpMode.opModeIsActive()) {
            difference = Math.abs(backL.getCurrentPosition() - backL.getTargetPosition());
            linearOpMode.telemetry.addData("front left", frontL.getCurrentPosition());
            linearOpMode.telemetry.addData("front right", frontR.getCurrentPosition());
            linearOpMode.telemetry.addData("back left", backL.getCurrentPosition());
            linearOpMode.telemetry.addData("back right", backR.getCurrentPosition());
            linearOpMode.telemetry.update();
        }
        
        linearOpMode.telemetry.addData("stopping", backL.getTargetPosition());
        linearOpMode.telemetry.update();
        
        if(position<0){
            stopMotorsBackward();
        }
        else{
            stopMotorsForward();
        }
        
        
        linearOpMode.telemetry.addData("target", position);
        linearOpMode.telemetry.addData("front left", frontL.getCurrentPosition());
        linearOpMode.telemetry.addData("front right", frontR.getCurrentPosition());
        linearOpMode.telemetry.addData("back left", backL.getCurrentPosition());
        linearOpMode.telemetry.addData("back right", backR.getCurrentPosition());
        linearOpMode.telemetry.update();
    }
    public boolean isBusy()
    {
        return frontR.isBusy() || backR.isBusy() || frontL.isBusy() || backL.isBusy();
    }
    private void setPositionLeft( int position, double drivePower){
        init();
        backL.setPower(0.68 * -drivePower);
        frontL.setPower(drivePower);
        backR.setPower(0.7 * drivePower);
        frontR.setPower(-drivePower);
        backL.setTargetPosition(-position);
        frontL.setTargetPosition(position);
        backR.setTargetPosition((int)(position));
        frontR.setTargetPosition((int)(-position));
    }
    
    private void setPowerLeft(double drivePower){
        backL.setPower(-drivePower);
        frontL.setPower(drivePower);
        backR.setPower(drivePower);
        frontR.setPower(-drivePower);
    }
    public void moveLeft(int position, double drivePower)
    {
        setPositionLeft(-position, drivePower);
        int difference = Math.abs(frontR.getCurrentPosition() - frontR.getTargetPosition());
        linearOpMode.telemetry.update();
        linearOpMode.telemetry.addData("motor", frontR.getCurrentPosition());
        linearOpMode.telemetry.update();
        int loopCount = 0;
        int prevPos = 0;
        while (linearOpMode.opModeIsActive() && difference > 50 && frontR.isBusy()) { 
            difference = Math.abs(frontR.getCurrentPosition() - frontR.getTargetPosition());
            
            //While target has not been reached
            
        }
        stopRobot();
        linearOpMode.telemetry.addData("target", position);
        linearOpMode.telemetry.addData("frontRight", frontR.getCurrentPosition());
        linearOpMode.telemetry.addData("frontLeft", frontL.getCurrentPosition());
        linearOpMode.telemetry.addData("backLeft", backL.getCurrentPosition());
        linearOpMode.telemetry.addData("backRight", backR.getCurrentPosition());
        linearOpMode.telemetry.update();
    }
    private void setPositionRight( int position, double drivePower){
        init();
        backL.setPower(0.71 * drivePower);
        frontL.setPower(-drivePower);
        backR.setPower(0.65 * -drivePower);
        frontR.setPower(drivePower);
        backL.setTargetPosition(position);
        frontL.setTargetPosition(-position);
        backR.setTargetPosition(-position);
        frontR.setTargetPosition(position);
    }
    
    private void setPowerRight(double drivePower){
        backL.setPower(drivePower);
        frontL.setPower(-drivePower);
        backR.setPower(-drivePower);
        frontR.setPower(drivePower);
    }
    
    public void moveRight(int position, double drivePower) {
        setPositionRight(-position, drivePower);
        int difference = Math.abs(frontR.getCurrentPosition() - frontR.getTargetPosition());
            
        linearOpMode.telemetry.update();
            linearOpMode.telemetry.addData("motor", frontR.getCurrentPosition());
            linearOpMode.telemetry.update();
            int loopCount =0;
            int prevPos = 0;
        while (linearOpMode.opModeIsActive() && difference > 50 && frontR.isBusy()) { //While target has not been reached
          difference = Math.abs(frontR.getCurrentPosition() - frontR.getTargetPosition());
            
        }
        stopRobot();
        linearOpMode.telemetry.addData("target", position);
        linearOpMode.telemetry.addData("frontRight", frontR.getCurrentPosition());
        linearOpMode.telemetry.addData("frontLeft", frontL.getCurrentPosition());
        linearOpMode.telemetry.addData("backLeft", backL.getCurrentPosition());
        linearOpMode.telemetry.addData("backRight", backR.getCurrentPosition());
        linearOpMode.telemetry.update();
    }
    
     public void turnLeft(double X)
    {
        disableEncoder();
        backL.setPower(-X);
        backR.setPower(X);
        frontL.setPower(-X);
        frontR.setPower(X);
    }
    public void turnRight(double X){
        disableEncoder();
        backL.setPower(X);
        backR.setPower(-X);
        frontL.setPower(X);
        frontR.setPower(-X);
    }
    
    public void stopRobot()
    {
        frontR.setPower(0.0);
        backR.setPower(0.0);
        frontL.setPower(0.0);
        backL.setPower(0.0);
        disableEncoder();
    }
    
    
}