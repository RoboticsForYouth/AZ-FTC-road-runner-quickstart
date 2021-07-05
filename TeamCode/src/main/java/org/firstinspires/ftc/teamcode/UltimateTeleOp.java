/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// @TeleOp(name="AZTeleOp", group="Linear Opmode")

public class UltimateTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backL;
    private DcMotor frontL;
    private DcMotor backR;
    private DcMotor frontR;
    private Intake intake;
    private Base base;
    private IMU imu;
    private WobbleTool wobbleTool; 
    private Launcher launcher; 
    
    private enum DriveMode {
        fourWheelDrive,
        frontWheelDrive,
        backWheelDrive
        
    }
    private DriveMode driveMode = DriveMode.fourWheelDrive;
    
    public double motor_fast_speed = 1;
    public double motor_slow_speed = 0.65;
    
    private boolean isCollecting = false;
    private boolean isReversing = false;
    private boolean isPushing = false;
    private boolean isPushed = false;
    private boolean isShooting = false;
    private boolean isLaunching = false;
    private boolean slow = false;
    
   
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backL  = hardwareMap.get(DcMotor.class, "leftBack");
        frontL = hardwareMap.get(DcMotor.class, "leftFront");
        backR  = hardwareMap.get(DcMotor.class, "rightBack");
        frontR = hardwareMap.get(DcMotor.class, "rightFront");

        backL.setDirection(DcMotor.Direction.REVERSE);
        backR.setDirection(DcMotor.Direction.FORWARD);
        frontL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        
        intake = new Intake(this);
        wobbleTool = new WobbleTool(this);
        launcher = new Launcher(this); 
        base = new Base(this);
        imu = new IMU(this, base);
        
        waitForStart();
        runtime.reset();
        launcher.init();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            //intake.odometer(); 
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double angle = Math.atan2(gamepad1.left_stick_y, -(gamepad1.left_stick_x)) - Math.PI / 4;
            double rightX = -0.7 * gamepad1.right_stick_x;
            final double fl = r * Math.cos(angle) + rightX;
            final double fr = r * Math.sin(angle) - rightX;
            final double bl = r * Math.sin(angle) + rightX;
            final double br = r * Math.cos(angle) - rightX;
            
            //base
            if(r > 0 || rightX > 0 || rightX < 0) {
                double joystickPower = -1*motor_fast_speed;
                if(gamepad1.dpad_left && slow == false){
                    joystickPower = -1*motor_slow_speed;
                    slow = true; 
                } else if (gamepad1.dpad_left)  { 
                    slow = false; 
                }
                if (driveMode == DriveMode.fourWheelDrive) {
                    frontL.setPower(joystickPower*fl);
                    frontR.setPower(joystickPower*fr);
                    backL.setPower(joystickPower*bl);
                    backR.setPower(joystickPower*br);  
                } else if (driveMode == DriveMode.frontWheelDrive) {
                    frontL.setPower(joystickPower*fl);
                    frontR.setPower(joystickPower*fr);
                } else if (driveMode == DriveMode.backWheelDrive) {
                    backL.setPower(joystickPower*bl);
                    backR.setPower(joystickPower*br);
                }
                
            }
            else{
                frontL.setPower(0.0);
                frontR.setPower(0.0);
                backL.setPower(0.0);
                backR.setPower(0.0);  
            }
            
            if(gamepad2.x && gamepad2.y) {
                driveMode = DriveMode.frontWheelDrive;
            }
            
            if(gamepad2.a && gamepad2.b) {
                driveMode = DriveMode.backWheelDrive;
            }
            
            if(gamepad2.a && gamepad2.y) {
                driveMode = DriveMode.fourWheelDrive;
            }
            
            //intake
            if(gamepad1.right_bumper) {
                if(!isCollecting){
                    intake.collect();
                    isCollecting = true;
                    
                }
            } else if(isCollecting){
                intake.stop();
                isCollecting = false;
            }
            
            if(!gamepad1.right_bumper) {
                isPushed = false;
            }
            
            // else if(!gamepad1.right_bumper && isCollecting){
            //         intake.stop();
            //         isCollecting = false;
            // }
            
            
            
            
            if(gamepad1.left_bumper) {
                if( !isReversing){
                    ParallelUtil.runInParallel(
                        new Runnable(){
                            public void run(){
                                intake.reverse();
                            }
                        }
                    );
                    isReversing = true;
                }
            } else if( isReversing) {
                    intake.stop();
                    isReversing = false;
            }
            
            
           
            //wobble arm
            if(gamepad1.a) {
                wobbleTool.rest();
            }
            
            if(gamepad2.x) {
                launcher.toggleLaunchPower();
            }
            if(gamepad1.b) {
                if(!isPushing){
                ParallelUtil.runInParallel(
                        new Runnable(){
                            public void run(){
                                intake.push();
                                isPushing = true;
                            }
                        }
                    );
                    
                }
            } else if(isPushing){
                isPushing = false;
            }
            
            if(gamepad1.y) {
                wobbleTool.down();
            }
            if(gamepad1.x) {
                wobbleTool.up();
            }
            if(gamepad1.dpad_right) {
                wobbleTool.release();
                wobbleTool.pos(0.6);
            }
            
            //wobble grip
            if(gamepad1.dpad_up) {
                wobbleTool.grip();
            }
            if(gamepad1.dpad_down) {
                wobbleTool.release();
            }
            
            //launcher 
            
            if(gamepad1.left_trigger > 0) {
                if(isLaunching && isShooting == false){
                    
                    launcher.stopLauncher();
                    sleep(500);
                    isLaunching = false;
                }
                else if(isShooting == false) {
                 ParallelUtil.runInParallel(
                        new Runnable(){
                            public void run(){
                                
                                launcher.startWithVel(850); //900
                                isLaunching = true;
                                
                            }
                        }
                    );
                    sleep(500);
                }
            }
            
            if(gamepad1.right_trigger > 0){
                if(!isShooting) {
                    isShooting = true;
                ParallelUtil.runInParallel(
                        new Runnable(){
                            public void run(){
                                launcher.launch3();
                                isShooting = false;
                                isLaunching = false;
                            }
                        }
                    );
            
                }    
            }
            
            //power shots
            
            if(gamepad2.right_trigger > 0) {
                base.enableEncoder();
                base.runToPosition(-2200, 0.5);
                base.disableEncoder();
                launcher.waitUntilMotorVelocity(800);
                launcher.shoot();
                launcher.startWithVel(760); 
                base.turnRight(0.45);
                sleep(100);
                base.stopMotors();
                sleep(100);
                launcher.shoot();
                base.turnLeft(0.45);
                sleep(150);
                base.stopMotors();
                sleep(100);
                launcher.shoot();
                base.turnRight(0.4);
                sleep(100);
                base.stopMotors();
                launcher.stopLauncher();
                isLaunching = false;

                
            }
            if(gamepad2.left_trigger > 0) {
                if(isLaunching){
                    
                    launcher.stopLauncher();
                    sleep(500);
                    isLaunching = false;
                }
                else {
                 ParallelUtil.runInParallel(
                        new Runnable(){
                            public void run(){
                                
                                launcher.startWithVel(800); 
                                isLaunching = true;
                                
                            }
                        }
                    );
                    sleep(500);
                }
            }
            
           
        }
    }
}
