package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

@Autonomous(name="1RedLeftAuto", group="Linear Opmode")
public class RedLeftAuto extends AutoBase {

    @Override
    public void runOpMode() {
        
        initAuto("WebcamLeft");

        while (opModeIsActive()) {
            if (first) {
                detect(true); //true - Powershot, false - highgoal (vel)
                powerShot(false); //true - Right; false - Left
                
                if (numRings.equals("Quad")) {
                    
                    base.runToPosition(-2350, 0.45);
                    sleep(500);
                    imu.turnLeft90(); 
                    sleep(200);
                    base.runToPosition(900, 0.45);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.pos(0.6);
                    sleep(200);
                    base.runToPosition(-400, 0.45);
                    imu.rotate(0, 0, 0.4);
                    sleep(500);
                    base.runToPosition(1900, 0.6);
                    
                }
                else if (numRings.equals("Single")) {
                    base.moveRight(700, 0.6);
                    sleep(100);
                    imu.rotate(0, 0, 0.3);
                    sleep(100);
                    imu.rotate(0, 0, 0.25);
                    base.runToPosition(-1000, 0.45);
                    imu.turnLeft90();
                    sleep(500);
                    base.runToPosition(-50, 0.35);
                    wobbleTool.down();                    
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.up();
                    imu.rotate(0, 0, 0.35);
                    base.runToPosition(800, 0.5);
                    
                }
                else { 
                    base.runToPosition(-100, 0.7);
                    sleep(500);
                    imu.turnLeft90(); 
                    sleep(200);
                    base.runToPosition(1400, 0.7);
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.pos(0.6);
                    sleep(200);
                    base.runToPosition(-100, 0.45);
                    imu.rotate(0, 0, 0.5);
                    wobbleTool.down();
                    rotateTo0(2);
                    base.runToPosition(1800, 0.3);
                    //base.runToPosition(800, 0.3);
                    sleep(1000);
                    wobbleTool.grip();
                    sleep(1000);
                    wobbleTool.up();
                    base.runToPosition(-1400, 0.7);
                    imu.turnLeft90(); 
                    wobbleTool.down();
                    sleep(1000);
                    wobbleTool.release();
                    wobbleTool.pos(0.6);
                    sleep(100);
                    imu.rotate(180, 0, 0.5);

                    
                    


                  
                }
                first = false;
            }
            
            
        }
        launcher.stopLauncher();
        base.stopMotors();
    }
}
