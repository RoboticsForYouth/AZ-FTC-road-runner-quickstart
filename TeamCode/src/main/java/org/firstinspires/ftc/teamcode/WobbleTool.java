package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


public class WobbleTool {
    //jeff was here
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     = 0.5;     // Maximum rotational position
    static final double MIN_POS     =  0.3;     // Minimum rotational position
    static final double downPosition = 0.35; 
    static final double restingPosition = 1;
    static final double upPosition = 0.75; 
    
    private ElapsedTime runtime = new ElapsedTime();
    private Servo   wobbleArm;
    private Servo   wobbleGrip;
    LinearOpMode linearOpMode;
    
    public  WobbleTool( LinearOpMode opMode){
        this.linearOpMode = opMode;
        wobbleArm = linearOpMode.hardwareMap.get(Servo.class, "wobblearm");
        wobbleGrip = linearOpMode.hardwareMap.get(Servo.class, "wobblegrip");
        this.runtime.reset();
    }
    
    public void rest() {
        wobbleArm.setPosition(restingPosition);
        
    }
    public void up() {
        wobbleArm.setPosition(upPosition);
    }
 
    public void down() {
        wobbleArm.setPosition(downPosition);
    }
    
    public void grip() {
        wobbleGrip.setPosition(0.93);
        
    }
    
    public void release() {
        wobbleGrip.setPosition(0.5);
        
    }
    
    public void push() {
        wobbleArm.setPosition(0.3);
        
    }
    
    public void pos(double position) {
        wobbleArm.setPosition(position);
        
    }
}
