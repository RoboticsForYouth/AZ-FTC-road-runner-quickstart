package org.firstinspires.ftc.teamcode.powerplay.tools;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous
public class ConeTool extends LinearOpMode {

    private LinearOpMode opMode;
    Lift lift;
    Claw claw;

    public ConeTool(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode);
    }

    //use only during testing
    public ConeTool() {

    }

    private void init(LinearOpMode opMode){
        lift = new Lift(opMode);
        claw = new Claw(opMode);
    }

    public void grabCone(){
        claw.close();
    }

    public void dropCone(){
        if( lift.getCurrentState() == Lift.LiftLevel.LOW ||
            lift.getCurrentState() == Lift.LiftLevel.MEDIUM ||
            lift.getCurrentState() == Lift.LiftLevel.HIGH
        ) {
            lift.lowerToDrop();
            sleep(500);
        }
        claw.open();

    }

    public void liftTo(Lift.LiftLevel liftLevel){
        lift.liftTo(liftLevel);
    }


    //use only when you need to manually reset motor to zero position
    //Scenario: The robot disconnects during competition and the slide is
    //not in zero position. We need to reset the slide to zero position for
    //all out preset buttons to work
    public void raiseWithoutEncoder(double power){lift.raiseWithoutEncoder(power);}
    public void lowerWithoutEncoder(double power){lift.lowerWithoutEncoder(power);}
    public void stopLift(){lift.stopLift();}

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        init(this);

        waitForStart();

        grabCone();
        sleep(2000);
        liftTo(Lift.LiftLevel.LOW);
        sleep(1000);
        dropCone();
        sleep(1000);
        liftTo(Lift.LiftLevel.ZERO);
        sleep(2000);
    }


    public void setConeThreshold() {
        claw.setGrabThreshold();
    }

    public boolean isConeDetected() {
        return claw.isConeDetected();
    }
    public int getConeColor(int red, int green, int blue) {
        if(red > blue) {
            return Color.RED;
        }
        return Color.BLUE;
    }
    public void setConeColor(int color) {
        claw.setConeColor(color);
    }
    public void setDropHeight(int height) {
        lift.setDropHeight(height);
    }
}
