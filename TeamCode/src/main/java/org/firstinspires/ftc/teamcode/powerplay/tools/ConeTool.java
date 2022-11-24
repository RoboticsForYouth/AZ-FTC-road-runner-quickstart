package org.firstinspires.ftc.teamcode.powerplay.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ConeTool extends LinearOpMode {

    private LinearOpMode opMode;
    Lift lift;
    Claw claw;

    public ConeTool(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode);
    }

    //use only during testing
    public ConeTool(){

    }

    private void init(LinearOpMode opMode){
        lift = new Lift(opMode);
        claw = new Claw(opMode);
    }

    public void grabCone(){
        claw.close();
//        sleep(1000);
//        lift.liftTo(Lift.LiftLevel.CLEAR);
    }

    public void dropCone(){
//        lift.lowerToDrop();
//        sleep(500);
        claw.open();
//        sleep(500);
//        lift.raiseAfterDrop();
    }

    public void liftTo(Lift.LiftLevel liftLevel){
        lift.liftTo(liftLevel);
    }

    //use only when you need to manually reset motor to zero position
    //Scenario: The robot disconnects during competition and the slide is
    //not in zero position. We need to reset the slide to zero position for
    //all out preset buttons to work
    public void lowerWithoutEncoder(){
        lift.lowerWithoutEncoder();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        init(this);

        waitForStart();

        grabCone();
        sleep(2000);
        liftTo(Lift.LiftLevel.HIGH);
        sleep(2000);
        dropCone();
        sleep(2000);
        liftTo(Lift.LiftLevel.ZERO);
        sleep(2000);
    }


}
