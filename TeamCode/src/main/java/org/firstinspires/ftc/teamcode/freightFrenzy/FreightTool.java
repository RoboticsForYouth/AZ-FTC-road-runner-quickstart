package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class FreightTool extends LinearOpMode {
    Intake intake;
    Wrist wrist;
    Arm arm;
    TurnTable turnTable; 
    LinearOpMode opMode;
    boolean isFreightToolBusy = false;

    //Moves freightTool to allianceHub pos
    public void allianceHub() {
        arm.moveToLevel(Arm.ArmLevel.LEVEL3);
        wrist.setLevel3DropPos();
    }

    //moves freightTool to sharedHub pos
    public void sharedHub() {
        arm.moveToLevel(Arm.ArmLevel.LEVEL1);
    }

    public void dropFreight() {
        intake.drop();
    }

    private enum ToolState {
        HOME, SHARED_HUB_DROP, ALLIANCE_HUB_DROP, INTAKE, MOVE, DROP, TAPE_DRIVE
    }

    ToolState toolState = ToolState.HOME;

    public void setup(){
        arm = new Arm(opMode);
        turnTable = new TurnTable(opMode, arm);
        intake = new Intake(opMode);
        wrist = new Wrist(opMode);
    }

    public void setupPos(){
//        wrist.setupPos();
        turnTable.setupPos();
    }
    public FreightTool() {
        opMode = this;
    }

    public FreightTool(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }
    public void prepForDrop(int level) {
        arm.moveToLevel(level);
       // wrist.setSecuredPos();
        turnTable.turnToPos(280, 0.2);
//        if(level == 1) {
//            wrist.setLevel1DropPos();
//        } else if(level == 2) {
//            wrist.setLevel2DropPos();
//        }

    }
    public void intake() {
        if(toolState != ToolState.INTAKE) {
            toolState = ToolState.INTAKE;
            turnTable.turnTo0();
            wrist.intakePos();
            sleep(1000);
            arm.moveToLevel(5);
            intake.intake();
        }
    }

    public void move() {
        if(toolState != ToolState.MOVE) {
            toolState = ToolState.MOVE;
            intake.stopIntake();
            wrist.setSecuredPos();
            turnTable.turnTo0();
            arm.moveToLevel(4);
        }
    }

    public void drop() {
        intake.drop();
        //sleep(1000);
    }
    public void reset() {
        turnTable.turnToPos(0, 0.1);
        wrist.homePos();
        sleep(1000);
        arm.moveToLevel(1);
    }

    public synchronized void prepSharedHubDrop() {
        isFreightToolBusy = true;
        arm.moveToLevel(Arm.ArmLevel.ARMROTATE);
        turnTable.turnTo(TurnTable.Direction.CLOCKWISE, 115);
        waitUntilMotorBusy();
        drop();
        isFreightToolBusy = false;
    }

    public synchronized void moveTo0() {
        isFreightToolBusy = true;
        turnTable.turnToPos(0,0.3);
        arm.moveTo0();
        isFreightToolBusy = false;
    }
    private void waitUntilMotorBusy() {
        while(arm.isBusy() || turnTable.isBusy()) {
            sleep(100);
        }
    }

    public void waitUntilBusy() {
        while(isFreightToolBusy) {
            sleep(100);
        }

    }

    public String getDisplayValues() {
        return new StringBuilder()
                .append(arm.getDisplayValues())
                .append("\n")
                .append(turnTable.getDisplayValues())
                .toString();
    }

    @Override
    public void runOpMode(){
        setup();
        
        waitForStart();
        wrist.setupPos();

        wrist.intakePos();
        intake.timeIntake(2000);
        arm.moveToLevel(1);
       // wrist.dropPos();
        intake.drop();
        sleep(10000);
        
 
    }
}