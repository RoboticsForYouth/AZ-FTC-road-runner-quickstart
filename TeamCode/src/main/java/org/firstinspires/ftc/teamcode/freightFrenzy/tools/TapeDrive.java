package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp (name = "TapeDriveTeleOp")
public class TapeDrive extends LinearOpMode{
    DcMotor tapeMotor = null;
    LinearOpMode opMode = this;
    FreightTool freightTool = null;

    public TapeDrive(LinearOpMode opMode, FreightTool freightTool){
        this.freightTool = freightTool;
        setup(opMode);
    }

    public TapeDrive(){
        super();
    }
    public void setup(LinearOpMode opMode){
        this.opMode = opMode;
        if(freightTool == null){
            freightTool = new FreightTool(opMode);
        }
        //get the motor
        tapeMotor = opMode.hardwareMap.get(DcMotor.class, "tapeDrive");
    }

    public void releaseTape(double power) {
        tapeMotor.setPower(power);
    }
    public void withdrawTape(double power) {
        tapeMotor.setPower(-power);
    }
    public void stopTape() {
        tapeMotor.setPower(0);
    }

    @Override
    public void runOpMode(){
        try{
            telemetry.addLine("Setup");
            telemetry.update();
            setup(this);
            waitForStart();
            freightTool.setupPos();
            freightTool.tapeDrivePos();
            while(opModeIsActive()){
                if(gamepad1.dpad_up){
                    freightTool.encMoveUp();
                }else if( gamepad1.dpad_down){
                    freightTool.encMoveDown();
                }else if(gamepad1.dpad_right){
                    freightTool.encTurnRight();
                }else if(gamepad1.dpad_left){
                    freightTool.encTurnLeft();
                }
                if (gamepad1.left_trigger > 0){
                    withdrawTape(gamepad1.left_trigger);
                }
                else if (gamepad1.right_trigger > 0){
                    releaseTape(gamepad1.right_trigger);
                }
                else {
                    stopTape();
                }
            }

            // telemetry.addLine("Initialized");
            // telemetry.update();

            // tapeMotor.setPower(1);
            // sleep(2000);

            // tapeMotor.setPower(-1);
            // sleep(2000);
            // tapeMotor.setPower(0);
        } catch(Exception e){
            telemetry.addLine(e.toString());
            telemetry.update();
        }
        // sleep(2000);




    }


}