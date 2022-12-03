package org.firstinspires.ftc.teamcode.powerplay.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;

@TeleOp
public class PowerPlayTeleOp extends LinearOpMode {
    public static final int DEFAULT_DRIVE_FACTOR = 2;
    public static final int SLOW_DRIVE_FACTOR = 4;
    SampleMecanumDrive drive;
    ConeTool coneTool;
    boolean manualLiftOp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        coneTool = new ConeTool(this);
        waitForStart();
        while (opModeIsActive()) {
            int driveFactor = DEFAULT_DRIVE_FACTOR;
            if( gamepad1.right_bumper){
                driveFactor = SLOW_DRIVE_FACTOR;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / driveFactor,
                            -gamepad1.left_stick_x / driveFactor,
                            gamepad1.right_stick_x
                    )
            );

            drive.update();
            if (gamepad1.dpad_right) {
                coneTool.liftTo(Lift.LiftLevel.HIGH);
            } else if (gamepad1.dpad_up) {
                coneTool.liftTo(Lift.LiftLevel.MEDIUM);
            } else if (gamepad1.dpad_left) {
                coneTool.liftTo(Lift.LiftLevel.LOW); }
            else if (gamepad1.dpad_down) {
                coneTool.liftTo(Lift.LiftLevel.CLEAR);
            } else if (gamepad1.left_bumper) {
                coneTool.liftTo(Lift.LiftLevel.ZERO);
            }
            else if (gamepad1.y) {
                coneTool.liftTo(Lift.LiftLevel.FOURTH_CONE);
            }
            else if (gamepad1.a) {
                coneTool.liftTo(Lift.LiftLevel.SECOND_CONE);
            }
            if (gamepad1.x) {
                AZUtil.runInParallel(new Runnable() {
                    @Override
                    public void run() {
                        coneTool.dropCone();
                    }
                });
            } else if (gamepad1.b) {
                AZUtil.runInParallel(new Runnable() {
                    @Override
                    public void run() {
                        coneTool.grabCone();
                    }
                });
            }

            //use for correcting lift position when it stops in auto or
            //robot disconnects
            if(gamepad1.left_trigger > 0){
                manualLiftOp = true;
                coneTool.lowerWithoutEncoder(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger > 0){
                manualLiftOp = true;
                coneTool.raiseWithoutEncoder(gamepad1.right_trigger);
            }

            if( manualLiftOp
                    && gamepad1.left_trigger == 0
                    && gamepad1.right_trigger == 0
            ){
                coneTool.stopLift();
                manualLiftOp = false;
            }
        }
    }
}
