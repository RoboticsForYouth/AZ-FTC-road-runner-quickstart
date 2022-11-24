package org.firstinspires.ftc.teamcode.powerplay.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;

@TeleOp
public class SampleTeleOp extends LinearOpMode {
    SampleMecanumDrive drive;
    ConeTool coneTool;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        coneTool = new ConeTool(this);
        waitForStart();
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / 1.5,
                            -gamepad1.left_stick_x / 1.5,
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
        }
    }
}
