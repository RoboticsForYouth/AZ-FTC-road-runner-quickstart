package org.firstinspires.ftc.teamcode.powerplay.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class ClawTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Servo claw;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

        setup();
        waitForStart();
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            if( gamepad1.a){
                claw.setPosition(1);
            }
            if(gamepad1.b){
                claw.setPosition(0.5);
            }

        }
    }

    private void setup() {
        claw = hardwareMap.get(Servo.class, "claw");
    }
}
