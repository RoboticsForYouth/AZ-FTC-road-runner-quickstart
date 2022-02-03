package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.Carousel;
import org.firstinspires.ftc.teamcode.freightFrenzy.FreightTool;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class FFTeleOpStates extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backL;
    private DcMotor frontL;
    private DcMotor backR;
    private DcMotor frontR;
    Carousel duckyTool;
    FreightTool freightTool;
    LinearOpMode opMode;
    private boolean isDuckyToolStopped;

    private enum GamePad2Mode {
        NONE, TAPE_DRIVE, RESET_ARM, RESET_INTAKE
    }

    GamePad2Mode gamepad2Mode = GamePad2Mode.NONE;

    boolean duckyToolClockwise = true;

    public void setup() {
        freightTool = new FreightTool(this);

        duckyTool = new Carousel(this);
//        tapeDrive = new TapeDrive(this, freightTool);
    }

    public double motor_fast_speed = 1;
    public double motor_slow_speed = 0.5;
    boolean fast = true;

    boolean intakeOn = false;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        setup();

        waitForStart();
        runtime.reset();
        freightTool.setupPos();
        freightTool.move();

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


            if (gamepad1.right_bumper) {
                intakeCmd.run();
            }

            if (gamepad1.x) { //move pos
                freightTool.move();
            }

            if (gamepad1.dpad_up) { //move to alliance hub pos
                freightTool.allianceHub();
            }

            if (gamepad1.dpad_down) { //move to shared hub pos
                freightTool.sharedHub();
            }
            if (gamepad1.a) { //drop freight
                freightTool.dropFreight();
            }

            if (gamepad1.right_trigger > 0.1) {
                duckyTool.blueDuckyDrop();
                isDuckyToolStopped = false;
            } else if (gamepad1.left_trigger > 0.1) {
                duckyTool.redDuckyDrop();
                isDuckyToolStopped = false;
            } else {
                if (!isDuckyToolStopped) {
                    duckyTool.stopDuckyTool();
                    isDuckyToolStopped = true;
                }
            }

            telemetry.addData("gamepad", gamepad1);
            telemetry.update();
        }
    }

    FFCommandExecutor dropFreihtCmd = new FFCommandExecutor() {
        @Override
        void execute() {
            freightTool.dropFreight();
        }
    };
    FFCommandExecutor sharedHubCmd = new FFCommandExecutor() {
        @Override
        void execute() {
            freightTool.sharedHub();
        }
    };

    FFCommandExecutor allianceHubCmd = new FFCommandExecutor() {
        @Override
        void execute() {
            freightTool.allianceHub();
        }
    };

    FFCommandExecutor intakeCmd = new FFCommandExecutor() {
        @Override
        void execute() {
            freightTool.intake();
        }
    };

    FFCommandExecutor moveCmd = new FFCommandExecutor() {
        @Override
        void execute() {
            freightTool.move();
        }
    };


}
