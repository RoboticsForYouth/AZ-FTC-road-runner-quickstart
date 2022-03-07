package org.firstinspires.ftc.teamcode.freightFrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.Detection;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class OLDBlueAutoBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive;
    Detection cam;
    String pos;
    Servo claw;
    DcMotor lift;
    DcMotor arm;
    CRServo duckyTool;
    int originX = 0;
    int originY = 0;
    int originHeading = 0;
    Trajectory drop, duck, park, move, back;
    LinearOpMode linearOpMode;

    private static final double SCOOP_READY = 0.35;
    private static final double SCOOP_SECURED = 0.50;
    private static final double SCOOP_SECURED_MORE = 0.75;
    private static final double SCOOP_DROP = 0;
    private static final double SCOOP_INIT = 0.27;
    private static final int LIFT_DOWN_POS = 150;
    private static final int LIFT_DROP_POS = 0;
    private static final int LIFT_UP_POS = 2400;
    private static final double LIFT_POWER = 1.0;
    private static final int ARM_DOWN_POS = -10;
    private static final int ARM_INTAKE = -35;
    private static final int ARM_LIFT_READY = 20;
    private static final int ARM_UP_POS = -300;
    private static final int ARM_MID_POS = -175;
    private static final double ARM_POWER = 0.35;
    private static final double DUCK_POWER = 0.3;

    public void initAuto() {

        drive = new SampleMecanumDrive(hardwareMap);
        cam = new Detection(this);
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
        duckyTool = hardwareMap.get(CRServo.class, "duckyTool");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    public void detection() {
        //pos = cam.getPos();
        pos = "right";
        telemetry.addData("Pos=", pos);
    }

    public void moveToDrop(boolean right) {
        claw.setPosition(SCOOP_SECURED);
        if (right) {
            drop = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX + 27, originY + 15, Math.toRadians(45))).build();
            drive.followTrajectory(drop);
            dropBlock(pos);
        } else {
            drop = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX + 18, originY - 30, Math.toRadians(-30))).build();
            drive.followTrajectory(drop);
            dropBlock(pos);

        }
    }

    public void carousel(boolean right) {
        if (right) {
            duck = drive.trajectoryBuilder(drop.end())
                    .lineToSplineHeading(new Pose2d(originX - 2, originY - 45, Math.toRadians(-60))).build();
            drive.followTrajectory(duck);
            duckyTool.setPower(DUCK_POWER);
            sleep(3500);
            duckyTool.setPower(0);
        } else {
            duck = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(72, 0), Math.toRadians(90)).build();
            drive.followTrajectory(duck);
        }
    }

    public void park(boolean right, boolean warehouse) {
        if (right) {
            if (warehouse) {
                //??
            } else {
                park = drive.trajectoryBuilder(duck.end())
                        .lineToSplineHeading(new Pose2d(originX + 40, originY - 54, Math.toRadians(-100))).build();
                drive.followTrajectory(park);
            }
        } else {
            if (warehouse) {
                park = drive.trajectoryBuilder(drop.end())
                        .lineToSplineHeading(new Pose2d(originX - 7, originY - 7, Math.toRadians(-110))).build();
                Trajectory park2 = drive.trajectoryBuilder(park.end())
                        .lineToSplineHeading(new Pose2d(originX - 7, originY + 36, Math.toRadians(-90))).build();
                drive.followTrajectory(park);
                drive.followTrajectory(park2);
                setLiftPos(LIFT_DROP_POS);
            } else {
                //??
            }

        }
    }

    private void setArmPos(int pos) {
        arm.setPower(ARM_POWER);
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // waitUntilMotorBusy(arm);
        waitUntilMotorPos(arm, pos);

    }

    private void setLiftPos(int pos) {
        lift.setPower(LIFT_POWER);
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitUntilMotorPos(lift, pos);
        // sleep(1000);
    }

    private void waitUntilMotorBusy(DcMotor motor) {
        runtime.reset();
        while (!opModeIsActive() && motor.isBusy()) {
            sleep(100);
        }

    }

    private void waitUntilMotorPos(DcMotor motor, int targetPos) {
        int diff = Math.abs(motor.getCurrentPosition() - targetPos);
        while (!opModeIsActive() && motor.isBusy()) {
            sleep(100);
        }

        while (runtime.milliseconds() <= diff) {
            sleep(100);
        }
    }

    private void dropBlock(String pos) {
        claw.setPosition(SCOOP_SECURED);
        sleep(500);
        setArmPos(ARM_LIFT_READY);
        setLiftPos(LIFT_UP_POS);
        sleep(1000);
        claw.setPosition(SCOOP_SECURED_MORE);
        sleep(1000);

        if (pos.equals("right")) {
            //highest
            setArmPos(ARM_UP_POS);
            move = drive.trajectoryBuilder(drop.end())
                    .forward(14).build();
            drive.followTrajectory(move);
            claw.setPosition(SCOOP_DROP);
            sleep(1500);
            back = drive.trajectoryBuilder(move.end())
                    .back(14).build();


        } else if (pos.equals("center")) {
            setArmPos(ARM_UP_POS);
            setLiftPos(LIFT_DOWN_POS);
            sleep(750);
            move = drive.trajectoryBuilder(drop.end())
                    .forward(12).build();
            drive.followTrajectory(move);
            claw.setPosition(SCOOP_DROP);
            sleep(1500);
            back = drive.trajectoryBuilder(move.end())
                    .back(12).build();

        } else {
            setArmPos(ARM_MID_POS);
            setLiftPos(LIFT_DOWN_POS);
            sleep(750);
            move = drive.trajectoryBuilder(drop.end())
                    .forward(10).build();
            drive.followTrajectory(move);
            claw.setPosition(SCOOP_DROP);
            sleep(500);
            //setArmPos(-160);
            back = drive.trajectoryBuilder(move.end())
                    .back(10).build();
        }
        drive.followTrajectory(back);
        runInParallel(
                new Runnable() {
                    public void run() {
                        claw.setPosition(SCOOP_SECURED_MORE);
                        setLiftPos(LIFT_UP_POS);
                        sleep(750);
                        setArmPos(ARM_DOWN_POS);
                        sleep(750);
                        claw.setPosition(SCOOP_INIT);
                        sleep(750);
                        setLiftPos(LIFT_DOWN_POS);
                    }
                }
        );
    }

    public void runInParallel(Runnable r) {
        new Thread(r).start();
    }

}
