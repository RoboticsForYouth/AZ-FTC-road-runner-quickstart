package org.firstinspires.ftc.teamcode.powerplay.teleOp;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.aztools.AZUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;

@TeleOp
public class RedPowerPlayTeleOp extends LinearOpMode {
    public static final double DEFAULT_DRIVE_FACTOR = 1.25;
    public static final int SLOW_DRIVE_FACTOR = 4;
    SampleMecanumDrive drive;
    ConeTool coneTool;
    boolean manualLiftOp = false;
    private ElapsedTime runtime = new ElapsedTime();
    boolean grabMode = false;
    boolean highMode = false;
    private Lift.LiftLevel[] liftLevels = new Lift.LiftLevel[]{
            Lift.LiftLevel.CONE_2,
            Lift.LiftLevel.ZERO,
            Lift.LiftLevel.CONE_5,
            Lift.LiftLevel.CONE_4,
            Lift.LiftLevel.CONE_3
    };
    private int coneStackCount = 0;
    private boolean coneStackMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        coneTool = new ConeTool(this);
        coneTool.setDropHeight(300);
        coneTool.setConeColor(getConeColor());
        waitForStart();
        grabMode = true;
        while (opModeIsActive()) {
            double driveFactor = DEFAULT_DRIVE_FACTOR;
            if( coneTool.isLiftHigh() | gamepad1.right_bumper) {
                driveFactor = SLOW_DRIVE_FACTOR;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / driveFactor,
                            -gamepad1.left_stick_x / driveFactor,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if(gamepad1.right_bumper){
                coneTool.setConeThreshold();
            }

            if (gamepad1.dpad_right) {

                coneTool.liftTo(Lift.LiftLevel.HIGH);
            } else if (gamepad1.dpad_up) {
                highMode = false;
                coneTool.liftTo(Lift.LiftLevel.MEDIUM);
            } else if (gamepad1.dpad_left) {
                highMode = false;
                coneTool.liftTo(Lift.LiftLevel.LOW); }
            else if (gamepad1.dpad_down) {
                highMode = false;
                coneTool.liftTo(Lift.LiftLevel.CLEAR);
            } else if (gamepad1.left_bumper) {
                highMode = false;
                coneTool.liftTo(Lift.LiftLevel.ZERO);
                AZUtil.runInParallel(new Runnable() {
                    @Override
                    public void run() {
                        sleep(1000);
                        grabMode = true;
                    }
                });
            }
            else if (gamepad1.y) {
                if(!coneStackMode) {
                    runtime.reset();
                    coneStackMode = true;
                    coneTool.liftTo(liftLevels[coneStackCount]);
                    coneStackCount++;
                    if (coneStackCount >= 5) {
                        coneStackCount = 0;
                    }
                }
                if(coneStackMode && runtime.seconds() >= 0.5) {
                    coneStackMode = false;
                }
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
            }

            if( grabMode == true ) {
                if (coneTool.isConeDetected() || gamepad1.b) {
                    AZUtil.runInParallel(new Runnable() {
                        @Override
                        public void run() {
                            coneTool.grabCone();
                        }
                    });
                    grabMode = false;
                }
            }
            if (gamepad1.b) {
                grabMode = true;
            }

            //use for correcting lift position when it stops in auto or
            //robot disconnects
            if(gamepad1.left_trigger > 0){
                manualLiftOp = true;
                coneTool.lowerWithoutEncoder(gamepad1.left_trigger/2);
            }
            if(gamepad1.right_trigger > 0){
                manualLiftOp = true;
                coneTool.raiseWithoutEncoder(gamepad1.right_trigger/2);
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
    public int getConeColor() {
        return Color.RED;
    }
}
