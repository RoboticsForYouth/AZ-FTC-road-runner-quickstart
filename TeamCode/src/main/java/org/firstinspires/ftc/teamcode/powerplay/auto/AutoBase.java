package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.powerplay.tools.ConeTool;
import org.firstinspires.ftc.teamcode.powerplay.tools.Lift;

@Autonomous
public class AutoBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ConeTool coneTool;
    SampleMecanumDrive drive;
    SleeveDetection sleeveDetection;
    int pos;
    Trajectory dropCone, dropCone2, dropCone3, park;
    Pose2d origin;
    public enum FieldPos {
        LEFT,
        RIGHT
    }

    FieldPos fieldPos = FieldPos.LEFT;

    int originX = 0;
    int originY = 0;
    int originHeading = 0;
    private Trajectory dropConeTrajectory;
    private Trajectory conePos1Trajectory;
    private Trajectory conePos2Trajectory;
    private Trajectory conePos3Trajectory;

    public void initAuto() {
        coneTool = new ConeTool(this);
        sleeveDetection = new SleeveDetection(this);
        sleeveDetection.setup();
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }


    public void setupPos() {
       coneTool.grabCone();
    }

    public void setFieldPos(FieldPos fieldPos){
        this.fieldPos = fieldPos;
    }

    public void detection() {
        pos = sleeveDetection.getPos();
        telemetry.addData("Position", pos);
        telemetry.update();
    }

    public void setUpLeftSideTrajectory(){
        origin = new Pose2d(8, 40, Math.toRadians(0));
        drive.setPoseEstimate(origin);
        Pose2d dropConePos = new Pose2d(16, 12);
        dropConeTrajectory = drive.trajectoryBuilder(origin)
                .lineToSplineHeading(dropConePos)
                .build();

        Vector2d backUp = new Vector2d(8, 12);
        Vector2d conePos1Step1 = new Vector2d(10, 60);
        Vector2d conePos1Step2 = new Vector2d(40, 60);
        conePos1Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos1Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos1Step2, Math.toRadians(0))
                .build();

        Vector2d conePos2Step1 = new Vector2d(10, 30);
        Vector2d conePos2Step2 = new Vector2d(40, 30);
        conePos2Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step2, Math.toRadians(0))
                .build();

      Vector2d conePos3Step1 = new Vector2d(10, 0);
      Vector2d conePos3Step2 = new Vector2d(40, 0);
        conePos3Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos3Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos3Step2, Math.toRadians(0))
                .build();

    }

    public void setUpRightSideTrajectory(){
        origin = new Pose2d(8, -40, Math.toRadians(0));
        drive.setPoseEstimate(origin);
        Pose2d dropConePos = new Pose2d(16, -12);
        dropConeTrajectory = drive.trajectoryBuilder(origin)
                .lineToSplineHeading(dropConePos)
                .build();

        Vector2d backUp = new Vector2d(8, -12);
        Vector2d conePos1Step1 = new Vector2d(10, -60);
        Vector2d conePos1Step2 = new Vector2d(40, -60);
        conePos1Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos1Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos1Step2, Math.toRadians(0))
                .build();

        Vector2d conePos2Step1 = new Vector2d(10, -30);
        Vector2d conePos2Step2 = new Vector2d(40, -30);
        conePos2Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step2, Math.toRadians(0))
                .build();

        Vector2d conePos3Step1 = new Vector2d(10, 0);
        Vector2d conePos3Step2 = new Vector2d(40, 0);
        conePos3Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos3Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos3Step2, Math.toRadians(0))
                .build();

    }

    public void dropCone(boolean left) {
        if(left) {
            dropCone = drive.trajectoryBuilder(origin)
                    .lineToSplineHeading(new Pose2d(origin.getX(), origin.getY() + 32, origin.getHeading())).build();
            dropCone2 = drive.trajectoryBuilder(dropCone.end())
                    .lineToSplineHeading(new Pose2d(origin.getX() + 50, origin.getY() + 34, origin.getHeading()-45)).build();
        } else {
            dropCone = drive.trajectoryBuilder(new Pose2d(originX, originY, Math.toRadians(originHeading)))
                    .lineToSplineHeading(new Pose2d(originX + 30, originY, originHeading+90)).build();
            dropCone2 = drive.trajectoryBuilder(dropCone.end())
                    .lineToSplineHeading(new Pose2d(originX + 42, originY - 24, originHeading+180)).build();
        }
//        AZUtil.runInParallel(
//                new Runnable() {
//                    @Override
//                    public void run() {
//                        sleep(500);
//                        coneTool.liftTo(Lift.LiftLevel.HIGH);
//
//                    }
//                }
//        );
        drive.followTrajectory(dropCone);
        drive.followTrajectory(dropCone2);
        sleep(1000);
        coneTool.dropCone();
//        AZUtil.runInParallel(
//                new Runnable() {
//                    @Override
//                    public void run() {
//                        sleep(1000);
//                        coneTool.liftTo(Lift.LiftLevel.ZERO);
//
//                    }
//                }
//        );

    }

    public void park(int pos, boolean left) {
        if(left) {

            if(pos == 1) {

            } else if(pos == 2) {

            } else {

            }

        } else {

            if(pos == 1) {

            } else if(pos == 2) {

            } else {

            }

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();
        switch ( fieldPos ){
            case LEFT:
                setUpLeftSideTrajectory();
                break;
            case RIGHT:
                setUpRightSideTrajectory();
                break;
        }
        waitForStart();
        //Uncomment after navigation testing is complete
        coneTool.grabCone();
        sleep(1000);
        pos = sleeveDetection.getPos();
        //Uncomment after navigation testing is complete
        coneTool.liftTo(Lift.LiftLevel.LOW);
        sleep(500);
        drive.followTrajectory(dropConeTrajectory);
        telemetry.addData("Position", pos);
        telemetry.update();

//Uncomment after navigation testing is complete
        sleep(1000);
        coneTool.dropCone();
        sleep(1000);

        //go back to origin
//        drive.followTrajectory(drive.trajectoryBuilder(dropConeTrajectory.end())
//                .lineToSplineHeading(origin).build());

        switch (pos){
            case 1:
                drive.followTrajectory(conePos1Trajectory);
                break;
            case 2:
                drive.followTrajectory(conePos2Trajectory);
                break;
            case 3:
                drive.followTrajectory(conePos3Trajectory);
                break;
        }
        //Uncomment after navigation testing is complete
        coneTool.liftTo(Lift.LiftLevel.ZERO);
        sleep(5000);
    }
}
