package org.firstinspires.ftc.teamcode.powerplay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.freightFrenzy.tools.AZUtil;
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
    private Vector2d backUp;

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
        Pose2d dropConePos = new Pose2d(16, 17);
        dropConeTrajectory = drive.trajectoryBuilder(origin)
                .lineToSplineHeading(dropConePos)
                .build();

        Vector2d backUp = new Vector2d(13, 18);
        createConePosTrajectory(backUp);


    }
    public void setUpRightSideTrajectory(){
        origin = new Pose2d(8, 40, Math.toRadians(0));
        drive.setPoseEstimate(origin);
        Pose2d dropConePos = new Pose2d(16, 50);
        dropConeTrajectory = drive.trajectoryBuilder(origin)
                .lineToSplineHeading(dropConePos)
                .build();

        Vector2d backUp = new Vector2d(13, 48);
        Vector2d conePos1Step1 = new Vector2d(10, 64);
        Vector2d conePos1Step2 = new Vector2d(35, 64);
        conePos1Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .back(4)
               //.splineToConstantHeading(backUp, Math.toRadians(45))
                .splineToConstantHeading(conePos1Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos1Step2, Math.toRadians(0))
                .build();

        Vector2d conePos2Step1 = new Vector2d(10, 40);
        Vector2d conePos2Step2 = new Vector2d(45, 40);
        Vector2d conePos2Step3 = new Vector2d(39, 40);
        conePos2Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .back(4)
                //.splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step1, Math.toRadians(45))
                .splineToConstantHeading(conePos2Step2, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step3, Math.toRadians(0))
                .build();

        Vector2d conePos3Step1 = new Vector2d(10, 0);
        Vector2d conePos3Step2 = new Vector2d(30, 0);
        conePos3Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .back(4)
                //.splineToConstantHeading(backUp, Math.toRadians(45))
                .splineToConstantHeading(conePos3Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos3Step2, Math.toRadians(0))
                .build();

    }
    private void createConePosTrajectory(Vector2d backUp) {
        Vector2d conePos1Step1 = new Vector2d(10, 60);
        Vector2d conePos1Step2 = new Vector2d(35, 60);
        conePos1Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .back(4)
                //.splineToConstantHeading(backUp, Math.toRadians(45))
                .splineToConstantHeading(conePos1Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos1Step2, Math.toRadians(0))
                .build();

        Vector2d conePos2Step1 = new Vector2d(10, 30);
        Vector2d conePos2Step2 = new Vector2d(45, 30);
        Vector2d conePos2Step3 = new Vector2d(39, 30);
        conePos2Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .back(4)
                //.splineToConstantHeading(backUp, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step1, Math.toRadians(45))
                .splineToConstantHeading(conePos2Step2, Math.toRadians(0))
                .splineToConstantHeading(conePos2Step3, Math.toRadians(0))
                .build();

        Vector2d conePos3Step1 = new Vector2d(10, 6);
        Vector2d conePos3Step2 = new Vector2d(30, 6);
        conePos3Trajectory = drive.trajectoryBuilder(dropConeTrajectory.end())
                .back(4)
                //.splineToConstantHeading(backUp, Math.toRadians(45))
                .splineToConstantHeading(conePos3Step1, Math.toRadians(0))
                .splineToConstantHeading(conePos3Step2, Math.toRadians(0))
                .build();
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
        telemetry.addData("Trajectory", "Created");
        telemetry.update();
        //detect sleeve pos to verify alignment.Need to run it for a few times before
        //the sleeve is accurately detected
        for(int i=0; i<3; i++) {
            getSleevePos();
        }

        waitForStart();

        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                coneTool.grabCone();
            }
        });
        getSleevePos();
        sleep(1000);

        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                coneTool.liftTo(Lift.LiftLevel.LOW);
            }
        });
        sleep(1000);
        drive.followTrajectory(dropConeTrajectory);

        sleep(1000);
        coneTool.dropCone();
        sleep(1000);
        AZUtil.runInParallel(new Runnable() {
            @Override
            public void run() {
                coneTool.liftTo(Lift.LiftLevel.ZERO);
            }
        });
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

        //TODO: need to accurately determine the time to ensure < 30 secs
        sleep(5000);
    }

    private void getSleevePos() {
        pos = sleeveDetection.getPos();
        telemetry.addData("Pos", pos);
        int[] avg = sleeveDetection.getAvgs();
        telemetry.addData("Red: ", avg[0]);
        telemetry.addData("Green: ", avg[1]);
        telemetry.addData("Blue: ", avg[2]);
        telemetry.update();
    }
}
