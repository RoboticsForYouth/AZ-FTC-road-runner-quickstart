package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import static org.firstinspires.ftc.teamcode.freightFrenzy.tools.FreightInIntakeSensor.DETECTION_DISTANCE_CM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "FreightToolAuto")
public class FreightTool extends LinearOpMode {
    Intake intake;
    Wrist wrist;
    Arm arm;
    TurnTable turnTable;
    LinearOpMode opMode;
    SampleMecanumDrive drive;
    double xPos = 12.0;
    FreightSensor sensor = null;
    FreightInIntakeSensor intakeSensor = null;
    Trajectory move, hub;
    Pose2d pos;

    boolean isFreightToolBusy = false;
    private boolean frieghtDetected;
    private double sensorDistance;

    //Moves freightTool to allianceHub pos
    public void allianceHub() {
        intake.stopIntake();
        setAllianceHubDrop();
    }

    //moves freightTool to sharedHub pos
    public void sharedHub() {
        intake.stopIntake();
        arm.moveToLevel(Arm.ArmLevel.LEVEL1);
    }

    public void dropFreight() {
        intake.drop();
    }

    public void tapeDrivePos() {
        intake.stopIntake();
        turnTable.turnTo0();
        arm.moveToLevel(Arm.ArmLevel.TAPE_DRIVE);
    }

    public void encTurnLeft() {
        turnTable.turnInc(TurnTable.Direction.COUNTER_CLOCKWISE);
    }

    public void encTurnRight() {
        turnTable.turnInc(TurnTable.Direction.CLOCKWISE);
    }

    public void encMoveUp() {
        arm.moveUp();
    }

    public void encMoveDown() {
        arm.moveDown();
    }

    public void dropFreightTeleOp() {
        intake.dropInstant();
    }


    private enum ToolState {
        HOME, SHARED_HUB_DROP, ALLIANCE_HUB_DROP, INTAKE, MOVE, DROP, TAPE_DRIVE
    }

    ToolState toolState = ToolState.HOME;

    public void setup() {
        arm = new Arm(opMode);
        turnTable = new TurnTable(opMode, arm);
        intake = new Intake(opMode);
        wrist = new Wrist(opMode);
        sensor = new FreightSensor(opMode);
//        intakeSensor = new FreightInIntakeSensor(opMode);
    }

    public void setupPos() {
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

    public void prepForDrop(Arm.ArmLevel level) {
        intake.stopIntake();
        arm.moveToMinLevel(level);
        // wrist.setSecuredPos();
        //turnTable.turnToPos(280, 0.2);
    }

    public void intake() {
        intakeWithAngle(0);
    }

    //stop intake when freight detected
    public void intakeWithSensor(int deg){
//        AZUtil.runInParallelPool(
//                () -> {
//                    int tryCount =0;
//                    double sensorDistance = intakeSensor.getSensorDistance();
//                    //try for max 5 secs
//                    while (sensorDistance > DETECTION_DISTANCE_CM && tryCount < 100) {
//                        sensorDistance = intakeSensor.getSensorDistance();
////                        AZUtil.print(telemetry,"Intake Sensor (cm):", sensorDistance);
//                        sleep(50);
//                        tryCount++;
//                    }
//                    intake.stopIntake();
//                });
        intakeWithAngle(deg);
    }

    public void intakeWithAngle(int deg) {
        intake.intake();
        wrist.intakePos();
        arm.moveToLevel(Arm.ArmLevel.INTAKE);
        turnTable.turnTo(TurnTable.Direction.CLOCKWISE, deg, false);
    }

    public void move() {
        wrist.setSecuredPos();
        intake.stopIntake();
        turnTable.turnTo0();
        arm.moveToLevel(Arm.ArmLevel.MOVE);
    }

    public void drop() {
        intake.drop();
        //sleep(1000);
    }

    public void reset() {
        turnTable.turnToPos(0, 0.1);
        intake.stopIntake();
        wrist.homePos();
        sleep(1000);
        arm.moveToMinLevel(Arm.ArmLevel.HOME);
    }


    public synchronized void prepSharedHubDrop() {
        isFreightToolBusy = true;
        setShareHubDropPos();
        turnTable.turnTo(TurnTable.Direction.CLOCKWISE, 100, false);
        //waitUntilMotorBusy();

        isFreightToolBusy = false;
    }

    public void setShareHubDropPos() {
        intake.stopIntake();
        wrist.setLevel1DropPos();
        arm.moveToMinLevel(Arm.ArmLevel.LEVEL1);
    }

    public void setLevel2HubDropPos(){
        intake.stopIntake();
        wrist.setLevel2DropPos();
        arm.moveToMinLevel(Arm.ArmLevel.LEVEL2);
    }

    public synchronized void prepAllianceHubDrop() {
        isFreightToolBusy = true;
        setAllianceHubDrop();
//        turnTable.turnTo(TurnTable.Direction.CLOCKWISE, 115);
        waitUntilMotorBusy();
        drop();
        isFreightToolBusy = false;
    }

    public void setAllianceHubDrop() {
        wrist.setLevel3DropPos();
        intake.stopIntake();
        arm.moveToLevel(Arm.ArmLevel.LEVEL3);
    }

    public synchronized void moveTo0() {
        isFreightToolBusy = true;

        turnTable.turnTo0();
        arm.moveTo0();
        isFreightToolBusy = false;
    }

    private void waitUntilMotorBusy() {
        while ((arm.isBusy() || turnTable.isBusy()) && opModeIsActive()) {
            sleep(100);
        }
    }

    public void waitUntilBusy() {
        while (isFreightToolBusy) {
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

    public void autoIntake(int deg) {
        //double distance = sensor.getSensorDistance();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry telemetry;

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        //AZUtil.print(telemetry, "Object distance: ", distance);

        pos = new Pose2d(0, 0);
        intakeWithAngle(deg);
        //sensorDistance = sensor.getSensorDistance();
        intakeSensor.startDetection();

        // sensor.detectFreight();
        move = drive.trajectoryBuilder(pos)
                //.forward(distance1)
                .lineToSplineHeading(new Pose2d(xPos, 0, 0))
                .build();
        drive.followTrajectory(move);

        pos = move.end();


        int count = 0;
        while (!intakeSensor.isFreightDetected() && opModeIsActive() && count < 7) {
            //sleep(1000);
            double distance1 = 3.0;

            if (count == 0 || count == 3 || count == 6) {
                // turnTable.turnTo(TurnTable.Direction.CLOCKWISE, deg - 5, false);
                move = drive.trajectoryBuilder(pos)
                        .lineToSplineHeading(new Pose2d(xPos + distance1, 0, 0))
                        .build();
            } else if (count == 1 || count == 4 || count == 7) {
                // turnTable.turnTo(TurnTable.Direction.COUNTER_CLOCKWISE, deg - 5, false);
                move = drive.trajectoryBuilder(pos)
                        .lineToSplineHeading(new Pose2d(xPos + distance1, 0, 0))
                        .build();
            } else {
                // turnTable.turnTo(TurnTable.Direction.CLOCKWISE,deg, false);
                move = drive.trajectoryBuilder(pos)
                        .lineToSplineHeading(new Pose2d(xPos + distance1, 0, 0))
                        .build();
            }

            drive.followTrajectory(move);
            AZUtil.print(telemetry, "Pos", drive.getPoseEstimate());
            pos = move.end();
            xPos += distance1;
            count++;
        }

        intakeSensor.stopDetection();
        sleep(1000);
        stopIntake();

        hub = drive.trajectoryBuilder(move.end())
                .lineToSplineHeading(new Pose2d(-29, 0, 0))
                .build();
        AZUtil.runInParallelPool(
                () -> {
                    prepSharedHubDrop();

                }
        );
        try {
            drive.followTrajectory(hub);

        } catch (Exception e) {

        }
        sleep(500);
    }

     /*

        AZUtil.print(telemetry, "Starting intake", "");

        //intake();
        sensorDistance = sensor.getSensorDistance();
        intakeSensor.startDetection();
        move2 = drive.trajectoryBuilder(pos)
                .forward((double) (sensorDistance/2.54)+1)
                .build();


        drive.followTrajectory(move2);
        sleep(1000);
        int tryCount = 0;
        while (!intakeSensor.isFreightDetected() && tryCount < 3) {
            if (tryCount == 0) {
                turnTable.turnTo(TurnTable.Direction.CLOCKWISE, 10, false);
            } else if (tryCount == 1) {
                turnTable.turnTo(TurnTable.Direction.COUNTER_CLOCKWISE, 10, false);
            } else {
                turnTable.turnTo(TurnTable.Direction.CLOCKWISE,0, false);
            }
            sleep(1000);
            tryCount++;
        }

        pos = move2.end();
        */

    public void stopIntake() {
        intake.stopIntake();
    }


    public void dropSharedHub(Pose2d pose) {
        drop();
        sleep(1000);
//        waitUntilBusy();
        AZUtil.runInParallelPool(
                () -> {
                    turnTable.turnTo(TurnTable.Direction.COUNTER_CLOCKWISE, 0, false);
                }
        );
        Trajectory home = drive.trajectoryBuilder(hub.end()).
                forward(29)
                .build();
        drive.followTrajectory(home);
//        waitUntilMotorBusy();


    }

    @Override
    public void runOpMode() {
        setup();
        drive = new SampleMecanumDrive(opMode.hardwareMap);


        waitForStart();
        testAutoIntakeAndDropSharedHub();
        sleep(5000);

        //testHome();
        sleep(10000);

    }

    private void testFreightIntakeTest() {
        intake();
        sleep(4000);
        prepAllianceHubDrop();
    }

    private void testAutoIntakeAndDropSharedHub() {
        for (int i = 0; i < 25; i++) {
            int deg = 0;
            if (i % 2 == 0) {
                deg = 10;
                if (i % 4 == 0) {
                    //left side for red detect wall if set to 15 deg.
                    deg = -10;
                }
            }
            autoIntake(deg);
            dropSharedHub(new Pose2d(0, 0));
            sleep(100);
        }
        sleep(2000);
    }

    public void turnInc(TurnTable.Direction direction) {
        turnTable.turnInc(direction);
    }

    private void testLevel3Drop() {
        prepAllianceHubDrop();
    }

    private void testLevel1Drop() {
        prepSharedHubDrop();
    }

    private void testHome() {
        reset();
    }

    private void testIntake() {
        //The arm is at raised position for Intake
        //wrist is intake position slightly facing down
        intake();
    }
}