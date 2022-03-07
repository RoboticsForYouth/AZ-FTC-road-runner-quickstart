package org.firstinspires.ftc.teamcode.freightFrenzy.tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class Detection {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AdZuoYX/////AAABmbOYHuyjI04Hui2uXX61XshITK5fA7mXt0lOltN+1mgtofyO+CIW8xBdPp77dao7VktgN1gogSSE89qjponLTUOaPo+vaw3zN8TNuozhrF/T/AqfS6HSunynzusMnEBZmIVMIn0lHZxOoGH3yE0Y0BAXHtyPlbdZhhWgtr23wlPEzBz19KljAo1UrKjq5ztFvLcq+je43PkqXECs16R75/Dhxipt350BONTCDRSKAdaXpNkPUPk2aAJ58e5UPjHv9v5Hjn2syBOKqwoAzBDQrdpmNYmhHCYRzghIDCrOwR8Iipf1esG6vRYCF5dOaVlFUaKvsqMZMqOSJ+lEPLOG7EIJmbjhu8ZKkHR+HCfZADNO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    LinearOpMode linearOpMode;
    String pos;


    public Detection(LinearOpMode opMode) {
        this.linearOpMode = opMode;
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            // tfod.setZoom(1.15, 16.0/9);
        }
    }

    public boolean isThere() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        for (int i = 0; i < 5; i++) {
            if (updatedRecognitions != null)
                return true;
        }
        return false;
    }

    public String getLevel() {
        String pos = "left";
        linearOpMode.sleep(500);
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            int count = 0;
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            // updatedRecognitions = tfod.getUpdatedRecognitions();

//            while(!linearOpMode.isStopRequested() && !(updatedRecognitions == null ||
//                    updatedRecognitions.size() <=0) && count < 6) {
//                updatedRecognitions = tfod.getUpdatedRecognitions();
//                //linearOpMode.sleep(1000);
//                count++;
//            }
            if (updatedRecognitions == null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }
            linearOpMode.telemetry.addData("updatedRecognitions", updatedRecognitions);
            if (updatedRecognitions != null) {
                //boolean detected = false;
                for (Recognition recognition : updatedRecognitions) {
                    double avg = (recognition.getLeft() + recognition.getRight()) / 2;
                    if (avg < 450) {
                        pos = "center";
                    } else if (avg >= 450) {
                        pos = "right";
                    }

                    linearOpMode.telemetry.addData("Right", recognition.getRight());
                    linearOpMode.telemetry.addData("Left", recognition.getLeft());
                    linearOpMode.telemetry.addData("AVG", avg);
                    linearOpMode.telemetry.update();
                    linearOpMode.sleep(2000);

                }

                if (tfod != null) {
                    tfod.shutdown();
                }

            } else {
                pos = "left";
                linearOpMode.telemetry.addData("updatedRecognitions :", "null");
                linearOpMode.telemetry.update();
            }
            linearOpMode.telemetry.addData("Position", pos);
            linearOpMode.telemetry.update();
            linearOpMode.sleep(200);


        } else {
            linearOpMode.telemetry.addData("tfod = ", "null");
            linearOpMode.telemetry.update();
        }
        return pos;
    }

    public String getPos() {
        if (tfod != null) {
            int count = 0;
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions == null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }
            linearOpMode.telemetry.addData("updatedRecognitions", updatedRecognitions);
            if (updatedRecognitions != null) {
                //boolean detected = false;
                for (Recognition recognition : updatedRecognitions) {
                    double avg = (recognition.getLeft() + recognition.getRight()) / 2;

                    linearOpMode.telemetry.addData("AVG", avg);
                    linearOpMode.telemetry.update();
                    linearOpMode.sleep(4000);

                }

                if (tfod != null) {
                    tfod.shutdown();
                }

            }

        } else {
            linearOpMode.telemetry.addData("tfod = ", "null");
            linearOpMode.telemetry.update();
        }
        return "";
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = linearOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = linearOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", linearOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}