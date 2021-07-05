package org.firstinspires.ftc.teamcode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


public class RingDetector {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD = "Quad";
    private static final String SINGLE = "Single";
    private static final String ZERO = "Zero";
    private static final String VUFORIA_KEY =
            "AdZuoYX/////AAABmbOYHuyjI04Hui2uXX61XshITK5fA7mXt0lOltN+1mgtofyO+CIW8xBdPp77dao7VktgN1gogSSE89qjponLTUOaPo+vaw3zN8TNuozhrF/T/AqfS6HSunynzusMnEBZmIVMIn0lHZxOoGH3yE0Y0BAXHtyPlbdZhhWgtr23wlPEzBz19KljAo1UrKjq5ztFvLcq+je43PkqXECs16R75/Dhxipt350BONTCDRSKAdaXpNkPUPk2aAJ58e5UPjHv9v5Hjn2syBOKqwoAzBDQrdpmNYmhHCYRzghIDCrOwR8Iipf1esG6vRYCF5dOaVlFUaKvsqMZMqOSJ+lEPLOG7EIJmbjhu8ZKkHR+HCfZADNO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public String camera;
    LinearOpMode linearOpMode;
    
    public RingDetector(LinearOpMode opMode, String camera) {
        this.linearOpMode = opMode;
        this.camera = camera;
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

    }
    
    private String numRings() {
        String num = ZERO;
        linearOpMode.sleep(500);
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              linearOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
              // step through the list of recognitions and display boundary info.
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                // linearOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                // linearOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                //         recognition.getLeft(), recognition.getTop());
                // linearOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                //         recognition.getRight(), recognition.getBottom());
                // linearOpMode.telemetry.update();
                num = recognition.getLabel();
                //linearOpMode.sleep(3000);
                }
                
            }
        }
        return num;
    }
    public String multiDetect() {
        int countZ = 0;
        int countS = 0;
        int countQ = 0;
        for(int n = 0; n<=4; n++) {
                String numR = numRings();
                if(numR.equals(QUAD) && linearOpMode.opModeIsActive()) {
                    countQ++;
                }
                else if(numR.equals(SINGLE) && linearOpMode.opModeIsActive()) {
                    countS++;
                }
                else {
                    countZ++;
                }
            
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        // linearOpMode.telemetry.addData("CountQ", countQ);
        // linearOpMode.telemetry.addData("CountS", countS);
        // linearOpMode.telemetry.addData("CountZ", countZ);
        // linearOpMode.telemetry.update();
        // linearOpMode.sleep(1000);
        
        if(countQ > countS && linearOpMode.opModeIsActive()) {
            return QUAD;
        }
        if(countS > countQ  && linearOpMode.opModeIsActive()) {
            return SINGLE;
        }
        if(countZ > countQ && countZ > countS  && linearOpMode.opModeIsActive()) {
            return ZERO;
        }
        else {
            return QUAD;
        }
    }
    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = linearOpMode.hardwareMap.get(WebcamName.class, camera);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = linearOpMode.hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", linearOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD, SINGLE);
    }
    
    
    
    
}