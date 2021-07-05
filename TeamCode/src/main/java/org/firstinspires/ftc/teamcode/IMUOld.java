package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Func;


public class IMUOld {
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle,  correction, startAngle;
    LinearOpMode opMode;
    Telemetry telemetry;
    Base base;
    
    public static int RIGHT = 0;
    public static int LEFT = 1;
    
    public IMUOld(LinearOpMode opMode, Base base ){
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        this.base = base;
        init(opMode);
    }
    
    public void init(LinearOpMode opMode){
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())
        {
            opMode.sleep(1000);
            opMode.idle();
            telemetry.addData("Stop Requested", opMode.isStopRequested());
            telemetry.addData("Is calibrated", imu.isGyroCalibrated());
            telemetry.update();
        }
        
        getAngle();
        startAngle = lastAngles.firstAngle;
       
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        printPos();
    }

    public void printPos(){
        getAngle();
       
    
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2nd angle", lastAngles.secondAngle);
        telemetry.addData("3rd angle", lastAngles.thirdAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("4 actual angle", calcDesiredAngle((int)lastAngles.firstAngle));
        telemetry.update();
        
    }
    
    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle_old()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        
        if(angle <= 0) {
            angle = 180 + (180 + angle);
        }
        
        return angle;
    }
    
    public int getTurnDirection(double currentAngle, double destinationAngle) {
        double right, left = 0;
        if(currentAngle > destinationAngle) {
            right = currentAngle - destinationAngle;
            left = 360 - right;
        }
        else {
            left = destinationAngle - currentAngle;
            right = 360 - left;
        }
        if(right<left){
            return RIGHT;
        }
        return LEFT;
    }
    
    public void rotateToEC(double destinationAngle, double power, double threshold)
    {
        rotateTo(destinationAngle, power, threshold);
        rotateTo(destinationAngle, power/*/2*/, threshold/*/3*/);
    }
    
    public void rotateTo(double destinationAngle, double power, double threshold)
    {
        if( opMode.opModeIsActive()){
            double currentAngle = getAngle();
            String direction = "";
            
            if (getTurnDirection(currentAngle, destinationAngle) == 0)
            {   // turn right.
                base.turnRight(power);
                direction = "right";
            }
            else
            {   // turn left.
                base.turnLeft(power);
                direction = "left";
            }
            
            double lower_limit = destinationAngle - threshold;
            double higher_limit = destinationAngle + threshold;
            
            boolean inThreshold = false;
            if(lower_limit < 0){
                lower_limit = 360 + lower_limit;
                
                if((getAngle() < higher_limit && getAngle() > 0) || (getAngle() > lower_limit && getAngle() < 360))
                {
                    inThreshold = true;
                }
                while (opMode.opModeIsActive() && !inThreshold) {
                    if((getAngle() < higher_limit && getAngle() > 0) || (getAngle() > lower_limit && getAngle() < 360))
                    {
                        inThreshold = true;
                    }
                    telemetry.addData("current", getAngle());
                    telemetry.addData("destination", destinationAngle);
                    telemetry.addData("direction", direction);
                    telemetry.update();
            
                }
            
            }
            else if(higher_limit > 360){
                higher_limit = higher_limit - 360;
                
                if((getAngle() < higher_limit && getAngle() > 0) || (getAngle() > lower_limit && getAngle() < 360))
                {
                    inThreshold = true;
                }
                while (opMode.opModeIsActive() && !inThreshold) {
                    if((getAngle() < higher_limit && getAngle() > 0) || (getAngle() > lower_limit && getAngle() < 360))
                    {
                        inThreshold = true;
                    }
                    telemetry.addData("current", getAngle());
                    telemetry.addData("destination", destinationAngle);
                    telemetry.addData("direction", direction);
                    telemetry.update();
                }
            }
            else{
                while (opMode.opModeIsActive() && getAngle() < lower_limit || getAngle() > higher_limit) {
                    telemetry.addData("current", getAngle());
                    telemetry.addData("destination", destinationAngle);
                    telemetry.addData("direction", direction);
                    telemetry.update();
                }
            }
            
            base.stopRobot();
        }
    }
    
    public void rotate(int degrees, int error, double power)
    {

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees < 0)
        {   // turn right.
            base.turnRight(power);
        }
        else if (degrees > 0)
        {   // turn left.
            base.turnLeft(power);
        }
        else return;

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            
            while (opMode.opModeIsActive() && getAngle() == 0) {}
            while (opMode.opModeIsActive() && getAngle() > (degrees-error)) {}
        }
        else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < (degrees-error)) {printPos();}

        // turn the motors off.
        base.stopRobot();
        
        resetAngle();
    }
    
    
    public int calcDesiredAngle(int degrees)
    {
       if(degrees<0)
        {
            int angle = 180 - (int)startAngle;
            int angle2 = 180 + degrees;
            return angle + angle2;
        }
        else{
            return (int)startAngle - degrees;
        }
    
    }
    
    public void rotateAbsolute(int degrees){
            int desiredAngle = calcDesiredAngle(degrees);
            telemetry.addData("Rotating degrees", degrees);
            telemetry.addData("Calculated desired angle", desiredAngle);
            telemetry.update();
            rotate(desiredAngle - (int)lastAngles.firstAngle, 0);
            printPos();        
    }
     
    public double zero(){
        double angle = getAngle();
        rotate(((int)angle), 0, 0.3);
        opMode.telemetry.addData("angle", angle);
        opMode.telemetry.update();
        return angle;
    }
    
    public double zeroReverse(){
        double angle = getAngle();
        rotate(-1*((int)angle), 0, 0.3);
        opMode.telemetry.addData("angle", angle);
        opMode.telemetry.update();
        return angle;
    }
    
    public void rotate(int degrees, int error){
         rotate(degrees, error, 0.6);
    }
    
    
    public void turnLeft90(){
        rotate(90, 18);
    }
    
    public void turnRight90(){
        rotate(-90, -10);
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
