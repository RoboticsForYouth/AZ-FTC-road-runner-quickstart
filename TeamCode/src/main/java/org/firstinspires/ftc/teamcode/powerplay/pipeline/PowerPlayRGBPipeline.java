
package org.firstinspires.ftc.teamcode.powerplay.pipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PowerPlayRGBPipeline extends OpenCvPipeline {

    /*
     * An enum to define the skystone position
     */
    public enum ConePosition
    {
        ONE,
        TWO,
        THREE
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(225, 0, 0);
    static final Scalar TEAL = new Scalar(50, 150, 150);

    /*
     * The core values which define the location and size of the sample regions
     */

     Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(145,120);

    public int[] getAvg() {
        return new int[]{avgRed, avgGreen, avgBlue };
    }

    int REGION_WIDTH = 40;
    int REGION_HEIGHT = 65;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /* Working variables
         */
    Mat region1_Cb, region1_Cr, region1_Cg;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat Cr = new Mat();
    Mat Cg = new Mat();
    int avgBlue;
    int avgRed;
    int avgGreen;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile int position = 1;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {

//        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(input, Cr, 0);
        Core.extractChannel(input, Cg, 1);
        Core.extractChannel(input, Cb, 2);
    }

    public void setBoundingBox(Point topLeft, int width, int height){
        REGION1_TOPLEFT_ANCHOR_POINT.x = topLeft.x;
        REGION1_TOPLEFT_ANCHOR_POINT.y = topLeft.y;
        REGION_WIDTH = width;
        REGION_HEIGHT = height;
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        region1_Cg = Cg.submat(new Rect(region1_pointA, region1_pointB));

    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avgBlue = (int) Core.mean(region1_Cb).val[0];
        avgGreen = (int) Core.mean(region1_Cg).val[0];
        avgRed = (int) Core.mean(region1_Cr).val[0];


       // avg2 = (int) Core.mean(region2_Cb).val[0];
//        avg3 = (int) Core.mean(region3_Cb).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                5); // Thickness of the rectangle lines
        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
//

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
//

        System.out.println(avgBlue);
        System.out.println(avgGreen);
        System.out.println(avgRed);

        position = 1;
        Scalar borderColor = BLUE;

        if( avgGreen > avgBlue && avgGreen > avgRed){
            position = 2;
            borderColor = GREEN;
        } else if(avgRed > avgBlue && avgRed > avgGreen) {
            position = 3;
            borderColor = RED;
        }


            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    5); // Negative thickness means solid fill

        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public int getAnalysis()
    {
        return position;
    }
}

