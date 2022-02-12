package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.WebcamExample;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class BarcodeDetector {
    private final HardwareMap hardwareMap;
    private String webcam1Name = "Webcam1";
    private String webcam2Name = "Webcam2";
    private OpenCvWebcam webcam1Cv = null; // Define a first camera (the big one)
    private OpenCvWebcam webcam2Cv = null; // Define a second camera (the small one)
    private SamplePipeline webcam1pipeline = null;
    private SamplePipeline webcam2pipeline = null;

    public BarcodeDetector(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initCameras();
    }

    public LiftPosition getFreightLevel() {
        LiftPosition freightLevel;
        if (webcam1pipeline != null) {
            freightLevel = webcam1pipeline.getPossibleFreightLevel();
            if (freightLevel != null) {
                return freightLevel;
            }
        }
        if (webcam2pipeline != null) {
            freightLevel = webcam2pipeline.getPossibleFreightLevel();
            if (freightLevel != null) {
                return freightLevel;
            }
        }
        // Results are inconclusive, return best option
        return LiftPosition.THIRD;
    }

    private void initCameras() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance() // Splitting the viewport to allow two cameras
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        //Try to find one or more cameras
        WebcamName webcam1 = null;
        WebcamName webcam2 = null;
        try {
            webcam1 = hardwareMap.get(WebcamName.class, webcam1Name);
        }
        catch (Exception e) {
            System.out.println(webcam1Name + " not found");
        }
        try {
            webcam2 = hardwareMap.get(WebcamName.class, webcam2Name);
        }
        catch (Exception e) {
            System.out.println(webcam2Name + " not found");
        }
        if (webcam1 != null) {
            webcam1Cv = OpenCvCameraFactory.getInstance().createWebcam(webcam1, viewportContainerIds[0]);
        }
        if (webcam2 != null) {
            webcam2Cv = OpenCvCameraFactory.getInstance().createWebcam(webcam2, viewportContainerIds[1]);
        }

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        if (webcam1Cv != null) {
            webcam1pipeline = new SamplePipeline("Big Webcam");
            webcam1Cv.setPipeline(webcam1pipeline);
            webcam1Cv.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam1Cv.openCameraDeviceAsync(new MyListener(webcam1Cv));
        }
        if (webcam2Cv != null) {
            webcam2pipeline = new SamplePipeline("Little Webcam");
            webcam2Cv.setPipeline(webcam2pipeline);
            webcam2Cv.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam2Cv.openCameraDeviceAsync(new MyListener(webcam2Cv));
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        private final String name;
        private final double MIN_CONTOUR_AREA = 20;

        private Mat YCrCb = new Mat();
        private Mat Y = new Mat();
        private Mat Cr = new Mat();
        private Mat Cb = new Mat();

        private Mat HSV = new Mat();
        private Mat H = new Mat();
        private Mat S = new Mat();
        private Mat V = new Mat();

        private Mat mask = new Mat();
        private Mat rmask2 = new Mat();
        private Mat hierarchy = new Mat();

        private Point center = new Point();

        private DetectionColor box1detected = DetectionColor.NONE;
        private DetectionColor box2detected = DetectionColor.NONE;
        private DetectionColor box3detected = DetectionColor.NONE;

        public SamplePipeline() {
            this.name = "default";
        }

        public SamplePipeline(String name) {
            this.name = name;
        }

        /**
         * Return the level detected, or null if inconclusive.
         *
         * @return
         */
        public LiftPosition getPossibleFreightLevel() {
            if (box1detected != DetectionColor.NONE) {
                if (box2detected != DetectionColor.NONE) {
                    if (box3detected != DetectionColor.NONE) {
                        // Inconclusive
                        return null;
                    }
                    else {
                        return LiftPosition.THIRD;
                    }
                }
                else {
                    // box2 is NONE
                    if (box3detected != DetectionColor.NONE) {
                        // We see two boxes
                        return LiftPosition.SECOND;
                    }
                    else {
                        // Inconclusive
                        return null;
                    }
                }
            }
            else {
                // box1 is NONE
                if (box2detected != DetectionColor.NONE && box3detected != DetectionColor.NONE) {
                    // We see two boxes
                    return LiftPosition.FIRST;
                }
                else {
                    // Inconclusive
                    return null;
                }
            }
        }

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */
            @SuppressWarnings("UnnecessaryLocalVariable")
            Mat output = input; /* It is the same thing, but we are using output when we are making
                                   a change to it, and input when we are reading it.
                                 */


            int x = input.cols();
            int y = input.rows();
            int w = x/8;
            w = w - w/3;
            int h = y/8;
            h = h - h/3;

            int leftX_min = (int)(1.5*w);
            int leftX_max = leftX_min + w;

            int middleX_min = x/2 - w/2 - w/8;
            int middleX_max = middleX_min + w;

            int rightX_min = (int)(x - 2.25*w);
            int rightX_max = rightX_min + w;

            int middleY_min = y/2;
            int middleY_max = middleY_min + h;

            int topY_min = (int)(y/4 - 0.5 * h);
            int topY_max = topY_min + h;

            int bottomY_min = (int)(3 * y/4 - 0.5 * h);
            int bottomY_max = bottomY_min + h ;

            Point window_min = new Point(0, y/3);
            Point window_max = new Point(x, 2 * y / 3);

            //Regular boxes
            Point box1_pointA = new Point( //a
                    leftX_min,
                    middleY_min);
            Point box1_pointB = new Point( //b
                    leftX_max,
                    middleY_max);
            Point box2_pointC = new Point( //c
                    middleX_min,
                    middleY_min);
            Point box2_pointD = new Point( //d
                    middleX_max,
                    middleY_max);
            Point box3_pointE = new Point( //e
                    rightX_min,
                    middleY_min);
            Point box3_pointF = new Point( //f
                    rightX_max,
                    middleY_max);

            //convert input before trying to draw on the output
            inputToHSV(input);

            /*
             * Draw a simple box around the barcode dots.
             */
            Imgproc.rectangle( //box 1 - left
                    output, box1_pointA, box1_pointB,
                    box1detected.getValue(), 2);
            Imgproc.rectangle( //box 2 - middle
                    output, box2_pointC, box2_pointD,
                    box2detected.getValue(), 2);
            Imgproc.rectangle( // box 3 - right
                    output, box3_pointE, box3_pointF,
                    box3detected.getValue(), 2);

            // Blue tape: Cr 121,123,126, Cb 142, 139, 137
            // Red tape: Cr 147,149,142, Cb 126,124,127

            /*
            Mat box1 = Cr.submat(new Rect(box1_pointA,box1_pointB));
            System.out.println(name + " Box 1 Cr value average = ("+ Core.mean(box1).val[0] + ", " + Core.mean(box1).val[1] + ", " + Core.mean(box1).val[2] + ")");
            box1 = Cb.submat(new Rect(box1_pointA,box1_pointB));
            System.out.println(name + " Box 1 Cb value average = ("+ Core.mean(box1).val[0] + ", " + Core.mean(box1).val[1] + ", " + Core.mean(box1).val[2] + ")");

            Mat box2 = Cr.submat(new Rect(box2_pointC,box2_pointD));
            System.out.println(name + " Box 2 Cr value average = "+ Core.mean(box2).val[0]);
            box2 = Cb.submat(new Rect(box2_pointC,box2_pointD));
            System.out.println(name + " Box 2 Cb value average = "+ Core.mean(box2).val[0]);

            Mat box3 = Cr.submat(new Rect(box3_pointE,box3_pointF));
            System.out.println(name + " Box 3 Cr value average = "+ Core.mean(box3).val[0]);
            box3 = Cb.submat(new Rect(box3_pointE,box3_pointF));
            System.out.println(name + " Box 3 Cb value average = "+ Core.mean(box3).val[0]);
             */

            // Blue tape Bright: H 105,107,106, S 91,107,88
            // Blue tape Medium: H 109, 109, 107, S 91,106,88
            // Blue tape Dim   : H 109,133,78, S 78,92,60
            // Red tape Osterm: H 122,95,146, S 99,96,102
            // Red tape Bright: H 101,110,105, S 63,81,56
            // Red tape Medium: H 120,136,120, S 63,83,56
            // Red tape Dim   : H 157,162,145, S 58,76,50

            Mat box1 = HSV.submat(new Rect(box1_pointA,box1_pointB));
            Scalar hsvMean = Core.mean(box1);
            System.out.println(name + " Box 1 HSV value average = ("+ hsvMean.val[0] + ", " + hsvMean.val[1] + ", " + hsvMean.val[2] + ")");
            box1detected = detectBlue(hsvMean);
            if (DetectionColor.NONE.equals(box1detected)) {
                box1detected = detectRed(hsvMean);
            }
            System.out.println(name + "Box 1 detected " + box1detected);

            Mat box2 = HSV.submat(new Rect(box2_pointC,box2_pointD));
            hsvMean = Core.mean(box2);
            System.out.println(name + " Box 2 HSV value average = ("+ hsvMean.val[0] + ", " + hsvMean.val[1] + ", " + hsvMean.val[2] + ")");
            box2detected = detectBlue(hsvMean);
            if (DetectionColor.NONE.equals(box2detected)) {
                box2detected = detectRed(hsvMean);
            }
            System.out.println(name + "Box 2 detected " + box2detected);

            Mat box3 = HSV.submat(new Rect(box3_pointE,box3_pointF));
            hsvMean = Core.mean(box3);
            System.out.println(name + " Box 3 HSV value average = ("+ hsvMean.val[0] + ", " + hsvMean.val[1] + ", " + hsvMean.val[2] + ")");
            box3detected = detectBlue(hsvMean);
            if (DetectionColor.NONE.equals(box3detected)) {
                box3detected = detectRed(hsvMean);
            }
            System.out.println(name + "Box 3 detected " + box3detected);

            /*
            Mat middleWindow = input.submat(new Rect(window_min, window_max));
            Imgproc.rectangle(output, window_min, window_max,
                    DetectionColor.GREEN.getValue(), 2);
            Imgproc.cvtColor(middleWindow, HSV, Imgproc.COLOR_RGB2HSV_FULL);
            // Blur the image
            Imgproc.GaussianBlur(HSV, HSV, new Size(5,5), 0);

            Scalar LOWER_RED1 = new Scalar(0, 50, 50);
            Scalar UPPER_RED1 = new Scalar(10, 255, 255);
            Scalar LOWER_RED2 = new Scalar(170, 50, 50);
            Scalar UPPER_RED2 = new Scalar(180, 255, 255);

            // Red wraps around the hue spectrum from 170 - 180 - 0 - 10
            // Split the mask into the bottom and top of the spectrum
            Core.inRange(HSV, LOWER_RED1, UPPER_RED1, mask);
            Core.inRange(HSV, LOWER_RED2, UPPER_RED2, rmask2);
            // Combine the two masks
            Core.bitwise_or(mask, rmask2, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f points = new MatOfPoint2f();
            double current_contour;
            double contourArea = 7;
            System.out.println(name + " contours size=" + contours.size());
            for (int i = 0; i < contours.size(); i++) {
                current_contour = Imgproc.contourArea(contours.get(i));
                if (current_contour > contourArea) {
                    contourArea = current_contour;
                    contours.get(i).convertTo(points, CvType.CV_32FC2); // contours.get(i) is a single MatOfPoint, but to use minEnclosingCircle we need to pass a MatOfPoint2f so we need to do a conversion
                }
            }
            if (!points.empty() && contourArea > MIN_CONTOUR_AREA) {
                Imgproc.minEnclosingCircle(points, center, null);
                Imgproc.circle(output, center, (int) Math.round(Math.sqrt(contourArea / Math.PI)), DetectionColor.RED.getValue(), Core.FILLED);
                System.out.println(name + " contourArea=" + contourArea + ", drawing circle at " + center);
            }
            else {
                System.out.println(name + " contourArea=" + contourArea + ", not drawing any circle");
            }
            contours.clear();
             */

            return output;
        }

        /**
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         * \ (•◡•) / This is Frank. He is very helpful.
         */
        private void inputToYCrCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Y, 0);
            Core.extractChannel(YCrCb, Cr, 1);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        /**
         * This function takes the RGB frame, converts to HSV,
         * and extracts each channel to individual 'H', 'S'. and 'V' variables
         * \ (•◡•) / This is Frank. He is very helpful.
         */
        private void inputToHSV(Mat input)
        {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);
            //Core.extractChannel(HSV, H, 0);
            //Core.extractChannel(HSV, S, 1);
            //Core.extractChannel(HSV, V, 2);
        }

        private DetectionColor detectBlue(Scalar mean) {
            if (mean.val[0] > 140 && mean.val[0] < 155 && mean.val[1] > 50 && mean.val[2] > 50) {
                return DetectionColor.BLUE;
            }
            return DetectionColor.NONE;
        }

        private DetectionColor detectRed(Scalar mean) {
            if ((mean.val[0] > 160 || mean.val[0] < 10)
                && mean.val[1] > 50 && mean.val[2] > 50) {
                return DetectionColor.RED;
            }

            return DetectionColor.NONE;
        }
    }

    class MyListener implements OpenCvCamera.AsyncCameraOpenListener {
        OpenCvWebcam webcam;

        public MyListener(OpenCvWebcam webcam) {
            this.webcam = webcam;
        }

        @Override
        public void onOpened()
        {
            /*
             * Tell the webcam to start streaming images to us! Note that you must make sure
             * the resolution you specify is supported by the camera. If it is not, an exception
             * will be thrown.
             *
             * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
             * supports streaming from the webcam in the uncompressed YUV image format. This means
             * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
             * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
             *
             * Also, we specify the rotation that the webcam is used in. This is so that the image
             * from the camera sensor can be rotated such that it is always displayed with the image upright.
             * For a front facing camera, rotation is defined assuming the user is looking at the screen.
             * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
             * away from the user.
             */
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode)
        {
            /*
             * This will be called if the camera could not be opened
             */
        }

    }

    enum DetectionColor {
        NONE (new Scalar(255,0,255)),
        RED (new Scalar(255,0,0)),
        BLUE (new Scalar(0,0,255)),
        GREEN (new Scalar(0,255,0)),
        WHITE (new Scalar(255,255,255)),
        YELLOW (new Scalar(255,255,0));

        private final Scalar value;

        DetectionColor(Scalar value) {
            this.value = value;
        }

        public Scalar getValue() {
            return value;
        }
    }
}
