/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class WebcamExample extends LinearOpMode
{
    String webcam1Name = "Webcam1";
    String webcam2Name = "Webcam2";
    OpenCvWebcam webcam1Cv = null; // Define a first camera (the big one)
    OpenCvWebcam webcam2Cv = null; // Define a second camera (the small one)

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
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


        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

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
            webcam1Cv.setPipeline(new SamplePipeline("Big Webcam"));
            webcam1Cv.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam1Cv.openCameraDeviceAsync(new MyListener(webcam1Cv));
        }
        if (webcam2Cv != null) {
            webcam2Cv.setPipeline(new SamplePipeline("Little Webcam"));
            webcam2Cv.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam2Cv.openCameraDeviceAsync(new MyListener(webcam2Cv));
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * *:・ﾟ ₍ᐢ•ﻌ•ᐢ₎*:・ﾟ
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam1Cv.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam1Cv.getFps()));
            telemetry.addData("Total frame time ms", webcam1Cv.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam1Cv.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam1Cv.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam1Cv.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * ૮ ˆﻌˆ ა This is Gilbert, the resident code-dog. If you are confused, just focus on Gilbert and how
                 *         amazingly adorable he is.
                 *
                 * ʕ •ₒ• ʔ This is Fredrick, Gilbert's best friend. He is a bear. A very adorable bear.
                 *
                 * Why Helen... Why?!?!
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam1Cv.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
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

        boolean viewportPaused;

        private Mat YCrCb = new Mat();
        private Mat Y = new Mat();
        private Mat Cr = new Mat();
        private Mat Cb = new Mat();

        private DetectionColor box1detected = DetectionColor.NONE;
        private DetectionColor box2detected = DetectionColor.NONE;
        private DetectionColor box3detected = DetectionColor.NONE;

        public SamplePipeline() {
            this.name = "default";
        }

        public SamplePipeline(String name) {
            this.name = name;
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
            Scalar RED = new Scalar(255,0,0);
            Scalar GREEN = new Scalar(0,255,0);
            Scalar BLUE = new Scalar(0,0,255);
            Scalar WHITE = new Scalar(255,255,255);
            Scalar YELLOW = new Scalar(255,255,0);
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */
            @SuppressWarnings("UnnecessaryLocalVariable") Mat output = input; /*It is the same thing,
        but we are using output when we are making a change to it, and input when we are reading it. */


            int x = input.cols();
            int y = input.rows();
            int w = x/8;
            int h = y/8;

            int leftX_min = w;
            int leftX_max = leftX_min + 2*w;

            int middleX_min = x/2 - w/2 - w/8;
            int middleX_max = middleX_min + w;

            int rightX_min = x - 4*w;
            int rightX_max = rightX_min + w;

            int middleY_min = y/2;
            int middleY_max = middleY_min + h;

            int topY_min = (int)(y/4 - 0.5 * h);
            int topY_max = topY_min + h;

            int bottomY_min = (int)(3 * y/4 - 0.5 * h);
            int bottomY_max = bottomY_min + h ;

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

            // YCrCb boxes
            Point boxY_pointA = new Point( //a
                    leftX_min,
                    topY_min);
            Point boxY_pointB = new Point( //b
                    leftX_max,
                    topY_max);
            Point boxCr_pointC = new Point( //c
                    middleX_min,
                    topY_min);
            Point boxCr_pointD = new Point( //d
                    middleX_max,
                    topY_max);
            Point boxCb_pointE = new Point( //e
                    rightX_min,
                    topY_min);
            Point boxCb_pointF = new Point( //f
                    rightX_max,
                    topY_max);

            // HSV boxes
            Point boxH_pointA = new Point( //a
                    leftX_min,
                    bottomY_min);
            Point boxH_pointB = new Point( //b
                    leftX_max,
                    bottomY_max);
            Point boxS_pointC = new Point( //c
                    middleX_min,
                    bottomY_min);
            Point boxS_pointD = new Point( //d
                    middleX_max,
                    bottomY_max);
            Point boxV_pointE = new Point( //e
                    rightX_min,
                    bottomY_min);
            Point boxV_pointF = new Point( //f
                    rightX_max,
                    bottomY_max);

            //convert input before trying to draw on the output
            inputToYCrCb(input);

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
            // Value boxes
            /*
            Imgproc.rectangle( //box Y - left
                    output, boxY_pointA, boxY_pointB,
                    new Scalar(0, 255, 0), 2);
            Imgproc.rectangle( //box Cr - middle
                    output, boxCr_pointC, boxCr_pointD,
                    new Scalar(0, 255, 0), 2);
            Imgproc.rectangle( // box Cb - right
                    output, boxCb_pointE, boxCb_pointF,
                    new Scalar(0, 255, 0), 2);
            Imgproc.rectangle( //box H - left
                    output, boxH_pointA, boxH_pointB,
                    new Scalar(0, 0, 255), 2);
            Imgproc.rectangle( //box S - middle
                    output, boxS_pointC, boxS_pointD,
                    new Scalar(0, 0, 255), 2);
            Imgproc.rectangle( // box V - right
                    output, boxV_pointE, boxV_pointF,
                    new Scalar(0, 0, 255), 2);

             */

            Mat box1 = Cr.submat(new Rect(box1_pointA,box1_pointB));
            System.out.printf(name + " Box 1 Cr value average = "+ Core.mean(box1).val[1]);
            Mat box2 = Cr.submat(new Rect(box2_pointC,box2_pointD));
            System.out.printf(name + " Box 2 Cr value average = "+ Core.mean(box2).val[1]);
            Mat box3 = Cr.submat(new Rect(box3_pointE,box3_pointF));
            System.out.printf(name + " Box 3 Cr value average = "+ Core.mean(box3).val[1]);

            box1 = Cb.submat(new Rect(box1_pointA,box1_pointB));
            System.out.printf(name + " Box 1 Cb value average = "+ Core.mean(box1).val[2]);
            box2 = Cb.submat(new Rect(box2_pointC,box2_pointD));
            System.out.printf(name + " Box 2 Cb value average = "+ Core.mean(box2).val[2]);
            box3 = Cb.submat(new Rect(box3_pointE,box3_pointF));
            System.out.printf(name + " Box 3 Cb value average = "+ Core.mean(box3).val[2]);

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

//        @Override
        public void doNotRunOnViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam1Cv.pauseViewport();
            }
            else
            {
                webcam1Cv.resumeViewport();
            }
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


/*
༼ つ ◕_◕ ༽つ = Destiny the blob
 */