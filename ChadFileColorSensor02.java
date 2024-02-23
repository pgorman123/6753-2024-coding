import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.highgui.HighGui;

public class BetterColorDetection {

    private static final Scalar DEFAULT_LOWER_BOUND = new Scalar(0, 100, 100);
    private static final Scalar DEFAULT_UPPER_BOUND = new Scalar(22, 255, 255);
    private static final int ESC_KEY_CODE = 27;

    private Scalar lowerBound;
    private Scalar upperBound;

    public BetterColorDetection(Scalar lowerBound, Scalar upperBound) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    public void detectColor(String windowName) {
        VideoCapture videoCapture = new VideoCapture(0); // Accessing the webcam
        if (!videoCapture.isOpened()) {
            System.out.println("Error: Unable to access webcam.");
            return;
        }

        Mat frame = new Mat();
        Mat hsvImage = new Mat();

        while (true) {
            videoCapture.read(frame);
            if (frame.empty()) {
                break;
            }

            Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

            Mat colorMask = new Mat();
            Core.inRange(hsvImage, lowerBound, upperBound, colorMask);

            Mat maskedImage = new Mat();
            Core.bitwise_and(frame, frame, maskedImage, colorMask);

            HighGui.imshow(windowName, maskedImage);
            if (HighGui.waitKey(30) == ESC_KEY_CODE) {
                break;
            }
        }

        videoCapture.release();
        HighGui.destroyAllWindows();
    }

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        Scalar customLowerBound = new Scalar(5, 100, 100); // Example of custom lower bound
        Scalar customUpperBound = new Scalar(15, 255, 255); // Example of custom upper bound
        BetterColorDetection colorDetection = new BetterColorDetection(customLowerBound, customUpperBound);
        colorDetection.detectColor("Custom Color Detection");
    }
}
