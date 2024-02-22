import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class ColorDetection {

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        VideoCapture videoCapture = new VideoCapture(0); // Accessing the webcam
        if (!videoCapture.isOpened()) {
            System.out.println("Error: Unable to access webcam.");
            return;
        }

        Mat frame = new Mat();
        Mat hsvImage = new Mat();
        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);

        while (true) {
            videoCapture.read(frame);
            if (frame.empty()) {
                break;
            }

            Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsvImage, lowerRed, upperRed, hsvImage);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.erode(hsvImage, hsvImage, kernel);
            Imgproc.dilate(hsvImage, hsvImage, kernel);

            Core.bitwise_and(frame, frame, frame, hsvImage);

            Core.imshow("Color Detection", frame);
            if (Core.waitKey(30) == 27) {
                break;
            }
        }

        videoCapture.release();
        Core.destroyAllWindows();
    }
}
