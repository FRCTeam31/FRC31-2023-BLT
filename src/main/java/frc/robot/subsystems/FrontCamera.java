package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FrontCamera extends SubsystemBase {
    public Thread mProcessingThread;

    public FrontCamera() {
        // mProcessingThread = new Thread(
        // () -> {
        // // Get the UsbCamera from CameraServer and set video mode
        // var camera = CameraServer.startAutomaticCapture();
        // camera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        // // Get a CvSink. This will capture Mats (matrixes) from the camera
        // var cvSink = CameraServer.getVideo();

        // // Setup a CvSource. This will send images back to the Dashboard
        // var outputStream = CameraServer.putVideo("Flipped", 640, 480);

        // // Mats are very memory expensive so we want to re-use these Mats
        // var inputMat = new Mat();
        // var flippedOutputMat = new Mat();

        // // This cannot be 'true'. The program will never exit if it is. This
        // // lets the robot stop this thread when restarting robot code or
        // // deploying.
        // while (!Thread.interrupted()) {
        // // Tell the CvSink to grab a frame from the camera and put it
        // // in the source mat. If there is an error notify the output.
        // if (cvSink.grabFrame(inputMat) == 0) {
        // // Send the output the error.
        // outputStream.notifyError(cvSink.getError());
        // // skip the rest of the current iteration
        // continue;
        // }

        // // Flip the image
        // Core.flip(inputMat, flippedOutputMat, 1);

        // // Give the output stream a new image to display
        // outputStream.putFrame(flippedOutputMat);
        // }
        // });

        // mProcessingThread.start();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addBooleanProperty("Flipped thread is alive", () ->
        // mProcessingThread.isAlive(), null);
    }
}
