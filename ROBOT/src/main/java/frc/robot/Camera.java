

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * This class is a basic framework that creates threads to handle multiple
 * cameras streaming data back to the driver station at the same time in 
 * real time. If you do not understand how multithreading works, DO NOT
 * CHANGE ANY of the code in this file. If you want to learn about multithreading
 * in Java, use google. Do not mess around inside this file to educate yourself.
 * IF YOU DO NOT KNOW HOW MULTITHREADING WORKS, DO NOT F*CK AROUND IN THIS FILE.
 */
public class Camera extends Thread
{
    int cameraInstance;
    int width;
    int height;
    int framerate;
    String name;
    
    public Camera(int cameraInstance, int width, int height, int framerate, String name)
    {
        this.cameraInstance = cameraInstance;
        this.width = width;
        this.height = height;
        this.framerate = framerate;
        this.name = name;
    }

    public void run()
    {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(cameraInstance);
        camera.setResolution(width, height);
        camera.setFPS(framerate);
        CvSink cSink = CameraServer.getInstance().getVideo();
        CvSource cameraOutStream = CameraServer.getInstance().putVideo(name, width, height);
        Mat mSource = new Mat();
        Mat mOutput = new Mat();

        while(!Camera.interrupted())
        {
            cSink.grabFrame(mSource);
            Imgproc.cvtColor(mSource, mOutput, Imgproc.COLOR_BGR2GRAY);
            cameraOutStream.putFrame(mOutput);
        }
    }
}