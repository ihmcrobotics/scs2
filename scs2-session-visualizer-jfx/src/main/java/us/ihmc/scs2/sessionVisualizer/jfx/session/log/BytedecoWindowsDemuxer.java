package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber;
import us.ihmc.robotDataLogger.Camera;

import java.io.File;

public class BytedecoWindowsDemuxer
{
    // For testing things recorded from the logger
    public String videoPath;
    public String timestampPath;

    private final FFmpegFrameGrabber grabber;

    public BytedecoWindowsDemuxer(File dataDirectory, Camera camera)
    {
        videoPath = dataDirectory.getAbsolutePath() + "/" + camera.getVideoFile();
        timestampPath = dataDirectory.getAbsolutePath() + "/" + camera.getTimestampFile();

        try
        {
            grabber = new FFmpegFrameGrabber(videoPath);
            grabber.start();
        }
        catch (FrameGrabber.Exception e)
        {
            throw new RuntimeException(e);
        }
    }

    public String getName()
    {
        return "Bytedeco Windows";
    }

    public int getImageHeight()
    {
        return grabber.getImageHeight();
    }

    public int getImageWidth()
    {
        return grabber.getImageWidth();
    }

    public long getCurrentPTS()
    {
        return grabber.getTimestamp();
    }

    public void seekToPTS(long videoTimestamp)
    {
        try
        {
            grabber.setTimestamp(videoTimestamp);
        }
        catch (FFmpegFrameGrabber.Exception e)
        {
            throw new RuntimeException(e);
        }
    }

    public Frame getNextFrame()
    {
        try
        {
            return grabber.grabFrame();
        }
        catch (FrameGrabber.Exception e)
        {
            throw new RuntimeException(e);
        }
    }
}
