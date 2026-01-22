package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

public class MultiVideoViewer extends ObservedAnimationTimer
{
   private final Pane thumbnailsContainer;
   private final List<VideoDataReader> readers = new ArrayList<>();
   private final List<VideoViewer> videoViewers = new ArrayList<>();
   private boolean isStarted = false;
   private final Window owner;
   private final double defaultThumbnailWidth;

   public MultiVideoViewer(Window owner, Pane thumbnailsContainer, MultiVideoDataReader multiReader, double defaultThumbnailWidth)
   {
      this.owner = owner;
      this.thumbnailsContainer = thumbnailsContainer;
      this.defaultThumbnailWidth = defaultThumbnailWidth;

      for (VideoDataReader reader : multiReader.getReaders())
      {
         videoViewers.add(new VideoViewer(owner, reader, defaultThumbnailWidth));
         readers.add(reader);
      }
   }

   public void addVideoReader(MultiVideoDataReader multiReader)
   {
      multiReader.getReaders().forEach( reader ->
                                        {
                                           readers.add(reader);
                                           addVideoViewer(new VideoViewer(owner, reader, defaultThumbnailWidth));
                                        });
   }

   public void addVideoViewer(VideoViewer videoViewer)
   {
      videoViewers.add(videoViewer);
      if (isStarted)
         thumbnailsContainer.getChildren().add(videoViewer.getThumbnail());
   }

   @Override
   public void start()
   {
      super.start();

      for (VideoViewer videoViewer : videoViewers)
      {
         thumbnailsContainer.getChildren().add(videoViewer.getThumbnail());
      }

      isStarted = true;
   }

   @Override
   public void handleImpl(long now)
   {
      videoViewers.forEach(VideoViewer::update);
   }

   @Override
   public void stop()
   {
      super.stop();

      for (VideoViewer videoViewer : videoViewers)
      {
         thumbnailsContainer.getChildren().remove(videoViewer.getThumbnail());
         videoViewer.stop();
      }

      isStarted = false;
   }
}
