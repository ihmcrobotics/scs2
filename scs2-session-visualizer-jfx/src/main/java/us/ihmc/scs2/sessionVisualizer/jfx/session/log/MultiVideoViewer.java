package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

public class MultiVideoViewer extends ObservedAnimationTimer
{
   private final Pane thumbnailsContainer;
   private final List<VideoViewer> videoViewers = new ArrayList<>();

   public MultiVideoViewer(Window owner, Pane thumbnailsContainer, MultiVideoDataReader multiReader, double defaultThumbnailWidth)
   {
      this.thumbnailsContainer = thumbnailsContainer;

      for (VideoDataReader reader : multiReader.getReaders())
      {
         videoViewers.add(new VideoViewer(owner, reader, defaultThumbnailWidth));
      }
   }

   @Override
   public void start()
   {
      super.start();

      for (VideoViewer videoViewer : videoViewers)
      {
         thumbnailsContainer.getChildren().add(videoViewer.getThumbnail());
      }
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
   }
}
