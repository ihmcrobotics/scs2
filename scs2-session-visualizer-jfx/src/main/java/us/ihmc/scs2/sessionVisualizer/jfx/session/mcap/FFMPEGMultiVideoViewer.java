package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

import java.util.ArrayList;
import java.util.List;

public class FFMPEGMultiVideoViewer extends ObservedAnimationTimer
{
   private final Pane thumbnailsContainer;
   private final List<FFMPEGVideoViewer> videoViewers = new ArrayList<>();

   public FFMPEGMultiVideoViewer(Window owner, Pane thumbnailsContainer, FFMPEGMultiVideoDataReader multiReader, double defaultThumbnailWidth)
   {
      this.thumbnailsContainer = thumbnailsContainer;
      for (FFMPEGVideoDataReader reader : multiReader.getReaders())
      {
         videoViewers.add(new FFMPEGVideoViewer(owner, reader, defaultThumbnailWidth));
      }
   }

   @Override
   public void start()
   {
      super.start();
      for (FFMPEGVideoViewer videoViewer : videoViewers)
      {
         thumbnailsContainer.getChildren().add(videoViewer.getThumbnail());
      }
   }

   @Override
   public void handleImpl(long now)
   {
      videoViewers.forEach(FFMPEGVideoViewer::update);
   }

   @Override
   public void stop()
   {
      super.stop();
      for (FFMPEGVideoViewer videoViewer : videoViewers)
      {
         thumbnailsContainer.getChildren().remove(videoViewer.getThumbnail());
         videoViewer.stop();
      }
   }
}
