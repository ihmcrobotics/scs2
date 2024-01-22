package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import javafx.animation.Interpolator;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.geometry.Rectangle2D;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.util.Duration;
import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber;
import org.bytedeco.javacv.JavaFXFrameConverter;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader.FrameData;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class VideoViewer
{

   private static final double THUMBNAIL_HIGHLIGHT_SCALE = 1.05;

   private final ImageView thumbnail = new ImageView();
   private final StackPane thumbnailContainer = new StackPane(thumbnail);
   private final ImageView videoView = new ImageView();

   private final BooleanProperty updateVideoView = new SimpleBooleanProperty(this, "updateVideoView", false);
   private final ObjectProperty<Stage> videoWindowProperty = new SimpleObjectProperty<>(this, "videoWindow", null);
   private final JavaFXFrameConverter frameConverter = new JavaFXFrameConverter();
   private final us.ihmc.scs2.sessionVisualizer.jfx.session.mcap.VideoDataReader reader;
   private final double defaultThumbnailSize;

   private final ObjectProperty<Pane> imageViewRootPane = new SimpleObjectProperty<>(this, "imageViewRootPane", null);

   public VideoViewer(Window owner, us.ihmc.scs2.sessionVisualizer.jfx.session.mcap.VideoDataReader reader, double defaultThumbnailSize)
   {
      this.reader = reader;
      this.defaultThumbnailSize = defaultThumbnailSize;
      thumbnail.setPreserveRatio(true);
      videoView.setPreserveRatio(true);
      thumbnail.setFitWidth(defaultThumbnailSize);
      thumbnail.setOnMouseEntered(e ->
      {
         Timeline timeline = new Timeline(new KeyFrame(Duration.seconds(0.1),
                                                       new KeyValue(thumbnail.fitWidthProperty(),
                                                                    THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize,
                                                                    Interpolator.EASE_BOTH)));
         timeline.playFromStart();
      });
      thumbnail.setOnMouseExited(e ->
      {
         Timeline timeline = new Timeline(new KeyFrame(Duration.seconds(0.1),
                                                       new KeyValue(thumbnail.fitWidthProperty(), defaultThumbnailSize, Interpolator.EASE_BOTH)));
         timeline.playFromStart();
      });

      thumbnail.addEventHandler(MouseEvent.MOUSE_CLICKED, e ->
      {
         if (e.getClickCount() != 2)
            return;

         videoView.setImage(thumbnail.getImage());

         Stage stage;

         if (videoWindowProperty.get() != null)
         {
            stage = videoWindowProperty.get();
         }
         else
         {
            stage = new Stage();
            AnchorPane anchorPane = new AnchorPane();
            Pane root = createImageViewPane(videoView);
            anchorPane.getChildren().add(root);
            JavaFXMissingTools.setAnchorConstraints(root, 0);
            imageViewRootPane.set(root);

            videoWindowProperty.set(stage);
            stage.getIcons().add(SessionVisualizerIOTools.LOG_SESSION_IMAGE);
            stage.setTitle(reader.toString());
            owner.setOnHiding(e2 -> stage.close());
            Scene scene = new Scene(anchorPane);
            stage.setScene(scene);
            updateVideoView.bind(stage.showingProperty());
         }

         Screen screen = Screen.getScreensForRectangle(e.getScreenX(), e.getScreenY(), 1, 1).get(0);
         Rectangle2D visualBounds = screen.getVisualBounds();
         double width = 0.5 * visualBounds.getWidth();
         double height = 0.5 * visualBounds.getHeight();
         double x = visualBounds.getMinX() + 0.5 * (visualBounds.getWidth() - width);
         double y = visualBounds.getMinY() + 0.5 * (visualBounds.getHeight() - height);
         stage.setX(x);
         stage.setY(y);
         stage.setWidth(width);
         stage.setHeight(height);

         stage.toFront();
         stage.show();
      });
   }

   private static Pane createImageViewPane(ImageView imageView)
   {
      return new Pane(imageView)
      {
         @Override
         protected void layoutChildren()
         {
            Image image = imageView.getImage();
            if (image == null)
               return;
            double imageRatio = image.getWidth() / image.getHeight();
            double paneWidth = getWidth() - getPadding().getTop() - getPadding().getBottom();
            double paneHeight = getHeight() - getPadding().getLeft() - getPadding().getRight();
            double width = Math.min(paneWidth, paneHeight * imageRatio);
            double height = width / imageRatio;

            double x = 0.5 * (paneWidth - width);
            double y = 0.5 * (paneHeight - height);
            imageView.setFitWidth(width);
            imageView.setX(x + getPadding().getLeft());
            imageView.setY(y + getPadding().getTop());
         }
      };
   }

   public void update()
   {
      Frame currentFrame = reader.getCurrentFrame();

      if (currentFrame == null || currentFrame.image == null)
         return;
      Image currentImage = null;
      try
      {
         currentImage = this.frameConverter.convert(currentFrame);
      } catch (RuntimeException e)
      {
         LogTools.error("Frame has {} image channels", currentFrame.imageChannels);
      }

      if (currentImage == null)
         return;

      thumbnailContainer.setPrefWidth(THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize);
      thumbnailContainer.setPrefHeight(THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize * currentImage.getHeight() / currentImage.getWidth());

      thumbnail.setImage(currentImage);

      if (updateVideoView.get())
      {
         videoView.setImage(currentImage);

         if (imageViewRootPane.get() != null)
         {
            imageViewRootPane.get().setPadding(new Insets(16,16,16,16));
         }
      }
   }

   public void stop()
   {
      if (videoWindowProperty.get() != null)
      {
         videoWindowProperty.get().close();
         videoWindowProperty.set(null);
      }
      reader.shutdown();
   }

   public Node getThumbnail()
   {
      return thumbnailContainer;
   }
}
