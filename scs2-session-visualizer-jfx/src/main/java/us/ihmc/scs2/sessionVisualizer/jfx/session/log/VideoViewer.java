package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

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
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.util.Duration;
import org.bytedeco.javacv.JavaFXFrameConverter;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.BytedecoVideoReader.FrameData;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class VideoViewer
{

   private static final boolean LOGGER_VIDEO_DEBUG = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.logger.video.debug",
                                                                                                              "SCS2_GUI_LOGGER_VIDEO_DEBUG",
                                                                                                              false);
   private static final double THUMBNAIL_HIGHLIGHT_SCALE = 1.05;

   private final ImageView thumbnail = new ImageView();
   private final StackPane thumbnailContainer = new StackPane(thumbnail);
   private final ImageView videoView = new ImageView();
   private final Label queryRobotTimestampLabel = new Label();
   private final Label demuxerCurrentPTSLabel = new Label();
   private final Label cameraCurrentPTSLabel = new Label();
   private final Label robotTimestampLabel = new Label();

   private final BooleanProperty updateVideoView = new SimpleBooleanProperty(this, "updateVideoView", false);
   private final ObjectProperty<Stage> videoWindowProperty = new SimpleObjectProperty<>(this, "videoWindow", null);
   private final BytedecoVideoReader reader;
   private final double defaultThumbnailSize;

   private final JavaFXFrameConverter frameConverter = new JavaFXFrameConverter();

   private final ObjectProperty<Pane> imageViewRootPane = new SimpleObjectProperty<>(this, "imageViewRootPane", null);

   public VideoViewer(Window owner, BytedecoVideoReader reader, double defaultThumbnailSize)
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

            setupVideoStatistics(anchorPane);

            videoWindowProperty.set(stage);
            stage.getIcons().add(SessionVisualizerIOTools.LOG_SESSION_IMAGE);
            stage.setTitle(reader.getName());
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

   private void setupVideoStatistics(AnchorPane anchorPane)
   {
      Label videoStatisticTitle = new Label("Video Statistics");
      videoStatisticTitle.setFont(Font.font("Arial", FontWeight.BOLD, 14));

      Background generalBackground = new Background(new BackgroundFill(Color.WHITE, CornerRadii.EMPTY, Insets.EMPTY));
      Border noRightBorder = new Border(new BorderStroke(Color.BLACK, null, Color.BLACK, Color.BLACK,
                                       BorderStrokeStyle.SOLID, BorderStrokeStyle.NONE, BorderStrokeStyle.SOLID, BorderStrokeStyle.SOLID,
                                       CornerRadii.EMPTY, BorderWidths.DEFAULT, Insets.EMPTY));
      Border noLeftBorder = new Border(new BorderStroke(Color.BLACK, Color.BLACK, Color.BLACK, null,
                                       BorderStrokeStyle.SOLID, BorderStrokeStyle.SOLID, BorderStrokeStyle.SOLID, BorderStrokeStyle.NONE,
                                       CornerRadii.EMPTY, BorderWidths.DEFAULT, Insets.EMPTY));

      Border generalBorder = new Border(new BorderStroke(Color.BLACK, BorderStrokeStyle.SOLID, CornerRadii.EMPTY, BorderWidths.DEFAULT));
      Insets textInsets = new Insets(0, 2, 0, 2);


      if (LOGGER_VIDEO_DEBUG)
      {
         VBox videoStatisticBox = new VBox(videoStatisticTitle);
         videoStatisticBox.setAlignment(Pos.CENTER);
         videoStatisticBox.setBackground(generalBackground);
         videoStatisticBox.setBorder(generalBorder);

         VBox videoStatisticLabels = new VBox(new Label("queryRobotTimestamp"), new Label("robotTimestamp"), new Label("cameraCurrentPTS"), new Label("demuxerCurrentPTS"));
         videoStatisticLabels.setBackground(generalBackground);
         videoStatisticLabels.setBorder(noRightBorder);
         videoStatisticLabels.setPadding(textInsets);

         VBox videoStatistics = new VBox(queryRobotTimestampLabel, robotTimestampLabel, cameraCurrentPTSLabel, demuxerCurrentPTSLabel);
         videoStatistics.setBackground(generalBackground);
         videoStatistics.setBorder(noLeftBorder);
         videoStatistics.setPadding(textInsets);

         HBox labelsContainer = new HBox(0, videoStatisticLabels, videoStatistics);
         VBox videoStatisticsDisplay = new VBox(0, videoStatisticBox, labelsContainer);
         anchorPane.getChildren().add(videoStatisticsDisplay);
         AnchorPane.setLeftAnchor(videoStatisticsDisplay, 0.0);
         AnchorPane.setBottomAnchor(videoStatisticsDisplay, 0.0);
      }
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
      FrameData currentFrameData = reader.pollCurrentFrame();
      Image currentImage;

      if (currentFrameData.frame == null)
         return;

      thumbnailContainer.setPrefWidth(THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize);
      thumbnailContainer.setPrefHeight(THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize * reader.getImageHeight()/ reader.getImageWidth());

      try
      {
         if (currentFrameData.frame.imageHeight > 0 && currentFrameData.frame.imageWidth > 0)
         {
            currentImage = frameConverter.convert(currentFrameData.frame);
            thumbnail.setImage(currentImage);
            videoView.setImage(currentImage);
         }
      }
      catch(UnsupportedOperationException ignored)
      {
      }

      if (updateVideoView.get())
      {
         queryRobotTimestampLabel.setText(Long.toString(currentFrameData.queryRobotTimestamp));
         robotTimestampLabel.setText(Long.toString(currentFrameData.robotTimestamp));
         cameraCurrentPTSLabel.setText(Long.toString(currentFrameData.cameraCurrentPTS));
         demuxerCurrentPTSLabel.setText(Long.toString(currentFrameData.demuxerCurrentPTS));

         if (imageViewRootPane.get() != null)
         {
            imageViewRootPane.get().setPadding(new Insets(16,16,16,16));

            if (reader.replacedRobotTimestampsContainsIndex(reader.getCurrentIndex()))
            {
               imageViewRootPane.get().setBackground(new Background(new BackgroundFill(Color.DARKORANGE, CornerRadii.EMPTY, Insets.EMPTY)));
            }
            else
            {
               imageViewRootPane.get().setBackground(new Background(new BackgroundFill(Color.WHITE, CornerRadii.EMPTY, Insets.EMPTY)));
            }
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
   }

   public Node getThumbnail()
   {
      return thumbnailContainer;
   }
}
