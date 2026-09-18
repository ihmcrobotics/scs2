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
import javafx.scene.image.PixelReader;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.Border;
import javafx.scene.layout.BorderStroke;
import javafx.scene.layout.BorderStrokeStyle;
import javafx.scene.layout.BorderWidths;
import javafx.scene.layout.CornerRadii;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.util.Duration;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class VideoViewer
{

   private static final boolean LOGGER_VIDEO_DEBUG = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.logger.video.debug",
                                                                                                              "SCS2_GUI_LOGGER_VIDEO_DEBUG",
                                                                                                              false);
   private static final double THUMBNAIL_HIGHLIGHT_SCALE = 1.05;
   private static final Background REPLACED_TIMESTAMP_BACKGROUND = new Background(new BackgroundFill(Color.DARKORANGE, CornerRadii.EMPTY, Insets.EMPTY));
   private static final Background NORMAL_BACKGROUND = new Background(new BackgroundFill(Color.WHITE, CornerRadii.EMPTY, Insets.EMPTY));

   private final ImageView thumbnail = new ImageView();
   private final StackPane thumbnailContainer = new StackPane(thumbnail);
   private final ImageView videoView = new ImageView();
   private final Label queryRobotTimestampLabel = new Label();
   private final Label currentDemuxerTimestampLabel = new Label();
   private final Label currentVideoTimestampLabel = new Label();
   private final Label currentRobotTimestampLabel = new Label();

   private final BooleanProperty updateVideoView = new SimpleBooleanProperty(this, "updateVideoView", false);
   private final ObjectProperty<Stage> videoWindowProperty = new SimpleObjectProperty<>(this, "videoWindow", null);
   private final VideoDataReader reader;
   private final double defaultThumbnailSize;

   private final ObjectProperty<Pane> imageViewRootPane = new SimpleObjectProperty<>(this, "imageViewRootPane", null);

   /**
    * Caches to avoid redundantly re-applying unchanged layout/paint properties every pulse (60Hz) in
    * {@link #update()} - each of these forces a CSS/layout invalidation pass for no visual benefit
    * when the underlying value hasn't actually changed.
    */
   private double lastThumbnailContainerWidth = Double.NaN;
   private double lastThumbnailContainerHeight = Double.NaN;
   private Boolean lastReplacedTimestampHighlight = null;
   /**
    * Tracks which full-resolution frame the current {@link #thumbnail} image was downscaled from, so
    * {@link #createThumbnailImage(WritableImage, int)} only runs when a genuinely new frame arrived,
    * not on every pulse.
    */
   private WritableImage lastThumbnailSourceFrame = null;

   public VideoViewer(Window owner, VideoDataReader reader, double defaultThumbnailSize)
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
                                                                                  new KeyValue(thumbnail.fitWidthProperty(),
                                                                                               defaultThumbnailSize,
                                                                                               Interpolator.EASE_BOTH)));
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
            root.setPadding(new Insets(16, 16, 16, 16));
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
      Border noRightBorder = new Border(new BorderStroke(Color.BLACK,
                                                         null,
                                                         Color.BLACK,
                                                         Color.BLACK,
                                                         BorderStrokeStyle.SOLID,
                                                         BorderStrokeStyle.NONE,
                                                         BorderStrokeStyle.SOLID,
                                                         BorderStrokeStyle.SOLID,
                                                         CornerRadii.EMPTY,
                                                         BorderWidths.DEFAULT,
                                                         Insets.EMPTY));
      Border noLeftBorder = new Border(new BorderStroke(Color.BLACK,
                                                        Color.BLACK,
                                                        Color.BLACK,
                                                        null,
                                                        BorderStrokeStyle.SOLID,
                                                        BorderStrokeStyle.SOLID,
                                                        BorderStrokeStyle.SOLID,
                                                        BorderStrokeStyle.NONE,
                                                        CornerRadii.EMPTY,
                                                        BorderWidths.DEFAULT,
                                                        Insets.EMPTY));

      Border generalBorder = new Border(new BorderStroke(Color.BLACK, BorderStrokeStyle.SOLID, CornerRadii.EMPTY, BorderWidths.DEFAULT));
      Insets textInsets = new Insets(0, 2, 0, 2);

      if (LOGGER_VIDEO_DEBUG)
      {
         VBox videoStatisticBox = new VBox(videoStatisticTitle);
         videoStatisticBox.setAlignment(Pos.CENTER);
         videoStatisticBox.setBackground(generalBackground);
         videoStatisticBox.setBorder(generalBorder);

         VBox videoStatisticLabels = new VBox(new Label("queryRobotTimestamp"),
                                              new Label("currentRobotTimestamp"),
                                              new Label("currentVideoTimestamp"),
                                              new Label("currentDemuxerTimestamp"));
         videoStatisticLabels.setBackground(generalBackground);
         videoStatisticLabels.setBorder(noRightBorder);
         videoStatisticLabels.setPadding(textInsets);

         VBox videoStatistics = new VBox(queryRobotTimestampLabel, currentRobotTimestampLabel, currentVideoTimestampLabel, currentDemuxerTimestampLabel);
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
            Rectangle2D imageViewport = imageView.getViewport();
            if (imageViewport != null)
               imageRatio = imageViewport.getWidth() / imageViewport.getHeight();
            double paneWidth = getWidth() - getPadding().getLeft() - getPadding().getRight();
            double paneHeight = getHeight() - getPadding().getTop() - getPadding().getBottom();
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

      if (currentFrameData.frame == null)
         return;

      WritableImage currentFrame = currentFrameData.frame;

      double thumbnailContainerWidth = THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize;
      double thumbnailContainerHeight = THUMBNAIL_HIGHLIGHT_SCALE * defaultThumbnailSize * currentFrame.getHeight() / currentFrame.getWidth();
      if (thumbnailContainerWidth != lastThumbnailContainerWidth || thumbnailContainerHeight != lastThumbnailContainerHeight)
      {
         thumbnailContainer.setPrefWidth(thumbnailContainerWidth);
         thumbnailContainer.setPrefHeight(thumbnailContainerHeight);
         lastThumbnailContainerWidth = thumbnailContainerWidth;
         lastThumbnailContainerHeight = thumbnailContainerHeight;
      }

      if (currentFrame != lastThumbnailSourceFrame)
      {
         thumbnail.setImage(createThumbnailImage(currentFrame, (int) Math.round(defaultThumbnailSize)));
         lastThumbnailSourceFrame = currentFrame;
      }

      if (updateVideoView.get())
      {
         videoView.setImage(currentFrame);
         queryRobotTimestampLabel.setText(Long.toString(currentFrameData.queryRobotTimestamp));
         currentRobotTimestampLabel.setText(Long.toString(currentFrameData.currentRobotTimestamp));
         currentVideoTimestampLabel.setText(Long.toString(currentFrameData.currentVideoTimestamp));
         currentDemuxerTimestampLabel.setText(Long.toString(currentFrameData.currentDemuxerTimestamp));

         if (imageViewRootPane.get() != null)
         {
            boolean highlight = reader.replacedRobotTimestampsContainsIndex(reader.getCurrentIndex());
            if (lastReplacedTimestampHighlight == null || lastReplacedTimestampHighlight != highlight)
            {
               imageViewRootPane.get().setBackground(highlight ? REPLACED_TIMESTAMP_BACKGROUND : NORMAL_BACKGROUND);
               lastReplacedTimestampHighlight = highlight;
            }
         }
      }
   }

   /**
    * Downscales {@code source} into a new, small {@link WritableImage} for thumbnail display via
    * nearest-neighbor sampling.
    * <p>
    * {@code thumbnail}'s {@code fitWidth} only controls how large the image is drawn - it doesn't
    * shrink what gets uploaded to the GPU as a texture. Handing the full-resolution decoded frame
    * (e.g. 1280x720) straight to a small {@code ImageView} was forcing a full-resolution texture
    * upload every pulse, for every camera, just to show a strip of tiny thumbnails.
    * <p>
    * Always allocates a fresh image rather than mutating a reused one in place: {@code ImageView}
    * only re-renders on an actual reference change (a plain, non-animated {@code WritableImage}
    * doesn't get a change listener registered for in-place pixel mutations - see
    * {@code ImageView#imageProperty()}), and at thumbnail resolution the allocation itself is
    * negligible (unlike the full-resolution image this method reads from, which is why that one
    * *is* reused - see {@code FrameImageConverter}).
    *
    * @param source       the decoded, full-resolution frame to downscale
    * @param targetWidth  the thumbnail's pixel width; must be positive
    * @return a new {@link WritableImage}, {@code targetWidth} wide, matching {@code source}'s aspect ratio
    */
   static WritableImage createThumbnailImage(WritableImage source, int targetWidth)
   {
      int sourceWidth = (int) source.getWidth();
      int sourceHeight = (int) source.getHeight();
      int width = Math.max(1, targetWidth);
      int height = Math.max(1, Math.round((float) width * sourceHeight / sourceWidth));

      WritableImage scaled = new WritableImage(width, height);
      PixelReader sourceReader = source.getPixelReader();
      PixelWriter destWriter = scaled.getPixelWriter();

      for (int y = 0; y < height; y++)
      {
         int sourceY = Math.min(sourceHeight - 1, y * sourceHeight / height);

         for (int x = 0; x < width; x++)
         {
            int sourceX = Math.min(sourceWidth - 1, x * sourceWidth / width);
            destWriter.setArgb(x, y, sourceReader.getArgb(sourceX, sourceY));
         }
      }

      return scaled;
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

   public VideoDataReader getReader()
   {
      return reader;
   }
}
