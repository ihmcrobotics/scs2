package us.ihmc.scs2.examples.video;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Slider;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber;
import org.bytedeco.javacv.JavaFXFrameConverter;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.log.LogTools;

import java.io.File;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * similar to https://github.com/bytedeco/javacv/blob/master/samples/JavaFxPlayVideoAndAudio.java
 */
public class VideoViewerWithSlider
{
   {
      // TODO: (AM) hack to suppress warnings from https://github.com/bytedeco/javacv/issues/780
      avutil.av_log_set_level(avutil.AV_LOG_ERROR);
   }

   private File videoFile = null;
   private ImageView imageView = null;
   private FFmpegFrameGrabber frameGrabber = null;
   private Slider timeSlider = null;
   private AtomicInteger currentTimestamp = new AtomicInteger(-1);
   private final ExecutorService videoExecutor = Executors.newSingleThreadExecutor();

   public void init(Stage primaryStage)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().addAll(new FileChooser.ExtensionFilter("Video Files", "*.mp4", "*.avi"));
      this.videoFile = fileChooser.showOpenDialog(primaryStage);

      frameGrabber = new FFmpegFrameGrabber(videoFile);
      try
      {
         frameGrabber.start();
      }
      catch (FFmpegFrameGrabber.Exception e)
      {
         LogTools.error(e.getMessage());
      }


      final BorderPane root = new BorderPane();
      root.setPrefSize(1200, 800);
      imageView = new ImageView();
      StackPane imageContainer = new StackPane(imageView);
      imageView.setPreserveRatio(true);
      imageView.fitWidthProperty().bind(imageContainer.widthProperty());
      imageView.fitHeightProperty().bind(imageContainer.heightProperty());
      root.setCenter(imageContainer);
      BorderPane.setAlignment(imageContainer, Pos.CENTER);

      HBox mediaBar = new HBox();
      mediaBar.setAlignment(Pos.CENTER);
      mediaBar.setPadding(new Insets(5, 10, 5, 10));
      BorderPane.setAlignment(mediaBar, Pos.CENTER);

      timeSlider = new Slider();
      HBox.setHgrow(timeSlider, Priority.NEVER);
      timeSlider.setMin(0);
      timeSlider.setMax(frameGrabber.getLengthInVideoFrames());
      timeSlider.setPrefWidth(600);
      mediaBar.getChildren().add(timeSlider);

      root.setBottom(mediaBar);

      final Scene scene = new Scene(root);

      primaryStage.setTitle(videoFile.getName());
      primaryStage.setScene(scene);
      primaryStage.setOnCloseRequest(event -> this.stop());

      primaryStage.show();
   }

   public void playVideoUsingSlider()
   {
      final JavaFXFrameConverter frameConverter = new JavaFXFrameConverter();

      timeSlider.valueProperty().addListener((observable, oldValue, newValue) ->
                                             {
                                                timeSlider.setValue(newValue.intValue()); // Snap the slider motion to integer increments
                                                currentTimestamp.set(newValue.intValue());
                                             });
      videoExecutor.submit(() ->
                           {
                              while (!Thread.interrupted())
                              {
                                 try
                                 {
                                    frameGrabber.setVideoFrameNumber(currentTimestamp.get());
                                    Frame frame = frameGrabber.grabFrame();
                                    if (frame == null)
                                    {
                                       continue;
                                    }

                                    if (frame.image != null)
                                    {
                                       Platform.runLater(new Runnable()
                                       {
                                          final Image image = frameConverter.convert(frame);
                                          @Override
                                          public void run()
                                          {
                                             imageView.setImage(image);
                                          }
                                       });
                                    }
                                 }
                                 catch (FrameGrabber.Exception e)

                                 {
                                    LogTools.error(e.getMessage());
                                 }
                              }
                           });
   }

   public void stop() throws RuntimeException
   {
      try
      {
         videoExecutor.shutdownNow();
         videoExecutor.awaitTermination(1, TimeUnit.SECONDS);
         frameGrabber.stop();
         frameGrabber.release();
      }
      catch (FFmpegFrameGrabber.Exception | InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      Platform.setImplicitExit(true);
      ApplicationRunner.runApplication(new Application()
      {
         VideoViewerWithSlider demo = new VideoViewerWithSlider();

         @Override
         public void start(Stage primaryStage) throws Exception
         {
            demo.init(primaryStage);
            demo.playVideoUsingSlider();
         }
      });
   }
}
