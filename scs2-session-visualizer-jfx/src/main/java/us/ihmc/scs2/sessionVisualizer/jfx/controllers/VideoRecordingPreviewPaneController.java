package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.animation.AnimationTimer;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.geometry.Rectangle2D;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SnapshotParameters;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SceneVideoRecordingRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.IntegerConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

public class VideoRecordingPreviewPaneController
{
   @FXML
   private VBox mainPane;
   @FXML
   private AnchorPane imageViewContainer;
   @FXML
   private ImageView imageView;
   @FXML
   private Slider currentBufferIndexSlider;
   @FXML
   private ComboBox<String> resolutionComboBox;
   @FXML
   private TextField frameRateTextField, realTimeRateTextField;

   private enum Resolution
   {
      SD_640x480("SD (640 x 480)", 640, 480),
      HD_1280x720("HD (1280 x 720)", 1280, 720),
      FULL_HD_1920x1080("Full HD (1920 x 1080)", 1920, 1080),
      QUAD_HD_2560x1440("Quad HD (2560 x 1440)", 2560, 1440);

      private final static Map<String, Resolution> descriptionToEnumConstant = Arrays.stream(values())
                                                                                     .collect(Collectors.toMap(e -> e.getDescription(), Function.identity()));
      private final static String[] allDescriptions = Arrays.stream(values()).map(Resolution::getDescription).toArray(String[]::new);

      private final String description;
      private final int width;
      private final int height;

      Resolution(String description, int width, int height)
      {
         this.description = description;
         this.width = width;
         this.height = height;
      }

      public String getDescription()
      {
         return description;
      }

      public int getWidth()
      {
         return width;
      }

      public int getHeight()
      {
         return height;
      }

      public static Resolution fromDescription(String description)
      {
         return descriptionToEnumConstant.get(description);
      }
   }

   private WritableImage image;

   private final IntegerProperty width = new SimpleIntegerProperty(this, "width", Resolution.FULL_HD_1920x1080.getWidth());
   private final IntegerProperty height = new SimpleIntegerProperty(this, "height", Resolution.FULL_HD_1920x1080.getHeight());
   private Property<Integer> frameRate;
   private Property<Double> realTimeRate;

   private final Property<SessionMode> currentSessionMode = new SimpleObjectProperty<>(this, "currentSessionMode", null);
   private AtomicReference<YoBufferPropertiesReadOnly> bufferProperties;

   private Stage stage;
   private Group rootNode3D;
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;

   private final List<Runnable> cleanupActions = new ArrayList<>();
   private final SnapshotParameters snapshotParameters = new SnapshotParameters();

   private final AnimationTimer refreshViewAnimation = new AnimationTimer()
   {
      @Override
      public void handle(long now)
      {
         refreshView();
      }
   };

   public void initialize(Window owner, Group mainView3DRoot, PerspectiveCamera targetCamera, JavaFXMessager messager, SessionVisualizerTopics topics)
   {
      this.rootNode3D = mainView3DRoot;
      this.messager = messager;
      this.topics = topics;

      snapshotParameters.setDepthBuffer(true);
      snapshotParameters.setCamera(targetCamera);
      snapshotParameters.setFill(Color.GRAY);

      resolutionComboBox.setItems(FXCollections.observableArrayList(Resolution.allDescriptions));
      resolutionComboBox.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) ->
      {
         Resolution resolution = Resolution.fromDescription(newValue);
         width.set(resolution.getWidth());
         height.set(resolution.getHeight());
      });
      resolutionComboBox.getSelectionModel().select(Resolution.FULL_HD_1920x1080.getDescription());

      messager.bindBidirectional(topics.getSessionCurrentMode(), currentSessionMode, false);

      TextFormatter<Integer> frameRateFormatter = new TextFormatter<>(new IntegerConverter(), 60, new PositiveIntegerValueFilter());
      TextFormatter<Double> realTimeRateFormatter = new TextFormatter<>(new DoubleStringConverter(), 1.0);
      frameRateTextField.setTextFormatter(frameRateFormatter);
      realTimeRateTextField.setTextFormatter(realTimeRateFormatter);

      frameRate = frameRateFormatter.valueProperty();
      realTimeRate = realTimeRateFormatter.valueProperty();

      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
      MutableBoolean updatingBufferIndex = new MutableBoolean(false);
      bufferProperties = messager.createInput(topics.getYoBufferCurrentProperties());
      cleanupActions.add(() -> messager.removeInput(topics.getYoBufferCurrentProperties(), bufferProperties));

      ChangeListener<? super SessionMode> currentSessionModeListener = (o, oldValue, newValue) ->
      {
         if (newValue != SessionMode.PAUSE)
         {
            messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
         }
         else if (bufferProperties.get() != null)
         {
            currentBufferIndexSlider.setMax(bufferProperties.get().getSize());
            updatingBufferIndex.setTrue();
            currentBufferIndexSlider.setValue(bufferProperties.get().getCurrentIndex());
            updatingBufferIndex.setFalse();
         }
      };
      currentSessionMode.addListener(currentSessionModeListener);
      cleanupActions.add(() -> currentSessionMode.removeListener(currentSessionModeListener));

      TopicListener<YoBufferPropertiesReadOnly> currentBufferPropertiesListener = m ->
      {
         if (currentSessionMode.getValue() != SessionMode.PAUSE)
            return;

         currentBufferIndexSlider.setMax(m.getSize());

         if (updatingBufferIndex.isFalse())
         {
            updatingBufferIndex.setTrue();
            currentBufferIndexSlider.setValue(m.getCurrentIndex());
            updatingBufferIndex.setFalse();
         }
      };
      messager.registerJavaFXSyncedTopicListener(topics.getYoBufferCurrentProperties(), currentBufferPropertiesListener);
      cleanupActions.add(() -> messager.removeJavaFXSyncedTopicListener(topics.getYoBufferCurrentProperties(), currentBufferPropertiesListener));

      ChangeListener<? super Number> currentBufferIndexSliderListener = (o, oldValue, newValue) ->
      {
         if (currentSessionMode.getValue() != SessionMode.PAUSE)
            return;

         if (updatingBufferIndex.isFalse())
         {
            updatingBufferIndex.setTrue();
            messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), newValue.intValue());
            updatingBufferIndex.setFalse();
         }
      };
      currentBufferIndexSlider.valueProperty().addListener(currentBufferIndexSliderListener);
      cleanupActions.add(() -> currentBufferIndexSlider.valueProperty().removeListener(currentBufferIndexSliderListener));

      stage = new Stage();
      stage.setTitle("Video export preview and properties");
      stage.setScene(new Scene(mainPane));
      refreshViewAnimation.start();
      stage.setOnCloseRequest(e -> close());
      owner.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e -> close());
   }

   public void close()
   {
      cleanupActions.forEach(Runnable::run);
      cleanupActions.clear();
      refreshViewAnimation.stop();
      stage.close();
   }

   public Stage getStage()
   {
      return stage;
   }

   private void refreshView()
   {
      int localWidth = width.get();
      int localHeight = height.get();

      if (image == null || localHeight != image.getHeight() || localWidth != image.getWidth())
      {
         snapshotParameters.setViewport(new Rectangle2D(0, 0, localWidth, localHeight));
         image = new WritableImage(localWidth, localHeight);
         imageView.setImage(image);
         stage.sizeToScene();
      }

      rootNode3D.snapshot(snapshotParameters, image);
   }

   @FXML
   void cancel(ActionEvent event)
   {
      close();
   }

   @FXML
   void exportVideo(ActionEvent event)
   {
      File result = SessionVisualizerIOTools.videoExportSaveFileDialog(stage);

      if (result == null)
         return;

      SceneVideoRecordingRequest request = new SceneVideoRecordingRequest();
      request.setFile(result);
      request.setFrameRate(frameRate.getValue());
      request.setRealTimeRate(realTimeRate.getValue());
      request.setWidth(width.get());
      request.setHeight(height.get());
      close();
      messager.submitMessage(topics.getSceneVideoRecordingRequest(), request);
   }
}