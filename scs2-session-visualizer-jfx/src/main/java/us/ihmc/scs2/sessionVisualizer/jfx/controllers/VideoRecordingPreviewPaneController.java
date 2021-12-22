package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.io.File;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.SnapshotParameters;
import javafx.scene.SubScene;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Scale;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SceneVideoRecordingRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.IntegerConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

public class VideoRecordingPreviewPaneController
{
   @FXML
   private Stage stage;
   @FXML
   private VBox mainPane;
   @FXML
   private AnchorPane imageViewContainer;
   @FXML
   private ImageView imageView;
   @FXML
   private Rectangle viewportRectangle;
   @FXML
   private Slider currentBufferIndexSlider;
   @FXML
   private ComboBox<String> resolutionComboBox;
   @FXML
   private TextField frameRateTextField, realTimeRateTextField;
   @FXML
   private TextField startIndexTextField, endIndexTextField;

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

   private final IntegerProperty width = new SimpleIntegerProperty(this, "width", Resolution.FULL_HD_1920x1080.getWidth());
   private final IntegerProperty height = new SimpleIntegerProperty(this, "height", Resolution.FULL_HD_1920x1080.getHeight());
   private Property<Integer> frameRate;
   private Property<Double> realTimeRate;
   private Property<Integer> startIndex, endIndex;

   private final Property<SnapshotParameters> paramsProperty = new SimpleObjectProperty<>(this, "params", new SnapshotParameters());
   private final Property<SessionMode> currentSessionMode = new SimpleObjectProperty<>(this, "currentSessionMode", null);
   private Property<YoBufferPropertiesReadOnly> bufferProperties;

   private Window owner;

   private SubScene targetScene;
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;

   public void initialize(Window owner, SubScene targetScene, JavaFXMessager messager, SessionVisualizerTopics topics)
   {
      this.owner = owner;
      this.targetScene = targetScene;
      this.messager = messager;
      this.topics = topics;

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
      TextFormatter<Integer> startFormatter = new TextFormatter<>(new IntegerConverter(), -1, createBufferIndexFilter());
      TextFormatter<Integer> endFormatter = new TextFormatter<>(new IntegerConverter(), -1, createBufferIndexFilter());
      frameRateTextField.setTextFormatter(frameRateFormatter);
      realTimeRateTextField.setTextFormatter(realTimeRateFormatter);
      startIndexTextField.setTextFormatter(startFormatter);
      endIndexTextField.setTextFormatter(endFormatter);

      frameRate = frameRateFormatter.valueProperty();
      realTimeRate = realTimeRateFormatter.valueProperty();
      startIndex = startFormatter.valueProperty();
      endIndex = endFormatter.valueProperty();

      width.addListener((o, oldValue, newValue) -> updatePreview());
      height.addListener((o, oldValue, newValue) -> updatePreview());
      frameRate.addListener((o, oldValue, newValue) -> updatePreview());
      realTimeRate.addListener((o, oldValue, newValue) -> updatePreview());
      stage.showingProperty().addListener((o, oldValue, newValue) -> updatePreview());
      currentBufferIndexSlider.valueProperty().addListener((o, oldValue, newValue) -> updatePreview());

      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
      MutableBoolean updatingBufferIndex = new MutableBoolean(false);
      bufferProperties = messager.createPropertyInput(topics.getYoBufferCurrentProperties());

      currentSessionMode.addListener((o, oldValue, newValue) ->
      {
         if (newValue != SessionMode.PAUSE)
         {
            messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
         }
         else if (bufferProperties.getValue() != null)
         {
            currentBufferIndexSlider.setMax(bufferProperties.getValue().getSize());
            if (startIndex.getValue() == -1)
               startIndex.setValue(bufferProperties.getValue().getInPoint());
            if (endIndex.getValue() == -1)
               endIndex.setValue(bufferProperties.getValue().getOutPoint());

            updatingBufferIndex.setTrue();
            currentBufferIndexSlider.setValue(bufferProperties.getValue().getCurrentIndex());
            updatingBufferIndex.setFalse();
         }
      });

      messager.registerJavaFXSyncedTopicListener(topics.getYoBufferCurrentProperties(), m ->
      {
         if (currentSessionMode.getValue() != SessionMode.PAUSE)
            return;

         currentBufferIndexSlider.setMax(m.getSize());
         if (startIndex.getValue() == -1)
            startIndex.setValue(m.getInPoint());
         if (endIndex.getValue() == -1)
            endIndex.setValue(m.getOutPoint());

         if (updatingBufferIndex.isFalse())
         {
            updatingBufferIndex.setTrue();
            currentBufferIndexSlider.setValue(m.getCurrentIndex());
            updatingBufferIndex.setFalse();
         }
      });

      currentBufferIndexSlider.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (currentSessionMode.getValue() != SessionMode.PAUSE)
            return;

         if (updatingBufferIndex.isFalse())
         {
            updatingBufferIndex.setTrue();
            messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), newValue.intValue());
            updatingBufferIndex.setFalse();
         }
      });

      stage.setTitle("Video export preview and properties");
      owner.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e -> close());
   }

   public void close()
   {
      stage.close();
   }

   private UnaryOperator<Change> createBufferIndexFilter()
   {
      return change ->
      {
         try
         {
            int index = Integer.parseInt(change.getControlNewText());
            if (index < -1)
               return null;
            if (index >= bufferProperties.getValue().getSize())
               return null;
            return change;
         }
         catch (NumberFormatException e)
         {
            return null;
         }
      };
   }

   public Stage getStage()
   {
      return stage;
   }

   private void updatePreview()
   {
      double inputWidth = targetScene.getWidth();
      double inputHeight = targetScene.getHeight();
      double outputWidth;
      double outputHeight;

      if (imageView.getFitWidth() > 0)
      {
         double scale = imageView.getFitWidth() / inputWidth;
         outputWidth = scale * inputWidth;
         outputHeight = scale * inputHeight;
      }
      else if (imageView.getFitHeight() > 0)
      {
         double scale = imageView.getFitHeight() / inputHeight;
         outputWidth = scale * inputWidth;
         outputHeight = scale * inputHeight;
      }
      else
      {
         outputWidth = inputWidth;
         outputHeight = inputHeight;
      }

      double outputRatio = width.getValue().doubleValue() / height.getValue().doubleValue();
      double inputRatio = inputWidth / inputHeight;

      computeTransform(outputWidth, outputHeight, inputWidth, inputHeight, paramsProperty.getValue());

      JavaFXMissingTools.runLaterIfNeeded(this.getClass(), () ->
      {
         WritableImage snapshot = targetScene.snapshot(paramsProperty.getValue(), null);
         imageView.setImage(snapshot);
         computeViewport(outputRatio, inputRatio, outputWidth, outputHeight, viewportRectangle);
         stage.sizeToScene();
      });
   }

   @FXML
   private void setIndicesToInOut()
   {
      startIndex.setValue(bufferProperties.getValue().getInPoint());
      endIndex.setValue(bufferProperties.getValue().getOutPoint());
   }

   @FXML
   void cancel(ActionEvent event)
   {
      close();
   }

   @FXML
   void exportVideo(ActionEvent event)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath("video"));
      fileChooser.getExtensionFilters().add(new ExtensionFilter("MP4", "*.mp4"));
      File result = fileChooser.showSaveDialog(owner);

      if (result == null)
         return;

      SessionVisualizerIOTools.setDefaultFilePath("video", result);
      SceneVideoRecordingRequest request = new SceneVideoRecordingRequest();
      request.setFile(result);
      request.setBufferStart(startIndex.getValue());
      request.setBufferEnd(endIndex.getValue());
      request.setFrameRate(frameRate.getValue());
      request.setRealTimeRate(realTimeRate.getValue());
      request.setWidth(width.get());
      request.setHeight(height.get());
      close();
      messager.submitMessage(topics.getSceneVideoRecordingRequest(), request);
   }

   private static void computeViewport(double outputRatio, double inputRatio, double imageWidth, double imageHeight, Rectangle rectangle)
   {
      double minX = 0.0;
      double minY = 0.0;
      double width = imageWidth;
      double height = imageHeight;

      if (!EuclidCoreTools.epsilonEquals(inputRatio, outputRatio, 1.0e-6))
      {
         if (inputRatio > outputRatio)
         { // Need to reduce the width
            double adjustedWidth = imageHeight * outputRatio;
            minX = 0.5 * (imageWidth - adjustedWidth);
            minY = 0.0;
            width = adjustedWidth;
            height = imageHeight;
         }
         else
         { // Need to reduce the height
            double adjustedHeight = imageWidth / outputRatio;
            minX = 0.0;
            minY = 0.5 * (imageHeight - adjustedHeight);
            width = imageWidth;
            height = adjustedHeight;
         }
      }

      rectangle.setX(minX);
      rectangle.setY(minY);
      rectangle.setWidth(width);
      rectangle.setHeight(height);
   }

   private static SnapshotParameters computeTransform(double outputWidth, double outputHeight, double inputWidth, double inputHeight, SnapshotParameters params)
   {
      double widthScale = outputWidth / inputWidth;
      double heightScale = outputHeight / inputHeight;
      double scale = Math.min(widthScale, heightScale);
      params.setTransform(new Scale(scale, scale));
      return params;
   }
}