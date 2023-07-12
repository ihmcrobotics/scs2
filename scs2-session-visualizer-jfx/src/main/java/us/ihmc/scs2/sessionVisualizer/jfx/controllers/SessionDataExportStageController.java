package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.VBox;
import javafx.stage.DirectoryChooser;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.MessageBidirectionalBinding;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.DataFormat;

public class SessionDataExportStageController implements VisualizerController
{
   @FXML
   private Stage stage;
   @FXML
   private VBox mainPane;
   @FXML
   private Slider currentBufferIndexSlider;
   @FXML
   private ToggleButton exportRobotDefinitionToggleButton;
   @FXML
   private ToggleButton exportTerrainDefinitionToggleButton;
   @FXML
   private ToggleButton exportYoGraphicsDefinitionToggleButton;
   @FXML
   private ToggleButton exportRobotStateToggleButton;
   @FXML
   private ToggleButton exportDataToggleButton;
   @FXML
   private ComboBox<DataFormat> dataFormatComboBox;
   @FXML
   private SessionVariableFilterPaneController variableFilterPaneController;

   private final Property<SessionMode> currentSessionMode = new SimpleObjectProperty<>(this, "currentSessionMode", null);
   private final Property<YoBufferPropertiesReadOnly> bufferProperties = new SimpleObjectProperty<>(this, "bufferProperties", null);

   private final List<Runnable> cleanupActions = new ArrayList<>();

   private Window owner;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      owner = toolkit.getWindow();
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();

      MessageBidirectionalBinding<SessionMode, SessionMode> currentSessionModeBinding = messager.bindBidirectional(topics.getSessionCurrentMode(),
                                                                                                                   currentSessionMode,
                                                                                                                   false);
      cleanupActions.add(() ->
      {
         messager.removeFXTopicListener(topics.getSessionCurrentMode(), currentSessionModeBinding);
         currentSessionMode.removeListener(currentSessionModeBinding);
      });

      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
      MutableBoolean updatingBufferIndex = new MutableBoolean(false);
      TopicListener<YoBufferPropertiesReadOnly> bufferPropertiesBinding = messager.bindPropertyToTopic(topics.getYoBufferCurrentProperties(), bufferProperties);
      cleanupActions.add(() -> messager.removeFXTopicListener(topics.getYoBufferCurrentProperties(), bufferPropertiesBinding));

      ChangeListener<? super SessionMode> currentSessionModeChangeListener = (o, oldValue, newValue) ->
      {
         if (newValue != SessionMode.PAUSE)
         {
            messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
         }
         else if (bufferProperties.getValue() != null)
         {
            currentBufferIndexSlider.setMax(bufferProperties.getValue().getSize());
            updatingBufferIndex.setTrue();
            currentBufferIndexSlider.setValue(bufferProperties.getValue().getCurrentIndex());
            updatingBufferIndex.setFalse();
         }
      };
      currentSessionMode.addListener(currentSessionModeChangeListener);
      cleanupActions.add(() -> currentSessionMode.removeListener(currentSessionModeChangeListener));

      TopicListener<YoBufferPropertiesReadOnly> bufferPropertiesTopicListener = m ->
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
      messager.addFXTopicListener(topics.getYoBufferCurrentProperties(), bufferPropertiesTopicListener);
      cleanupActions.add(() -> messager.removeFXTopicListener(topics.getYoBufferCurrentProperties(), bufferPropertiesTopicListener));

      ChangeListener<? super Number> bufferIndexSliderListener = (o, oldValue, newValue) ->
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
      currentBufferIndexSlider.valueProperty().addListener(bufferIndexSliderListener);
      cleanupActions.add(() -> currentBufferIndexSlider.valueProperty().removeListener(bufferIndexSliderListener));

      dataFormatComboBox.setItems(FXCollections.observableArrayList(DataFormat.values()));
      dataFormatComboBox.getSelectionModel().select(DataFormat.ASCII);

      variableFilterPaneController.initialize(toolkit.getGlobalToolkit());
      cleanupActions.add(variableFilterPaneController::dispose);

      EventHandler<? super WindowEvent> closeWindowEventHandler = e -> close();
      owner.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler);
      cleanupActions.add(() -> owner.removeEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler));

      stage.setOnCloseRequest(e -> close());

      SessionVisualizerIOTools.addSCSIconToWindow(stage);
      JavaFXMissingTools.centerWindowInOwner(stage, owner);
   }

   public Stage getStage()
   {
      return stage;
   }

   public void close()
   {
      stage.close();
      cleanupActions.forEach(Runnable::run);
      cleanupActions.clear();
   }

   @FXML
   void cancel(ActionEvent event)
   {
      close();
   }

   @FXML
   void exportData(ActionEvent event)
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();
      directoryChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath("export-data"));
      File result = directoryChooser.showDialog(owner);

      if (result == null)
         return;

      SessionVisualizerIOTools.setDefaultFilePath("export-data", result);
      SessionDataExportRequest request = new SessionDataExportRequest();
      request.setFile(result);
      request.setOverwrite(true);
      request.setVariableFilter(variableFilterPaneController.buildVariableFilter());
      request.setRegistryFilter(variableFilterPaneController.buildRegistryFilter());
      request.setExportRobotDefinitions(exportRobotDefinitionToggleButton.isSelected());
      request.setExportTerrainObjectDefinitions(exportTerrainDefinitionToggleButton.isSelected());
      request.setExportSessionYoGraphicDefinitions(exportYoGraphicsDefinitionToggleButton.isSelected());
      request.setExportRobotStateDefinitions(exportRobotStateToggleButton.isSelected());
      request.setExportSessionBufferRegistryDefinition(exportDataToggleButton.isSelected());
      if (exportDataToggleButton.isSelected())
         request.setExportSessionBufferDataFormat(dataFormatComboBox.getSelectionModel().getSelectedItem());
      else
         request.setExportSessionBufferDataFormat(null);
      request.setOnExportStartCallback(() -> messager.submitMessage(topics.getDisableUserControls(), true));
      request.setOnExportEndCallback(() -> messager.submitMessage(topics.getDisableUserControls(), false));
      close();
      messager.submitMessage(topics.getSessionDataExportRequest(), request);
   }
}
