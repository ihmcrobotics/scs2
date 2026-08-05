package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.application.Platform;
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
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.FXCoalescedUpdater;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.DataFormat;

import java.util.function.Consumer;

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
   private SCS2Messager messager;
   private Session session;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      owner = toolkit.getWindow();
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      session = toolkit.getSession();

      Consumer<SessionProperties> sessionModeListener = properties -> Platform.runLater(() -> currentSessionMode.setValue(properties.getActiveMode()));
      session.addSessionPropertiesListener(sessionModeListener);
      sessionModeListener.accept(session.getSessionProperties());
      ChangeListener<SessionMode> currentSessionModeSubmitListener = (o, oldValue, newValue) -> session.setSessionMode(newValue);
      currentSessionMode.addListener(currentSessionModeSubmitListener);
      cleanupActions.add(() ->
      {
         session.removeSessionPropertiesListener(sessionModeListener);
         currentSessionMode.removeListener(currentSessionModeSubmitListener);
      });

      session.setSessionMode(SessionMode.PAUSE);
      MutableBoolean updatingBufferIndex = new MutableBoolean(false);
      FXCoalescedUpdater<YoBufferPropertiesReadOnly> bufferPropertiesUpdater = new FXCoalescedUpdater<>(bufferProperties::setValue);
      Consumer<YoBufferPropertiesReadOnly> bufferPropertiesBinding = bufferPropertiesUpdater::update;
      session.addCurrentBufferPropertiesListener(bufferPropertiesBinding);
      cleanupActions.add(() -> session.removeCurrentBufferPropertiesListener(bufferPropertiesBinding));

      ChangeListener<? super SessionMode> currentSessionModeChangeListener = (o, oldValue, newValue) ->
      {
         if (newValue != SessionMode.PAUSE)
         {
            session.setSessionMode(SessionMode.PAUSE);
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

      FXCoalescedUpdater<YoBufferPropertiesReadOnly> bufferPropertiesTopicUpdater = new FXCoalescedUpdater<>(m ->
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
      });
      Consumer<YoBufferPropertiesReadOnly> bufferPropertiesTopicListener = bufferPropertiesTopicUpdater::update;
      session.addCurrentBufferPropertiesListener(bufferPropertiesTopicListener);
      cleanupActions.add(() -> session.removeCurrentBufferPropertiesListener(bufferPropertiesTopicListener));

      ChangeListener<? super Number> bufferIndexSliderListener = (o, oldValue, newValue) ->
      {
         if (currentSessionMode.getValue() != SessionMode.PAUSE)
            return;

         if (updatingBufferIndex.isFalse())
         {
            updatingBufferIndex.setTrue();
            session.submitBufferIndexRequest(newValue.intValue());
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
      session.submitSessionDataExportRequest(request);
   }
}
