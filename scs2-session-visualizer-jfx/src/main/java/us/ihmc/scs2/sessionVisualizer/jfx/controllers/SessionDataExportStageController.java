package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.function.UnaryOperator;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.controlsfx.control.CheckTreeView;

import com.jfoenix.controls.JFXButton;
import com.jfoenix.controls.JFXComboBox;
import com.jfoenix.controls.JFXSlider;
import com.jfoenix.controls.JFXTextField;
import com.jfoenix.controls.JFXToggleButton;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBoxTreeItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;
import javafx.scene.control.TreeItem;
import javafx.scene.control.cell.CheckBoxTreeCell;
import javafx.scene.layout.HBox;
import javafx.stage.DirectoryChooser;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.IntegerConverter;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.DataFormat;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class SessionDataExportStageController
{
   @FXML
   private Stage stage;
   @FXML
   private HBox mainPane;
   @FXML
   private CheckTreeView<Object> selectedVariablesCheckTreeView;
   @FXML
   private JFXButton selectAllButton, unselectAllButton;
   @FXML
   private JFXSlider currentBufferIndexSlider;
   @FXML
   private JFXTextField inPointTextField, outPointTextField;
   @FXML
   private JFXToggleButton exportRobotDefinitionToggleButton;
   @FXML
   private JFXToggleButton exportTerrainDefinitionToggleButton;
   @FXML
   private JFXToggleButton exportYoGraphicsDefinitionToggleButton;
   @FXML
   private JFXToggleButton exportDataToggleButton;
   @FXML
   private JFXComboBox<DataFormat> dataFormatComboBox;

   private Property<Integer> inPointIndex, outPointIndex;

   private final Property<SessionMode> currentSessionMode = new SimpleObjectProperty<>(this, "currentSessionMode", null);
   private final Property<YoBufferPropertiesReadOnly> bufferProperties = new SimpleObjectProperty<>(this, "bufferProperties", null);

   private final List<Runnable> cleanupActions = new ArrayList<>();

   private CheckBoxTreeItem<Object> rootItem;

   private Window owner;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private YoManager yoManager;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      owner = toolkit.getWindow();
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      yoManager = toolkit.getYoManager();

      MessageBidirectionalBinding<SessionMode, SessionMode> currentSessionModeBinding = messager.bindBidirectional(topics.getSessionCurrentMode(),
                                                                                                                   currentSessionMode,
                                                                                                                   false);
      cleanupActions.add(() ->
      {
         messager.removeJavaFXSyncedTopicListener(topics.getSessionCurrentMode(), currentSessionModeBinding);
         currentSessionMode.removeListener(currentSessionModeBinding);
      });

      TextFormatter<Integer> inPointFormatter = new TextFormatter<>(new IntegerConverter(), -1, createBufferIndexFilter());
      TextFormatter<Integer> outPointFormatter = new TextFormatter<>(new IntegerConverter(), -1, createBufferIndexFilter());
      inPointTextField.setTextFormatter(inPointFormatter);
      outPointTextField.setTextFormatter(outPointFormatter);

      inPointIndex = inPointFormatter.valueProperty();
      outPointIndex = outPointFormatter.valueProperty();

      messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
      MutableBoolean updatingBufferIndex = new MutableBoolean(false);
      TopicListener<YoBufferPropertiesReadOnly> bufferPropertiesBinding = messager.bindPropertyToTopic(topics.getYoBufferCurrentProperties(), bufferProperties);
      cleanupActions.add(() -> messager.removeJavaFXSyncedTopicListener(topics.getYoBufferCurrentProperties(), bufferPropertiesBinding));

      ChangeListener<? super SessionMode> currentSessionModeChangeListener = (o, oldValue, newValue) ->
      {
         if (newValue != SessionMode.PAUSE)
         {
            messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
         }
         else if (bufferProperties.getValue() != null)
         {
            currentBufferIndexSlider.setMax(bufferProperties.getValue().getSize());
            if (inPointIndex.getValue() == -1)
               inPointIndex.setValue(bufferProperties.getValue().getInPoint());
            if (outPointIndex.getValue() == -1)
               outPointIndex.setValue(bufferProperties.getValue().getOutPoint());

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
         if (inPointIndex.getValue() == -1)
            inPointIndex.setValue(m.getInPoint());
         if (outPointIndex.getValue() == -1)
            outPointIndex.setValue(m.getOutPoint());

         if (updatingBufferIndex.isFalse())
         {
            updatingBufferIndex.setTrue();
            currentBufferIndexSlider.setValue(m.getCurrentIndex());
            updatingBufferIndex.setFalse();
         }
      };
      messager.registerJavaFXSyncedTopicListener(topics.getYoBufferCurrentProperties(), bufferPropertiesTopicListener);
      cleanupActions.add(() -> messager.removeJavaFXSyncedTopicListener(topics.getYoBufferCurrentProperties(), bufferPropertiesTopicListener));

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

      selectedVariablesCheckTreeView.setCellFactory(param -> new CheckBoxTreeCell<Object>()
      {
         @Override
         public void updateItem(Object item, boolean empty)
         {
            super.updateItem(item, empty);

            if (empty)
            {
               setText(null);
               setGraphic(null);
            }
            else
            {
               if (item instanceof YoRegistry)
               {
                  setText(((YoRegistry) item).getName());
               }
               else if (item instanceof YoVariable)
               {
                  setText(((YoVariable) item).getName());
               }
               else
               {
                  throw new IllegalStateException("Unexpected item type: " + item.getClass());
               }
            }
         }
      });
      selectedVariablesCheckTreeView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);
      selectedVariablesCheckTreeView.setShowRoot(true);
      YoRegistryChangedListener rootRegistryListener = change -> refreshTreeView();
      yoManager.getRootRegistry().addListener(rootRegistryListener);
      cleanupActions.add(() -> yoManager.getRootRegistry().removeListener(rootRegistryListener));
      refreshTreeView();

      EventHandler<? super WindowEvent> closeWindowEventHandler = e -> close();
      owner.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler);
      cleanupActions.add(() -> owner.removeEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler));

      stage.setOnCloseRequest(e -> close());
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

   private void refreshTreeView()
   {
      rootItem = new CheckBoxTreeItem<Object>(yoManager.getRootRegistry());
      rootItem.setSelected(true);
      rootItem.setExpanded(true);
      buildTreeRecursively(rootItem);
      selectedVariablesCheckTreeView.setRoot(rootItem);
   }

   private void buildTreeRecursively(TreeItem<Object> parent)
   {
      Object value = parent.getValue();

      if (value instanceof YoRegistry)
      {
         YoRegistry registry = (YoRegistry) value;
         for (YoVariable variable : registry.getVariables())
         {
            CheckBoxTreeItem<Object> childItem = new CheckBoxTreeItem<>(variable);
            childItem.setSelected(true);
            childItem.setExpanded(true);
            parent.getChildren().add(childItem);
         }

         for (YoRegistry childRegistry : registry.getChildren())
         {
            CheckBoxTreeItem<Object> childItem = new CheckBoxTreeItem<>(childRegistry);
            childItem.setSelected(true);
            childItem.setExpanded(true);
            parent.getChildren().add(childItem);
            buildTreeRecursively(childItem);
         }
      }
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
   void setInOutToCurrent()
   {
      inPointIndex.setValue(bufferProperties.getValue().getInPoint());
      outPointIndex.setValue(bufferProperties.getValue().getOutPoint());
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
      request.setInPoint(inPointIndex.getValue());
      request.setOutPoint(outPointIndex.getValue());
      // TODO
      //      request.setRecordPeriod(???);
      request.setVariableFilter(buildVariableFilter());
      request.setRegistryFilter(buildRegistryFilter());
      request.setExportRobotDefinitions(exportRobotDefinitionToggleButton.isSelected());
      request.setExportTerrainObjectDefinitions(exportTerrainDefinitionToggleButton.isSelected());
      request.setExportSessionYoGraphicDefinitions(exportYoGraphicsDefinitionToggleButton.isSelected());
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

   private Predicate<YoVariable> buildVariableFilter()
   {
      Set<String> selectedFullnames = collectSelectedVariableFullnames(rootItem, null);
      return var -> selectedFullnames.contains(var.getFullNameString());
   }

   private Set<String> collectSelectedVariableFullnames(CheckBoxTreeItem<Object> current, Set<String> fullnamesToPack)
   {
      if (current.getValue() instanceof YoRegistry)
      {
         if (!current.isSelected() && !current.isIndeterminate())
            return fullnamesToPack;

         for (TreeItem<Object> childItem : current.getChildren())
         {
            fullnamesToPack = collectSelectedVariableFullnames((CheckBoxTreeItem<Object>) childItem, fullnamesToPack);
         }
      }
      else if (current.getValue() instanceof YoVariable)
      {
         if (!current.isSelected())
            return fullnamesToPack;

         if (fullnamesToPack == null)
            fullnamesToPack = new LinkedHashSet<>();
         fullnamesToPack.add(((YoVariable) current.getValue()).getFullNameString());
      }

      return fullnamesToPack;
   }

   private Predicate<YoRegistry> buildRegistryFilter()
   {
      Set<String> selectedFullnames = collectSelectedRegistryFullnames(rootItem, null);
      return var -> selectedFullnames.contains(var.getNamespace().getName());
   }

   private Set<String> collectSelectedRegistryFullnames(CheckBoxTreeItem<Object> current, Set<String> fullnamesToPack)
   {
      if (current.getValue() instanceof YoRegistry)
      {
         if (!current.isSelected() && !current.isIndeterminate())
            return fullnamesToPack;

         if (fullnamesToPack == null)
            fullnamesToPack = new LinkedHashSet<>();
         fullnamesToPack.add(((YoRegistry) current.getValue()).getNamespace().getName());

         for (TreeItem<Object> childItem : current.getChildren())
         {
            fullnamesToPack = collectSelectedRegistryFullnames((CheckBoxTreeItem<Object>) childItem, fullnamesToPack);
         }
      }

      return fullnamesToPack;
   }
}
