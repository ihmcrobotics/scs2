package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.controlsfx.control.CheckTreeView;

import com.jfoenix.controls.JFXButton;
import com.jfoenix.controls.JFXComboBox;
import com.jfoenix.controls.JFXSlider;
import com.jfoenix.controls.JFXToggleButton;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIcon;
import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBoxTreeItem;
import javafx.scene.control.ComboBox;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TreeItem;
import javafx.scene.control.cell.CheckBoxTreeCell;
import javafx.scene.layout.VBox;
import javafx.stage.DirectoryChooser;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TreeViewTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.DataFormat;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class SessionDataExportStageController implements VisualizerController
{
   private static final String MODIFIED_FILTER_NAME = "- Modified -";
   private static final String NONE_FILTER_NAME = "None";
   private static final String ALL_FILTER_NAME = "All";

   @FXML
   private Stage stage;
   @FXML
   private VBox mainPane;
   @FXML
   private CheckTreeView<Object> selectedVariablesCheckTreeView;
   @FXML
   private JFXButton selectAllButton, unselectAllButton;
   @FXML
   private ComboBox<String> filterComboBox;
   @FXML
   private JFXSlider currentBufferIndexSlider;
   @FXML
   private JFXToggleButton exportRobotDefinitionToggleButton;
   @FXML
   private JFXToggleButton exportTerrainDefinitionToggleButton;
   @FXML
   private JFXToggleButton exportYoGraphicsDefinitionToggleButton;
   @FXML
   private JFXToggleButton exportRobotStateToggleButton;
   @FXML
   private JFXToggleButton exportDataToggleButton;
   @FXML
   private JFXComboBox<DataFormat> dataFormatComboBox;

   private SessionDataFilterParameters currentFilter;
   private final SessionDataCollectionBasedFilter modifiedFilter = new SessionDataCollectionBasedFilter(MODIFIED_FILTER_NAME);
   private final Property<SessionMode> currentSessionMode = new SimpleObjectProperty<>(this, "currentSessionMode", null);
   private final Property<YoBufferPropertiesReadOnly> bufferProperties = new SimpleObjectProperty<>(this, "bufferProperties", null);

   private final ObjectProperty<ContextMenu> activeContexMenu = new SimpleObjectProperty<>(this, "activeContextMenu", null);

   private final List<Runnable> cleanupActions = new ArrayList<>();

   private CheckBoxTreeItem<Object> rootItem;

   private Window owner;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private YoManager yoManager;

   @Override
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
      selectedVariablesCheckTreeView.setOnContextMenuRequested(e ->
      {
         if (activeContexMenu.get() != null)
         {
            activeContexMenu.get().hide();
            activeContexMenu.set(null);
         }

         FontAwesomeIconView collapseIcon = new FontAwesomeIconView(FontAwesomeIcon.MINUS_SQUARE_ALT);
         FontAwesomeIconView expandIcon = new FontAwesomeIconView(FontAwesomeIcon.PLUS_SQUARE_ALT);
         MenuItem collapseItem = new MenuItem("Collapse all", collapseIcon);
         MenuItem expandItem = new MenuItem("Expand all", expandIcon);
         collapseItem.setOnAction(e2 -> collapseAll());
         expandItem.setOnAction(e2 -> expandAll());

         ContextMenu contextMenu = new ContextMenu(collapseItem, expandItem);
         contextMenu.show(selectedVariablesCheckTreeView, e.getScreenX(), e.getScreenY());
         activeContexMenu.set(contextMenu);
      });

      selectedVariablesCheckTreeView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);
      selectedVariablesCheckTreeView.setShowRoot(true);
      YoRegistryChangedListener rootRegistryListener = change -> refreshTreeView();
      yoManager.getRootRegistry().addListener(rootRegistryListener);
      cleanupActions.add(() -> yoManager.getRootRegistry().removeListener(rootRegistryListener));
      refreshTreeView();

      SessionDataFilterParameters allFilter = new SessionDataFilterParameters(ALL_FILTER_NAME, v -> true, null);
      SessionDataFilterParameters noneFilter = new SessionDataFilterParameters(NONE_FILTER_NAME, v -> false, null);

      filterComboBox.setItems(FXCollections.observableArrayList());
      filterComboBox.getItems().addAll(MODIFIED_FILTER_NAME);
      filterComboBox.getItems().addAll(ALL_FILTER_NAME);
      filterComboBox.getItems().addAll(NONE_FILTER_NAME);
      filterComboBox.getItems().addAll(toolkit.getSessionDataPreferenceManager().getFilterMap().keySet());

      filterComboBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         if (MODIFIED_FILTER_NAME.equals(oldValue))
            updateModifiedFilterFromTreeView();

         if (MODIFIED_FILTER_NAME.equals(newValue))
            currentFilter = modifiedFilter;
         else if (ALL_FILTER_NAME.equals(newValue))
            currentFilter = allFilter;
         else if (NONE_FILTER_NAME.equals(newValue))
            currentFilter = noneFilter;
         else
            currentFilter = toolkit.getSessionDataPreferenceManager().getFilterMap().get(newValue);

         applyFilterToTreeItems(currentFilter);
      });
      filterComboBox.getSelectionModel().select(ALL_FILTER_NAME);

      selectAllButton.setOnAction(e -> filterComboBox.getSelectionModel().select(ALL_FILTER_NAME));
      unselectAllButton.setOnAction(e -> filterComboBox.getSelectionModel().select(NONE_FILTER_NAME));

      EventHandler<? super WindowEvent> closeWindowEventHandler = e -> close();
      owner.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler);
      cleanupActions.add(() -> owner.removeEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler));

      stage.setOnCloseRequest(e -> close());

      SessionVisualizerIOTools.addSCSIconToWindow(stage);
      JavaFXMissingTools.centerWindowInOwner(stage, owner);
   }

   private void applyFilterToTreeItems(SessionDataFilterParameters filter)
   {
      treeItemSelectedListenerEnabled = false;
      List<TreeItem<Object>> itemsToProcess = new ArrayList<>(Arrays.asList(rootItem));

      while (!itemsToProcess.isEmpty())
      {
         CheckBoxTreeItem<Object> item = (CheckBoxTreeItem<Object>) itemsToProcess.remove(itemsToProcess.size() - 1);

         if (item.getValue() instanceof YoVariable var)
         {
            item.setSelected(filter.getVariableFilter().test(var));
         }
         else
         {
            itemsToProcess.addAll(item.getChildren());
         }
      }

      treeItemSelectedListenerEnabled = true;
   }

   private boolean treeItemSelectedListenerEnabled = true;

   private ChangeListener<? super Boolean> treeItemSelectedListener = (o, oldValue, newValue) ->
   {
      if (treeItemSelectedListenerEnabled)
      {
         if (!MODIFIED_FILTER_NAME.equals(filterComboBox.getSelectionModel().getSelectedItem()))
         {
            updateModifiedFilterFromTreeView();
            filterComboBox.getSelectionModel().select(MODIFIED_FILTER_NAME);
         }
      }
   };

   private void refreshTreeView()
   {
      modifiedFilter.clear();
      rootItem = new CheckBoxTreeItem<>(yoManager.getRootRegistry());
      rootItem.setSelected(true);
      rootItem.setExpanded(true);
      buildTreeRecursively(rootItem);
      selectedVariablesCheckTreeView.setRoot(rootItem);
      filterComboBox.getSelectionModel().select(ALL_FILTER_NAME);
   }

   private void buildTreeRecursively(TreeItem<Object> parent)
   {
      Object value = parent.getValue();

      if (value instanceof YoRegistry)
      {
         YoRegistry registry = (YoRegistry) value;

         for (YoRegistry childRegistry : registry.getChildren())
         {
            CheckBoxTreeItem<Object> childItem = new CheckBoxTreeItem<>(childRegistry);
            childItem.setSelected(!childRegistry.getChildren().isEmpty() || !childRegistry.getVariables().isEmpty());
            childItem.setExpanded(true);
            parent.getChildren().add(childItem);
            buildTreeRecursively(childItem);
         }

         for (YoVariable variable : registry.getVariables())
         {
            CheckBoxTreeItem<Object> childItem = new CheckBoxTreeItem<>(variable);
            childItem.selectedProperty().addListener(treeItemSelectedListener);
            childItem.setSelected(true);
            childItem.setExpanded(true);
            parent.getChildren().add(childItem);
         }
      }
   }

   private void updateModifiedFilterFromTreeView()
   {
      modifiedFilter.yoVariableNames.clear();
      List<TreeItem<Object>> itemsToProcess = new ArrayList<>(Arrays.asList(rootItem));

      while (!itemsToProcess.isEmpty())
      {
         CheckBoxTreeItem<Object> item = (CheckBoxTreeItem<Object>) itemsToProcess.remove(itemsToProcess.size() - 1);

         if (item.getValue() instanceof YoVariable var)
         {
            if (item.isSelected())
               modifiedFilter.yoVariableNames.add(var.getFullNameString());
         }
         else if (item.getValue() instanceof YoRegistry reg)
         {
            if (reg.getChildren().isEmpty() && reg.getVariables().isEmpty())
               item.setSelected(false);
            else
               itemsToProcess.addAll(item.getChildren());
         }
         else
         {
            throw new IllegalStateException("Unexpected value type: " + item.getValue().getClass().getSimpleName());
         }
      }
   }

   public void collapseAll()
   {
      TreeViewTools.collapseRecursively(rootItem);
   }

   public void expandAll()
   {
      TreeViewTools.expandRecursively(rootItem);
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
      request.setVariableFilter(buildVariableFilter());
      request.setRegistryFilter(buildRegistryFilter());
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

   private static class SessionDataCollectionBasedFilter extends SessionDataFilterParameters
   {
      private final Set<String> yoVariableNames = new LinkedHashSet<>();

      private final Predicate<YoVariable> variableFilter = v -> yoVariableNames.contains(v.getFullNameString());
      private final Predicate<YoRegistry> registryFilter = null;

      public SessionDataCollectionBasedFilter(String name)
      {
         super(name, null, null);
      }

      private void clear()
      {
         yoVariableNames.clear();
      }

      @Override
      public Predicate<YoVariable> getVariableFilter()
      {
         return variableFilter;
      }

      @Override
      public Predicate<YoRegistry> getRegistryFilter()
      {
         return registryFilter;
      }
   }
}
