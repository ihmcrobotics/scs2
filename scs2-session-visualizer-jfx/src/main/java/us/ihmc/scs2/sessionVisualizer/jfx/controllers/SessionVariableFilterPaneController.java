package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBoxTreeItem;
import javafx.scene.control.ComboBox;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TreeItem;
import javafx.scene.control.cell.CheckBoxTreeCell;
import javafx.scene.layout.Pane;
import org.controlsfx.control.CheckTreeView;
import org.kordamp.ikonli.javafx.FontIcon;
import us.ihmc.scs2.definition.yoVariable.YoVariableGroupDefinition;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TreeViewTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import jakarta.xml.bind.JAXBException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;

public class SessionVariableFilterPaneController
{
   private static final String MODIFIED_FILTER_NAME = "- Modified -";
   private static final String NONE_FILTER_NAME = "None";
   private static final String ALL_FILTER_NAME = "All";
   private static final String IMPORT_FILTER_NAME = "Import ...";

   @FXML
   private Pane mainPane;

   @FXML
   private CheckTreeView<Object> selectedVariablesCheckTreeView;
   @FXML
   private Button selectAllButton, unselectAllButton;
   @FXML
   private ComboBox<String> filterComboBox;

   private SessionDataFilterParameters currentFilter;
   private Map<String, SessionDataFilterParameters> filterMap;
   private final SessionDataCollectionBasedFilter modifiedFilter = new SessionDataCollectionBasedFilter(MODIFIED_FILTER_NAME);

   private final ObjectProperty<ContextMenu> activeContexMenu = new SimpleObjectProperty<>(this, "activeContextMenu", null);

   private CheckBoxTreeItem<Object> rootItem;

   private final List<Runnable> cleanupActions = new ArrayList<>();

   private YoManager yoManager;
   private Predicate<YoVariable> treeViewVariableFilter = null;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      initialize(toolkit, null);
   }

   public void initialize(SessionVisualizerToolkit toolkit, Predicate<YoVariable> treeViewVariableFilter)
   {
      yoManager = toolkit.getYoManager();
      this.treeViewVariableFilter = treeViewVariableFilter;

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

                                                                  FontIcon collapseIcon = new FontIcon("fa-minus-square-o");
                                                                  FontIcon expandIcon = new FontIcon("fa-plus-square-o");
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

      filterMap = toolkit.getSessionDataPreferenceManager().getFilterMap();

      filterComboBox.setItems(FXCollections.observableArrayList());
      filterComboBox.getItems().addAll(MODIFIED_FILTER_NAME);
      filterComboBox.getItems().addAll(ALL_FILTER_NAME);
      filterComboBox.getItems().addAll(NONE_FILTER_NAME);
      filterComboBox.getItems().addAll(filterMap.keySet());
      filterComboBox.getItems().add(IMPORT_FILTER_NAME);

      filterComboBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
                                                                            {
                                                                               if (IMPORT_FILTER_NAME.equals(newValue))
                                                                               {
                                                                                  if (!importFilter())
                                                                                     filterComboBox.getSelectionModel().select(oldValue);
                                                                                  return;
                                                                               }

                                                                               if (MODIFIED_FILTER_NAME.equals(oldValue))
                                                                                  updateModifiedFilterFromTreeView();

                                                                               if (MODIFIED_FILTER_NAME.equals(newValue))
                                                                                  currentFilter = modifiedFilter;
                                                                               else if (ALL_FILTER_NAME.equals(newValue))
                                                                                  currentFilter = allFilter;
                                                                               else if (NONE_FILTER_NAME.equals(newValue))
                                                                                  currentFilter = noneFilter;
                                                                               else
                                                                                  currentFilter = filterMap.get(newValue);

                                                                               applyFilterToTreeItems(currentFilter);
                                                                            });
      filterComboBox.getSelectionModel().select(ALL_FILTER_NAME);

      selectAllButton.setOnAction(e -> filterComboBox.getSelectionModel().select(ALL_FILTER_NAME));
      unselectAllButton.setOnAction(e -> filterComboBox.getSelectionModel().select(NONE_FILTER_NAME));
   }

   public void dispose()
   {
      cleanupActions.forEach(Runnable::run);
      cleanupActions.clear();
   }

   public Pane getMainPane()
   {
      return mainPane;
   }

   private boolean importFilter()
   {
      if (mainPane.getScene() == null || mainPane.getScene().getWindow() == null)
         return false;

      File result = SessionVisualizerIOTools.yoVariableGroupConfigurationOpenFileDialog(mainPane.getScene().getWindow());
      if (result == null)
         return false;

      try
      {
         SessionDataCollectionBasedFilter filter = new SessionDataCollectionBasedFilter(XMLTools.loadYoVariableGroupDefinition(new FileInputStream(result)));
         if (!filterMap.containsKey(filter.getName()))
            filterComboBox.getItems().add(filterComboBox.getItems().size() - 1, filter.getName());
         filterMap.put(filter.getName(), filter);
         filterComboBox.getSelectionModel().select(filter.getName());
         return true;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return false;
      }
   }

   @FXML
   private void saveActiveFilter()
   {
      if (mainPane.getScene() == null || mainPane.getScene().getWindow() == null)
         return;

      File result = SessionVisualizerIOTools.yoVariableGroupConfigurationSaveFileDialog(mainPane.getScene().getWindow());

      if (result == null)
         return;

      try
      {
         updateModifiedFilterFromTreeView();
         YoVariableGroupDefinition definition = modifiedFilter.toYoVariableGroupDefinition();
         definition.setName(result.getName().replace(SessionVisualizerIOTools.yoVariableGroupConfigurationFileExtension, ""));
         XMLTools.saveYoVariableGroupDefinition(new FileOutputStream(result), definition);
         if (!filterMap.containsKey(definition.getName()))
            filterComboBox.getItems().add(filterComboBox.getItems().size() - 1, definition.getName());
         filterMap.put(definition.getName(), new SessionDataCollectionBasedFilter(definition));
         filterComboBox.getSelectionModel().select(definition.getName());
      }
      catch (JAXBException | IOException e)
      {
         e.printStackTrace();
      }
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
            buildTreeRecursively(childItem);

            if (!childItem.getChildren().isEmpty())
               parent.getChildren().add(childItem);
         }

         for (YoVariable variable : registry.getVariables())
         {
            if (treeViewVariableFilter != null && !treeViewVariableFilter.test(variable))
               continue;

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

   public Predicate<YoVariable> buildVariableFilter()
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

   public Predicate<YoRegistry> buildRegistryFilter()
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
      private final Set<String> yoVariableNames;
      private final Set<String> yoRegistryNames;

      private final Predicate<YoVariable> variableFilter;
      private final Predicate<YoRegistry> registryFilter;

      public SessionDataCollectionBasedFilter(String name)
      {
         super(name, null, null);

         yoVariableNames = new LinkedHashSet<>();
         yoRegistryNames = null;

         variableFilter = v -> yoVariableNames.contains(v.getFullNameString());
         registryFilter = null;
      }

      public SessionDataCollectionBasedFilter(YoVariableGroupDefinition definition)
      {
         super(definition.getName(), null, null);

         Objects.requireNonNull(definition.getName());

         if (definition.getVariableNames() != null)
         {
            yoVariableNames = new LinkedHashSet<>(definition.getVariableNames());
            variableFilter = v -> yoVariableNames.contains(v.getFullNameString()) || yoVariableNames.contains(v.getName());
         }
         else
         {
            yoVariableNames = null;
            variableFilter = null;
         }

         if (definition.getRegistryNames() != null)
         {
            yoRegistryNames = new LinkedHashSet<>(definition.getRegistryNames());
            registryFilter = reg -> yoRegistryNames.contains(reg.getName()) || yoRegistryNames.contains(reg.getNamespace().getName());
         }
         else
         {
            yoRegistryNames = null;
            registryFilter = null;
         }
      }

      private void clear()
      {
         if (yoVariableNames != null)
            yoVariableNames.clear();
         if (yoRegistryNames != null)
            yoRegistryNames.clear();
      }

      public YoVariableGroupDefinition toYoVariableGroupDefinition()
      {
         YoVariableGroupDefinition definition = new YoVariableGroupDefinition();
         definition.setName(getName());
         definition.setVariableNames(yoVariableNames != null ? yoVariableNames.stream().toList() : null);
         definition.setRegistryNames(yoRegistryNames != null ? yoRegistryNames.stream().toList() : null);
         return definition;
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
