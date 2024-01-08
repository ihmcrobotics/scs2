package us.ihmc.scs2.sessionVisualizer.jfx;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.application.Platform;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableMap;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.Pair;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.log.LogTools;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.definition.yoChart.YoChartGroupConfigurationDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartGroupConfigurationListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SecondaryWindowControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.ChartTable2D.ChartTable2DSize;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartGroupPanelController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu.MainWindowMenuBarController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TabPaneTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

public class SecondaryWindowController implements VisualizerController
{
   @FXML
   private VBox mainNode;
   @FXML
   private TabPane tabPane;
   @FXML
   private MainWindowMenuBarController menuController;
   @FXML
   private SecondaryWindowControlsController controlsController;

   private final TopicListener<Pair<Window, File>> loadChartGroupConfigurationListener = m -> loadChartGroupConfiguration(m.getKey(), m.getValue());
   private final TopicListener<Pair<Window, File>> saveChartGroupConfigurationListener = m -> saveChartGroupConfiguration(m.getKey(), m.getValue(), false);

   private final ObservableMap<Tab, YoChartGroupPanelController> chartGroupControllers = FXCollections.observableHashMap();

   private SessionVisualizerWindowToolkit toolkit;
   private boolean isMessagerSetup = false;
   private StringProperty userDefinedChartWindowName = new SimpleStringProperty(this, "userDefinedChartWindowName", null);
   private Stage stage;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      menuController.initialize(toolkit);
      controlsController.initialize(toolkit);

      stage = toolkit.getWindow();
      Scene scene = new Scene(mainNode, 1024, 768);
      stage.setScene(scene);
      SessionVisualizerIOTools.addSCSIconToWindow(stage);
      stage.setTitle("Chart Window");
      stage.addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (e.isConsumed())
            return;
         closeAndDispose();
         stage.close();
      });

      ChangeListener<String> chartTitleListener = (o, oldValue, newValue) -> updateWindowTitle();

      tabPane.tabClosingPolicyProperty().setValue(TabPane.TabClosingPolicy.ALL_TABS);
      tabPane.tabDragPolicyProperty().setValue(TabPane.TabDragPolicy.REORDER);
      tabPane.getTabs().addListener((ListChangeListener<Tab>) c ->
      {
         while (c.next())
         {
            if (c.wasAdded())
            {
               for (Tab tab : c.getAddedSubList())
               {
                  YoChartGroupPanelController controller = chartGroupControllers.get(tab);
                  if (controller == null)
                     continue;
                  controller.chartGroupNameProperty().addListener(chartTitleListener);
               }
            }
            if (c.wasRemoved())
            {
               for (Tab tab : c.getRemoved())
               {
                  YoChartGroupPanelController controller = chartGroupControllers.remove(tab);
                  if (controller == null)
                     continue;
                  controller.closeAndDispose();
                  controller.chartGroupNameProperty().removeListener(chartTitleListener);
               }
            }
         }

         if (tabPane.getTabs().isEmpty())
            tabPane.getTabs().add(newChartGroupTab());

         updateWindowTitle();
      });

      userDefinedChartWindowName.addListener((o, oldValue, newValue) ->
                                             {
                                                if (newValue == null)
                                                   updateWindowTitle();
                                                else
                                                   stage.setTitle("Chart Window" + ": " + newValue);
                                             });

      tabPane.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
                                                                     {
                                                                        if (newValue == null)
                                                                           return;
                                                                        YoChartGroupPanelController controller = chartGroupControllers.get(newValue);
                                                                        if (controller != null)
                                                                           controller.start();
                                                                        if (oldValue == null)
                                                                           return;
                                                                        controller = chartGroupControllers.get(oldValue);
                                                                        if (controller != null)
                                                                           controller.stop();
                                                                     });

      MenuTools.setupContextMenu(tabPane,
                                 TabPaneTools.addBeforeMenuItemFactory(this::newChartGroupTab),
                                 TabPaneTools.addAfterMenuItemFactory(this::newChartGroupTab),
                                 TabPaneTools.removeMenuItemFactory(),
                                 TabPaneTools.removeAllMenuItemFactory(),
                                 exportTabMenuItemFactory(),
                                 exportAllTabsMenuItemFactory(),
                                 importTabMenuItemFactory());
      if (tabPane.getTabs().isEmpty())
         tabPane.getTabs().add(newChartGroupTab());

      SCS2JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();
      messager.addFXTopicListener(topics.getYoChartGroupLoadConfiguration(), loadChartGroupConfigurationListener);
      messager.addFXTopicListener(topics.getYoChartGroupSaveConfiguration(), saveChartGroupConfigurationListener);
      isMessagerSetup = true;
   }

   private void updateWindowTitle()
   {
      if (userDefinedChartWindowName.get() != null)
         return;

      String title = "Chart Window";

      List<String> nonEmptyTabNames = tabPane.getTabs()
                                             .stream()
                                             .filter(tab -> !chartGroupControllers.get(tab).isEmpty())
                                             .map(chartGroupControllers::get)
                                             .map(controller -> controller.chartGroupNameProperty().get())
                                             .toList();
      if (!nonEmptyTabNames.isEmpty())
         title += " - [%s]".formatted(String.join(", ", nonEmptyTabNames));
      stage.setTitle(title);
   }

   private Tab newChartGroupTab()
   {
      YoChartGroupPanelController controller = newChartGroup();
      if (controller == null)
         return null;

      Tab tab = new Tab();
      String defaultText = "Chart Group " + tabPane.getTabs().size();
      tab.setText(defaultText);
      tab.setClosable(true);
      tab.setOnCloseRequest(e ->
                            {
                               LogTools.info("Closing chart group: " + controller);
                               controller.closeAndDispose();
                            });
      tab.setContent(controller.getMainPane());

      Label tabHeader = TabPaneTools.editableTabHeader(tab);
      MutableBoolean isAutoUpdating = new MutableBoolean(false);
      controller.chartGroupNameProperty().addListener((o, oldValue, newValue) ->
                                                      {
                                                         if (isAutoUpdating.isTrue())
                                                            return;
                                                         isAutoUpdating.setTrue();
                                                         if (newValue == null || newValue.isEmpty())
                                                            tabHeader.setText(defaultText);
                                                         else
                                                            tabHeader.setText(newValue);
                                                         isAutoUpdating.setFalse();
                                                      });
      tabHeader.textProperty().addListener((o, oldValue, newValue) ->
                                           {
                                              if (isAutoUpdating.isTrue())
                                                 return;
                                              isAutoUpdating.setTrue();
                                              if (newValue.isEmpty())
                                              {
                                                 controller.setUserDefinedChartGroupName(null);
                                                 if (controller.chartGroupNameProperty().get() == null)
                                                    tabHeader.setText(defaultText);
                                                 else
                                                    tabHeader.setText(controller.chartGroupNameProperty().get());
                                              }
                                              else
                                              {
                                                 controller.setUserDefinedChartGroupName(newValue);
                                              }
                                              isAutoUpdating.setFalse();
                                           });

      tabPane.getSelectionModel().select(tab);
      chartGroupControllers.put(tab, controller);
      return tab;
   }

   private YoChartGroupPanelController newChartGroup()
   {
      try
      {
         // Loading the chart pane & controller
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_GROUP_PANEL_URL);
         loader.load();
         YoChartGroupPanelController controller = loader.getController();

         controller.initialize(toolkit);
         controller.start();
         controller.maxSizeProperty().setValue(new ChartTable2DSize(9, 6));
         return controller;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private Function<TabPane, MenuItem> exportTabMenuItemFactory()
   {
      return tabPane ->
      {
         Tab selectedTab = tabPane.getSelectionModel().getSelectedItem();
         if (selectedTab == null)
            return null;
         YoChartGroupPanelController controller = chartGroupControllers.get(selectedTab);
         if (controller == null)
            return null;

         FontAwesomeIconView exportIcon = new FontAwesomeIconView();
         exportIcon.getStyleClass().add(("save-icon-view"));
         MenuItem menuItem = new MenuItem("Export active tab...", exportIcon);
         menuItem.setOnAction(e ->
                              {
                                 File result = SessionVisualizerIOTools.yoChartConfigurationSaveFileDialog(stage);
                                 if (result == null)
                                    return;
                                 controller.saveChartGroupConfiguration(stage, result);
                              });
         return menuItem;
      };
   }

   private Function<TabPane, MenuItem> exportAllTabsMenuItemFactory()
   {
      return tabPane ->
      {
         FontAwesomeIconView exportIcon = new FontAwesomeIconView();
         exportIcon.getStyleClass().add(("save-icon-view"));
         MenuItem menuItem = new MenuItem("Export all tabs...", exportIcon);
         menuItem.setOnAction(e ->
                              {
                                 File result = SessionVisualizerIOTools.yoChartConfigurationSaveFileDialog(stage);
                                 if (result == null)
                                    return;
                                 saveChartGroupConfiguration(stage, result, false);
                              });
         return menuItem;
      };
   }

   private Function<TabPane, MenuItem> importTabMenuItemFactory()
   {
      return tabPane ->
      {
         FontAwesomeIconView importIcon = new FontAwesomeIconView();
         importIcon.getStyleClass().add(("load-icon-view"));
         MenuItem menuItem = new MenuItem("Import tab(s)...", importIcon);
         menuItem.setOnAction(e ->
                              {
                                 File result = SessionVisualizerIOTools.yoChartConfigurationOpenFileDialog(stage);
                                 if (result == null)
                                    return;
                                 loadChartGroupConfiguration(result, tabPane.getSelectionModel().getSelectedItem());
                              });
         return menuItem;
      };
   }

   private void loadChartGroupConfiguration(Window source, File file)
   {
      if (source != stage)
         return;

      loadChartGroupConfiguration(file, tabPane.getSelectionModel().getSelectedItem());
   }

   private void loadChartGroupConfiguration(File file, Tab insertionPoint)
   {
      LogTools.info("Loading file: " + file);

      try
      {
         Object loaded = XMLTools.loadYoChartGroupConfigurationUndefined(new FileInputStream(file));
         int insertionIndex = insertionPoint == null ? -1 : tabPane.getTabs().indexOf(insertionPoint);
         if (loaded instanceof YoChartGroupConfigurationDefinition chartGroupDefinition)
         {
            loadDefinition(chartGroupDefinition, insertionIndex);
         }
         else if (loaded instanceof YoChartGroupConfigurationListDefinition chartGroupListDefinition)
         {
            for (YoChartGroupConfigurationDefinition chartGroupDefinition : chartGroupListDefinition.getChartGroupConfigurations())
            {
               if (!loadDefinition(chartGroupDefinition, insertionIndex))
                  return;
               insertionIndex++;
            }

            if (chartGroupListDefinition.getName() != null)
               userDefinedChartWindowName.set(chartGroupListDefinition.getName());
         }
         else
         {
            LogTools.error("Failed to load file: " + file);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   /**
    * Selects the active chart group by index.
    * <p>
    * The active chart group is the one that is currently displayed and that can be edited.
    * </p>
    *
    * @param index the index of the chart group to select.
    * @return {@code true} if the index is valid and the chart group was selected, {@code false} otherwise.
    */
   public boolean selectActiveChartGroup(int index)
   {
      if (index < 0 || index >= tabPane.getTabs().size())
         return false;
      tabPane.getSelectionModel().select(index);
      return true;
   }

   /**
    * Selects the active chart group by name.
    * <p>
    * The active chart group is the one that is currently displayed and that can be edited.
    * If the chart group name is not found, nothing happens.
    * </p>
    *
    * @param chartGroupName the name of the chart group to select.
    * @return {@code true} if the chart group was found and selected, {@code false} otherwise.
    */
   public boolean selectActiveChartGroup(String chartGroupName)
   {
      for (Tab tab : tabPane.getTabs())
      {
         YoChartGroupPanelController controller = chartGroupControllers.get(tab);
         if (controller == null)
            continue;
         if (controller.chartGroupNameProperty().get().equals(chartGroupName))
         {
            tabPane.getSelectionModel().select(tab);
            return true;
         }
      }
      return false;
   }

   /**
    * Creates a new chart group with the given name.
    *
    * @param chartGroupName the name of the new chart group.
    * @return {@code true} if the chart group was created, {@code false} otherwise.
    */
   public boolean createNewChartGroup(String chartGroupName)
   {
      Tab newTab = newChartGroupTab();
      if (newTab == null)
         return false;
      YoChartGroupPanelController controller = chartGroupControllers.get(newTab);
      controller.setUserDefinedChartGroupName(chartGroupName);
      tabPane.getTabs().add(newTab);
      return true;
   }

   /**
    * Adds a variable to plot in the active chart group.
    * <p>
    * The active chart group is the one that is currently displayed and that can be edited.
    * </p>
    * <p>
    * The chart group is resized, if needed, to fit the new variable.
    * </p>
    *
    * @param variableName the name of the variable to plot.
    * @param row          the row of the chart in the chart group.
    * @param column       the column of the chart in the chart group.
    * @return {@code true} if the variable has been plotted, {@code false} otherwise.
    */
   public boolean addVariableToActiveChartGroup(String variableName, int row, int column)
   {
      Tab selectedTab = tabPane.getSelectionModel().getSelectedItem();
      if (selectedTab == null)
         return false;
      YoChartGroupPanelController controller = chartGroupControllers.get(selectedTab);
      if (controller == null)
         return false;
      return controller.addVariableToPlot(variableName, row, column, true);
   }

   /**
    * Adds a variable to plot in the chart group with the given name.
    * <p>
    * The chart group is resized, if needed, to fit the new variable.
    * </p>
    *
    * @param chartGroupName the name of the chart group to add the variable to.
    * @param variableName   the name of the variable to plot.
    * @param row            the row of the chart in the chart group.
    * @param column         the column of the chart in the chart group.
    * @param createIfAbsent whether to create a new chart group if the given name does not match any existing chart group.
    * @return {@code true} if the variable has been plotted, {@code false} otherwise.
    */
   public boolean addVariableToChartGroup(String chartGroupName, String variableName, int row, int column, boolean createIfAbsent)
   {
      for (Tab tab : tabPane.getTabs())
      {
         YoChartGroupPanelController controller = chartGroupControllers.get(tab);
         if (controller == null)
            continue;
         if (Objects.equals(controller.chartGroupNameProperty().get(), chartGroupName))
            return controller.addVariableToPlot(variableName, row, column, true);
      }

      if (!createIfAbsent)
         return false;

      YoChartGroupPanelController emptyController = null;

      for (Tab tab : tabPane.getTabs())
      {
         YoChartGroupPanelController controller = chartGroupControllers.get(tab);
         if (controller == null)
            continue;
         if (controller.isEmpty())
         {
            emptyController = controller;
            break;
         }
      }

      if (emptyController != null)
      {
         emptyController.setUserDefinedChartGroupName(chartGroupName);
         emptyController.addVariableToPlot(variableName, row, column, true);
         return true;
      }

      Tab newTab = newChartGroupTab();
      if (newTab == null)
         return false;
      YoChartGroupPanelController controller = chartGroupControllers.get(newTab);
      controller.setUserDefinedChartGroupName(chartGroupName);
      controller.addVariableToPlot(variableName, row, column, true);
      tabPane.getTabs().add(newTab);
      return true;
   }

   public boolean loadDefinition(YoChartGroupConfigurationDefinition chartGroupDefinition, int insertionIndex)
   {
      if (insertionIndex == -1)
      {
         Tab newTab = newChartGroupTab();
         if (newTab == null)
            return false;
         YoChartGroupPanelController controller = chartGroupControllers.get(newTab);
         controller.setChartGroupConfiguration(chartGroupDefinition);
         tabPane.getTabs().add(newTab);
         return true;
      }

      if (insertionIndex < tabPane.getTabs().size())
      {
         Tab insertionTab = tabPane.getTabs().get(insertionIndex);
         YoChartGroupPanelController controller = chartGroupControllers.get(insertionTab);
         if (controller.isEmpty())
         {
            controller.setChartGroupConfiguration(chartGroupDefinition);
            return true;
         }
      }

      Tab newTab = newChartGroupTab();
      if (newTab == null)
         return false;
      YoChartGroupPanelController newController = chartGroupControllers.get(newTab);
      newController.setChartGroupConfiguration(chartGroupDefinition);
      tabPane.getTabs().add(insertionIndex, newTab);
      return true;
   }

   private void saveChartGroupConfiguration(Window source, File file, boolean isConfigurationFile)
   {
      if (source != null && source != stage)
         return;

      if (!Platform.isFxApplicationThread())
         throw new IllegalStateException("Save must only be used from the FX Application Thread");

      LogTools.info("Saving file: " + file);

      try
      {
         YoChartGroupConfigurationListDefinition definition = toYoChartGroupConfigurationListDefinition();
         if (definition.getName() == null && !isConfigurationFile)
            definition.setName(file.getName().replace(SessionVisualizerIOTools.yoChartGroupConfigurationFileExtension, ""));
         XMLTools.saveYoChartGroupConfigurationListDefinition(new FileOutputStream(file), definition);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private YoChartGroupConfigurationListDefinition toYoChartGroupConfigurationListDefinition()
   {
      YoChartGroupConfigurationListDefinition configurationList = new YoChartGroupConfigurationListDefinition();
      for (Tab tab : tabPane.getTabs())
      {
         YoChartGroupPanelController controller = chartGroupControllers.get(tab);
         if (controller == null)
            continue;
         configurationList.addChartGroupConfiguration(controller.toYoChartGroupConfigurationDefinition());
      }
      return configurationList;
   }

   public void saveSessionConfiguration(SCSGuiConfiguration configuration)
   {
      configuration.addSecondaryWindowConfiguration(SecondaryWindowManager.toWindowConfigurationDefinition(stage));
      saveChartGroupConfiguration(stage, configuration.addSecondaryYoChartGroupConfigurationFile(), true);
   }

   public void start()
   {
      toolkit.start();
   }

   public void closeAndDispose()
   {
      tabPane.getTabs().clear();
      stage.close();

      SCS2JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();
      toolkit.getBackgroundExecutorManager().executeInBackground(() ->
                                                                 {
                                                                    if (!isMessagerSetup)
                                                                       return;
                                                                    isMessagerSetup = false;
                                                                    messager.removeFXTopicListener(topics.getYoChartGroupLoadConfiguration(),
                                                                                                   loadChartGroupConfigurationListener);
                                                                    messager.removeFXTopicListener(topics.getYoChartGroupSaveConfiguration(),
                                                                                                   saveChartGroupConfigurationListener);
                                                                 });

      toolkit.stop();
   }
}
