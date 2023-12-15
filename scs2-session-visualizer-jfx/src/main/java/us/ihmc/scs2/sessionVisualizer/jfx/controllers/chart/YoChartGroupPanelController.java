package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.application.Platform;
import javafx.beans.Observable;
import javafx.beans.property.*;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.collections.SetChangeListener;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Bounds;
import javafx.geometry.Point2D;
import javafx.scene.Node;
import javafx.scene.SnapshotParameters;
import javafx.scene.control.Button;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Label;
import javafx.scene.effect.GaussianBlur;
import javafx.scene.input.*;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.transform.Scale;
import javafx.stage.Popup;
import javafx.stage.Window;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.yoChart.YoChartGroupConfigurationDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartGroupConfigurationListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupLayout;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.DynamicLineChart;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.TableSizeQuickAccess;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.ChartTable2D.ChartTable2DSize;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ChartGroupTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class YoChartGroupPanelController implements VisualizerController
{
   private SessionVisualizerWindowToolkit toolkit;
   private YoCompositeSearchManager yoCompositeSearchManager;

   private final StringProperty chartGroupName = new SimpleStringProperty(this, "chartGroupName", null);
   private final StringProperty userDefinedChartGroupName = new SimpleStringProperty(this, "userDefinedChartGroupName", null);
   private final StringProperty automatedChartGroupName = new SimpleStringProperty(this, "automatedChartGroupName", null);

   private final GridPane gridPane = new GridPane();
   private final BooleanProperty isRunning = new SimpleBooleanProperty(this, "isRunning", false);
   private final ChartTable2D chartTable2D = new ChartTable2D(() -> createNewChartPanel(isRunning.get()));

   @FXML
   private AnchorPane mainPane;
   @FXML
   private Button dropDownMenuButton;

   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   private final ObservableList<YoVariable> plottedVariableList = FXCollections.observableArrayList();
   private Property<YoNameDisplay> userDesiredDisplayProperty;
   private final BooleanProperty useUniqueNames = new SimpleBooleanProperty(this, "useUniqueNames", false);

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      mainPane.getChildren().add(0, gridPane);
      AnchorPane.setTopAnchor(gridPane, 0.0);
      AnchorPane.setLeftAnchor(gridPane, 0.0);
      AnchorPane.setRightAnchor(gridPane, 0.0);
      AnchorPane.setBottomAnchor(gridPane, 0.0);
      gridPane.getStyleClass().add("chart-group-grid-pane");

      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      messager.submitMessage(topics.getRegisterRecordable(), mainPane);
      messager.addFXTopicListener(topics.getDisableUserControls(), m -> mainPane.setDisable(m));

      toolkit.getWindow().iconifiedProperty().addListener((o, oldValue, newValue) ->
                                                          {
                                                             if (newValue != isRunning.get())
                                                                return;
                                                             if (newValue)
                                                                stop();
                                                             else
                                                                start();
                                                          });

      chartGroupName.bind(automatedChartGroupName);

      userDefinedChartGroupName.addListener(new ChangeListener<>()
      {
         private Observable currentBind = automatedChartGroupName;

         @Override
         public void changed(ObservableValue<? extends String> o, String oldValue, String newValue)
         {
            if (newValue != null)
            {
               if (currentBind != userDefinedChartGroupName)
               {
                  currentBind = userDefinedChartGroupName;
                  chartGroupName.unbind();
                  chartGroupName.bind(userDefinedChartGroupName);
               }
            }
            else
            {
               if (currentBind != automatedChartGroupName)
               {
                  currentBind = automatedChartGroupName;
                  chartGroupName.unbind();
                  chartGroupName.bind(automatedChartGroupName);
               }
            }
         }
      });

      userDesiredDisplayProperty = messager.createPropertyInput(topics.getYoVariableNameDisplay());

      plottedVariableList.addListener((ListChangeListener<? super YoVariable>) change ->
      {
         updateAutoUniqueNameDisplay();
      });
      userDesiredDisplayProperty.addListener((o, oldValue, newValue) ->
                                             {
                                                if (newValue == YoNameDisplay.UNIQUE_NAME)
                                                   useUniqueNames.set(true);
                                                else
                                                   updateAutoUniqueNameDisplay();
                                             });

      if (userDesiredDisplayProperty.getValue() == YoNameDisplay.UNIQUE_NAME)
         useUniqueNames.set(true);

      SetChangeListener<YoVariable> plottedVariableChangeListener = change ->
      {
         if (change.wasAdded())
            plottedVariableList.add(change.getElementAdded());
         if (change.wasRemoved())
            plottedVariableList.remove(change.getElementRemoved());
      };

      chartTable2D.addListener(c ->
                               {
                                  YoChartPanelController chart = c.getChart();

                                  switch (c.type())
                                  {
                                     case ADD:
                                        gridPane.add(chart.getMainPane(), c.toCol(), c.toRow());
                                        chart.getPlottedVariables().addListener(plottedVariableChangeListener);
                                        chart.useUniqueNamesProperty().bind(useUniqueNames);
                                        break;
                                     case REMOVE:
                                        gridPane.getChildren().remove(chart.getMainPane());
                                        chart.getPlottedVariables().removeListener(plottedVariableChangeListener);
                                        chart.useUniqueNamesProperty().unbind();
                                        break;
                                     case MOVE:
                                        GridPane.setConstraints(chart.getMainPane(), c.toCol(), c.toRow());
                                        break;
                                     default:
                                        throw new IllegalStateException("Unexpected change type: " + c.type());
                                  }
                               });
   }

   public boolean isEmpty()
   {
      return chartTable2D.isEmpty();
   }

   private void updateAutoUniqueNameDisplay()
   {
      if (plottedVariableList.isEmpty())
      {
         // No YoVariable left, resetting the user group
         automatedChartGroupName.set(null);
         userDefinedChartGroupName.set(null);
      }
      else
      {
         toolkit.getGlobalToolkit()
                .generateChartGroupTitle(this,
                                         plottedVariableList,
                                         value -> JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> automatedChartGroupName.set(value)));
      }

      if (userDesiredDisplayProperty.getValue() == YoNameDisplay.UNIQUE_NAME)
      {
         useUniqueNames.set(true);
      }
      else
      {
         List<? extends YoVariable> distinctVariables = plottedVariableList.stream().distinct().toList();
         long distinctNameCount = distinctVariables.stream().map(YoVariable::getName).distinct().count();
         useUniqueNames.set(distinctNameCount < distinctVariables.size());
      }
   }

   public void setChartGroupConfiguration(YoChartGroupConfigurationDefinition definition)
   {
      if (!chartTable2D.set(definition))
         return;
      userDefinedChartGroupName.set(definition.getName());
   }

   private void handleCloseChart(ActionEvent event, YoChartPanelController chartToClose)
   {
      chartTable2D.removeChart(getChartIdentifier(chartToClose));
      chartTable2D.removeNullRowsAndColumns();
      event.consume();
   }

   public YoChartPanelController getChartController(ChartIdentifier chartIdentifier)
   {
      return chartTable2D.get(chartIdentifier.getRow(), chartIdentifier.getColumn());
   }

   public ChartIdentifier getChartIdentifier(YoChartPanelController controller)
   {
      return new ChartIdentifier(GridPane.getRowIndex(controller.getMainPane()), GridPane.getColumnIndex(controller.getMainPane()));
   }

   private YoChartPanelController createNewChartPanel(boolean start)
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_PANEL_FXML_URL);
         AnchorPane graphNode = loader.load();
         YoChartPanelController controller = loader.getController();
         controller.initialize(toolkit);
         DynamicLineChart chartNode = controller.getLineChart();
         chartNode.setOnDragOver(e -> handleDragOver(e, controller));
         chartNode.setOnDragDropped(e -> handleDragDropped(e, controller));
         chartNode.setOnDragEntered(e -> handleDragEntered(e, controller));
         chartNode.setOnDragExited(e -> handleDragExited(e, controller));

         controller.getCloseButton().setOnAction(e -> handleCloseChart(e, controller));
         controller.getChartMoveIcon().setOnDragDetected(e -> startChartMove(e, controller));
         controller.getChartMoveIcon().setOnDragDone(e -> finishChartMove(e, controller));

         setGrowProperty(graphNode);

         if (start)
            controller.start();

         return controller;
      }
      catch (IOException e)
      {
         throw new RuntimeException("Could not create new line chart panel:", e);
      }
   }

   private static void setGrowProperty(Node graphNode)
   {
      GridPane.setHgrow(graphNode, Priority.ALWAYS);
      GridPane.setVgrow(graphNode, Priority.ALWAYS);
   }

   public void start()
   {
      isRunning.set(true);
      chartTable2D.forEachChart(YoChartPanelController::start);
   }

   public void stop()
   {
      isRunning.set(false);
      chartTable2D.forEachChart(YoChartPanelController::stop);
   }

   public void stopAndClear()
   {
      stop();
      chartTable2D.clear();
   }

   public void closeAndDispose()
   {
      stopAndClear();
   }

   @FXML
   void openMenu(ActionEvent event)
   {
      Popup popup = new Popup();
      popup.autoHideProperty().set(true);
      TableSizeQuickAccess tableSizeQuickAccess = new TableSizeQuickAccess("Select graph table size:", chartTable2D.getSize(), chartTable2D.getMaxSize());
      AnchorPane rootNode = tableSizeQuickAccess.getMainPane();
      Pane backgroundPane = new Pane();
      backgroundPane.setStyle("-fx-background-radius:10;-fx-background-color:rgba(79,132,186,0.5);"); // TODO Needs to be extracted to CSS
      backgroundPane.setEffect(new GaussianBlur(10));
      rootNode.getChildren().add(0, backgroundPane);
      AnchorPane.setTopAnchor(backgroundPane, -5.0);
      AnchorPane.setBottomAnchor(backgroundPane, -5.0);
      AnchorPane.setLeftAnchor(backgroundPane, -5.0);
      AnchorPane.setRightAnchor(backgroundPane, -5.0);
      rootNode.setOnMouseClicked(e ->
                                 {
                                    chartTable2D.resize(new ChartTable2DSize(tableSizeQuickAccess.selectedRowsProperty().get(),
                                                                             tableSizeQuickAccess.selectedColumnsProperty().get()));
                                    popup.hide();
                                 });
      tableSizeQuickAccess.getClearAllButton().setOnMouseClicked(e ->
                                                                 {
                                                                    chartTable2D.clear();
                                                                    popup.hide();
                                                                 });
      tableSizeQuickAccess.getClearEmptyButton().setOnMouseClicked(e ->
                                                                   {
                                                                      chartTable2D.removeEmptyCharts();
                                                                      chartTable2D.removeNullRowsAndColumns();
                                                                      popup.hide();
                                                                   });
      popup.getContent().add(rootNode);
      dropDownMenuButton.setDisable(true);
      popup.setOnHiding(e -> dropDownMenuButton.setDisable(false));
      Bounds boundsInLocal = dropDownMenuButton.getBoundsInLocal();
      Point2D anchorPosition = dropDownMenuButton.localToScreen(boundsInLocal.getMinX(), boundsInLocal.getMaxY());

      popup.show(dropDownMenuButton, anchorPosition.getX(), anchorPosition.getY());
      event.consume();
   }

   public Property<ChartTable2DSize> maxSizeProperty()
   {
      return chartTable2D.maxSizeProperty();
   }

   public void setUserDefinedChartGroupName(String name)
   {
      userDefinedChartGroupName.set(name);
   }

   public ReadOnlyStringProperty chartGroupNameProperty()
   {
      return chartGroupName;
   }

   private void handleDragEntered(DragEvent event, YoChartPanelController controller)
   {
      if (acceptDragEventForDrop(event, controller))
      {
         if (event.getTransferMode() == TransferMode.MOVE)
         {
            controller.setSelectionHighlight(true);
         }
         else
         {
            List<YoComposite> yoComposites = DragAndDropTools.retrieveYoCompositesFromDragBoard(event.getDragboard(), yoCompositeSearchManager);
            List<ChartGroupLayout> configurations = ChartGroupTools.toChartGroupLayouts(yoComposites);
            configurations = shiftConfigurationsToSelectedChart(controller, configurations);
            List<YoChartPanelController> controllers = controllersInConfigurations(configurations);
            controllers.forEach(c -> c.setSelectionHighlight(true));
         }
      }
      event.consume();
   }

   private void handleDragExited(DragEvent event, YoChartPanelController controller)
   {
      if (acceptDragEventForDrop(event, controller))
      {
         chartTable2D.forEachChart(c -> c.setSelectionHighlight(false));
      }
      event.consume();
   }

   private void handleDragOver(DragEvent event, YoChartPanelController controller)
   {
      if (acceptDragEventForDrop(event, controller))
      {
         if (controller.isEmpty())
            event.acceptTransferModes(TransferMode.ANY);
         else
            event.acceptTransferModes(TransferMode.COPY);
      }
      event.consume();
   }

   private void handleDragDropped(DragEvent event, YoChartPanelController controller)
   {
      boolean success = false;
      List<YoComposite> yoComposites = DragAndDropTools.retrieveYoCompositesFromDragBoard(event.getDragboard(), yoCompositeSearchManager);

      if (yoComposites != null)
      {
         if (event.getTransferMode() == TransferMode.MOVE)
         { // We're moving a chart, dropping all variables in a single chart
            yoComposites.forEach(yoComposite -> yoComposite.getYoComponents().forEach(controller::addYoVariableToPlot));
         }
         else
         {
            List<ChartGroupLayout> layouts = ChartGroupTools.toChartGroupLayouts(yoComposites);
            layouts = shiftConfigurationsToSelectedChart(controller, layouts);

            if (layouts.size() > 1)
            {
               ContextMenu contextMenu = new ContextMenu();

               for (ChartGroupLayout layout : layouts)
               {
                  Label label = new Label(layout.getName());
                  CustomMenuItem menuItem = new CustomMenuItem(label);

                  label.setOnMouseEntered(e ->
                                          {
                                             chartTable2D.forEachChart(c -> c.setSelectionHighlight(false));
                                             controllersInConfiguration(layout).forEach(c -> c.setSelectionHighlight(true));
                                          });

                  menuItem.setOnAction(e -> applyLayout(layout));
                  contextMenu.getItems().add(menuItem);
               }

               contextMenu.show(mainPane, event.getScreenX(), event.getScreenY());
            }
            else if (layouts.size() == 1)
            {
               applyLayout(layouts.get(0));
            }
            else
            {
               yoComposites.forEach(yoComposite -> yoComposite.getYoComponents().forEach(controller::addYoVariableToPlot));
            }
         }

         success = true;
      }
      event.setDropCompleted(success);
      event.consume();
   }

   private void startChartMove(MouseEvent event, YoChartPanelController controller)
   {
      if (!event.isPrimaryButtonDown())
         return;

      List<YoVariable> yoVariables = new ArrayList<>(controller.getPlottedVariables());

      FontAwesomeIconView chartMoveIcon = controller.getChartMoveIcon();
      Dragboard dragBoard = chartMoveIcon.startDragAndDrop(TransferMode.MOVE);
      SnapshotParameters params = new SnapshotParameters();
      params.setTransform(new Scale(0.5, 0.5));
      dragBoard.setDragView(controller.getMainPane().snapshot(params, null));
      ClipboardContent clipboardContent = new ClipboardContent();

      if (yoVariables.size() == 1)
      {
         YoVariable yoVariable = yoVariables.get(0);
         if (yoVariable == null)
            return;
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_REFERENCE, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariable.getFullNameString()));
      }
      else
      {
         List<String> content = new ArrayList<>();
         for (YoVariable yoVariable : yoVariables)
         {
            content.add(YoCompositeTools.YO_VARIABLE);
            content.add(yoVariable.getFullNameString());
         }
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_LIST_REFERENCE, content);
      }
      dragBoard.setContent(clipboardContent);
      event.consume();
   }

   private void finishChartMove(DragEvent event, YoChartPanelController controller)
   {
      if (!event.isAccepted())
         return;
      if (event.getTransferMode() != TransferMode.MOVE)
         return;

      controller.clear();
   }

   private void applyLayout(ChartGroupLayout layout)
   {
      chartTable2D.forEachChart(c -> c.setSelectionHighlight(false));

      for (ChartIdentifier chartIdentifier : layout.getChartIdentifiers())
      {
         List<? extends YoVariable> yoVariables = layout.getYoVariables(chartIdentifier);
         YoChartPanelController chartController = getChartController(chartIdentifier);
         chartController.addYoVariablesToPlot(yoVariables);
      }
   }

   private List<YoChartPanelController> controllersInConfiguration(ChartGroupModel configuration)
   {
      if (!doesConfigurationFit(configuration))
         return null;
      else
         return configuration.getChartIdentifiers().stream().map(this::getChartController).collect(Collectors.toList());
   }

   public List<YoChartPanelController> controllersInConfigurations(List<? extends ChartGroupModel> configurations)
   {
      return configurations.stream()
                           .flatMap(config -> config.getChartIdentifiers().stream())
                           .distinct()
                           .map(this::getChartController)
                           .collect(Collectors.toList());
   }

   private List<ChartGroupLayout> shiftConfigurationsToSelectedChart(YoChartPanelController selectedChart, List<ChartGroupLayout> layouts)
   {
      ChartIdentifier selectedId = getChartIdentifier(selectedChart);
      return layouts.stream()
                    .map(config -> config.shift(selectedId.getRow(), selectedId.getColumn()))
                    .filter(this::doesConfigurationFit)
                    .collect(Collectors.toList());
   }

   private boolean acceptDragEventForDrop(DragEvent event, YoChartPanelController controller)
   {
      if (event.getGestureSource() == mainPane || event.getGestureSource() == controller.getChartMoveIcon())
         return false;

      if (event.getTransferMode() == TransferMode.MOVE && !controller.isEmpty())
         return false;

      return DragAndDropTools.retrieveYoCompositesFromDragBoard(event.getDragboard(), yoCompositeSearchManager) != null;
   }

   private boolean doesConfigurationFit(ChartGroupModel configuration)
   {
      return chartTable2D.getSize().contains(configuration.rowEnd(), configuration.columnEnd());
   }

   public void loadChartGroupConfiguration(Window source, File file)
   {
      if (source != toolkit.getWindow())
         return;

      LogTools.info("Loading file: " + file);

      try
      {
         Object loaded = XMLTools.loadYoChartGroupConfigurationUndefined(new FileInputStream(file));
         if (loaded instanceof YoChartGroupConfigurationDefinition definition)
            setChartGroupConfiguration(definition);
         else if (loaded instanceof YoChartGroupConfigurationListDefinition)
            LogTools.error("Chart group list is not supported. Probably loaded for the main window. Please load the file for a secondary window instead.");
         else
            LogTools.error(
                  "Failed to load chart group configuration from file: " + file + ". definition type is not supported: " + loaded.getClass().getSimpleName());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void saveChartGroupConfiguration(Window source, File file)
   {
      if (source != null && source != toolkit.getWindow())
         return;

      if (!Platform.isFxApplicationThread())
         throw new IllegalStateException("Save must only be used from the FX Application Thread");

      LogTools.info("Saving file: " + file);

      try
      {
         XMLTools.saveYoChartGroupConfigurationDefinition(new FileOutputStream(file), toYoChartGroupConfigurationDefinition());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public YoChartGroupConfigurationDefinition toYoChartGroupConfigurationDefinition()
   {
      YoChartGroupConfigurationDefinition definition = new YoChartGroupConfigurationDefinition();
      definition.setName(chartGroupName.get());
      definition.setNumberOfRows(chartTable2D.getSize().getNumberOfRows());
      definition.setNumberOfColumns(chartTable2D.getSize().getNumberOfCols());
      definition.setChartConfigurations(chartTable2D.toChartDefinitions());
      return definition;
   }

   public AnchorPane getMainPane()
   {
      return mainPane;
   }
}
