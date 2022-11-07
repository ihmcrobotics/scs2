package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import com.jfoenix.controls.JFXButton;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.MapChangeListener;
import javafx.collections.ObservableList;
import javafx.collections.ObservableMap;
import javafx.collections.ObservableSet;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.chart.InvisibleNumberAxis;
import javafx.scene.control.Button;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.Label;
import javafx.scene.control.MenuItem;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.input.ScrollEvent;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Border;
import javafx.scene.layout.BorderStroke;
import javafx.scene.layout.BorderStrokeStyle;
import javafx.scene.layout.BorderWidths;
import javafx.scene.layout.CornerRadii;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import javafx.util.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.definition.yoChart.ChartDoubleBoundsDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartConfigurationDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIntegerBounds;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartMarker;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.DynamicChartLegend;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.DynamicLineChart;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.DynamicLineChart.ChartStyle;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData.ChartDataUpdate;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ChartDataManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ChartTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoChartPanelController extends ObservedAnimationTimer implements VisualizerController
{
   private static final long LEGEND_UPDATE_PERIOD = TimeUnit.MILLISECONDS.toNanos(100);

   private static final String INPOINT_MARKER_STYLECLASS = "chart-inpoint-marker";
   private static final String OUTPOINT_MARKER_STYLECLASS = "chart-outpoint-marker";
   private static final String CURRENT_INDEX_MARKER_STYLECLASS = "chart-current-index-marker";
   private static final String ORIGIN_MARKER_STYLECLASS = "chart-origin-marker";
   private static final String KEYFRAME_MARKER_STYLECLASS = "chart-keyframe-marker";
   private static final String USER_MARKER_STYLECLASS = "chart-user-marker";

   @FXML
   private AnchorPane chartMainPane;
   @FXML
   private JFXButton closeButton;
   @FXML
   private FontAwesomeIconView chartMoveIcon;

   private final InvisibleNumberAxis xAxis = new InvisibleNumberAxis(0.0, 0.0, 1000.0);
   private final InvisibleNumberAxis yAxis = new InvisibleNumberAxis();
   private DynamicLineChart dynamicLineChart;

   private final ChartMarker inPointMarker = new ChartMarker(new SimpleDoubleProperty(this, "inPointMarkerCoordinate", 0.0));
   private final ChartMarker outPointMarker = new ChartMarker(new SimpleDoubleProperty(this, "outPointMarkerCoordinate", 0.0));
   private final ChartMarker bufferIndexMarker = new ChartMarker(new SimpleDoubleProperty(this, "bufferIndexMarkerCoordinate", 0.0));
   private final ObservableList<ChartMarker> userMarkers = FXCollections.observableArrayList();
   private final ObservableList<ChartMarker> keyFrameMarkers = FXCollections.observableArrayList();

   private YoCompositeSearchManager yoCompositeSearchManager;

   private Property<Integer> legendPrecision;

   private ChartDataManager chartDataManager;
   private final ObservableList<YoNumberSeries> yoNumberSeriesList = FXCollections.observableArrayList();
   private final ObservableMap<YoVariable, YoVariableChartPackage> charts = FXCollections.observableMap(new LinkedHashMap<>());
   private final ObservableSet<YoVariable> plottedVariables = FXCollections.observableSet(new LinkedHashSet<>());
   private YoBufferPropertiesReadOnly lastBufferProperties = null;
   private AtomicReference<YoBufferPropertiesReadOnly> newBufferProperties;
   private final TopicListener<int[]> keyFrameMarkerListener = newKeyFrames -> updateKeyFrameMarkers(newKeyFrames);
   private AtomicReference<List<String>> yoCompositeSelected;
   private Topic<List<String>> yoCompositeSelectedTopic;

   private final SimpleObjectProperty<ContextMenu> contextMenuProperty = new SimpleObjectProperty<>(this, "graphContextMenu", null);

   private Border defaultBorder = null;

   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private YoManager yoManager;
   private SessionVisualizerWindowToolkit toolkit;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      chartDataManager = toolkit.getChartDataManager();
      yoManager = toolkit.getYoManager();
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();
      topics = toolkit.getTopics();
      BackgroundExecutorManager backgroundExecutorManager = toolkit.getBackgroundExecutorManager();

      newBufferProperties = messager.createInput(topics.getYoBufferCurrentProperties());
      legendPrecision = messager.createPropertyInput(topics.getControlsNumberPrecision(), 5);

      dynamicLineChart = new DynamicLineChart(xAxis, yAxis, backgroundExecutorManager::executeInBackground, toolkit.getChartRenderManager());
      dynamicLineChart.markerAutoUpdateProperty().set(false);
      chartMainPane.getChildren().add(0, dynamicLineChart);
      AnchorPane.setTopAnchor(dynamicLineChart, 0.0);
      AnchorPane.setBottomAnchor(dynamicLineChart, 0.0);
      AnchorPane.setLeftAnchor(dynamicLineChart, 0.0);
      AnchorPane.setRightAnchor(dynamicLineChart, 0.0);

      xAxis.setLowerBound(-1);
      xAxis.setAutoRanging(false);

      yAxis.setAutoRanging(true);
      yAxis.setForceZeroInRange(false);

      inPointMarker.getStyleClass().add(INPOINT_MARKER_STYLECLASS);
      outPointMarker.getStyleClass().add(OUTPOINT_MARKER_STYLECLASS);
      bufferIndexMarker.getStyleClass().add(CURRENT_INDEX_MARKER_STYLECLASS);
      dynamicLineChart.addMarker(inPointMarker);
      dynamicLineChart.addMarker(outPointMarker);
      dynamicLineChart.addMarker(bufferIndexMarker);

      userMarkers.addListener((ListChangeListener<ChartMarker>) change ->
      {
         while (change.next())
         {
            if (change.wasAdded())
            {
               for (ChartMarker newMarker : change.getAddedSubList())
               {
                  dynamicLineChart.addMarker(newMarker);
                  if (!newMarker.getStyleClass().contains(USER_MARKER_STYLECLASS))
                     newMarker.getStyleClass().add(USER_MARKER_STYLECLASS);
               }
            }

            if (change.wasRemoved())
            {
               for (ChartMarker oldMarker : change.getRemoved())
               {
                  dynamicLineChart.removeMarker(oldMarker);
               }
            }
         }
      });

      keyFrameMarkers.addListener((ListChangeListener<ChartMarker>) change ->
      {
         while (change.next())
         {
            if (change.wasAdded())
            {
               for (ChartMarker newMarker : change.getAddedSubList())
               {
                  dynamicLineChart.addMarker(newMarker);
               }
            }

            if (change.wasRemoved())
            {
               for (ChartMarker oldMarker : change.getRemoved())
               {
                  dynamicLineChart.removeMarker(oldMarker);
               }
            }
         }
      });

      ChartMarker originMarker = new ChartMarker(new SimpleDoubleProperty(this, "origin", 0.0));
      ChangeListener<? super ChartStyle> originMarkerUpdater = (o, oldValue, newValue) ->
      {
         if (newValue == ChartStyle.RAW)
         {
            dynamicLineChart.addMarker(originMarker);
            originMarker.getStyleClass().add(ORIGIN_MARKER_STYLECLASS);
         }
         else
         {
            dynamicLineChart.removeMarker(originMarker);
         }
      };
      dynamicLineChart.chartStyleProperty().addListener(originMarkerUpdater);
      originMarkerUpdater.changed(null, null, dynamicLineChart.chartStyleProperty().get());

      dynamicLineChart.setOnDragDetected(this::handleDragDetected);
      dynamicLineChart.setOnDragOver(this::handleDragOver);
      dynamicLineChart.setOnDragDropped(this::handleDragDropped);
      dynamicLineChart.setOnDragEntered(this::handleDragEntered);
      dynamicLineChart.setOnDragExited(this::handleDragExited);
      dynamicLineChart.setOnMousePressed(this::handleMousePressed);
      dynamicLineChart.setOnMouseDragged(this::handleMouseDrag);
      dynamicLineChart.setOnMouseReleased(this::handleMouseReleased);
      dynamicLineChart.setOnScroll(this::handleScroll);
      contextMenuProperty.addListener((ChangeListener<ContextMenu>) (observable, oldValue, newValue) ->
      {
         if (oldValue != null)
            oldValue.hide();
      });
      charts.addListener((MapChangeListener<YoVariable, YoVariableChartPackage>) change ->
      {
         if (change.wasAdded())
         {
            plottedVariables.add(change.getValueAdded().getYoVariable());
            yoNumberSeriesList.add(change.getValueAdded().getSeries());
         }
         else if (change.wasRemoved())
         {
            plottedVariables.remove(change.getValueRemoved().getYoVariable());
            yoNumberSeriesList.remove(change.getValueRemoved().getSeries());
         }

         JavaFXMissingTools.runNFramesLater(1, () -> charts.values().forEach(YoVariableChartPackage::updateLegend));
      });

      messager.registerJavaFXSyncedTopicListener(topics.getCurrentKeyFrames(), keyFrameMarkerListener);
      // Only show the update markers when the session is running and the chart may be behind.
      messager.registerJavaFXSyncedTopicListener(topics.getSessionCurrentMode(),
                                                 m -> dynamicLineChart.updateIndexMarkersVisible().set(m == SessionMode.RUNNING));
      messager.submitMessage(topics.getRequestCurrentKeyFrames(), new Object());

      messager = toolkit.getMessager();
      yoCompositeSelectedTopic = toolkit.getTopics().getYoCompositeSelected();
      yoCompositeSelected = messager.createInput(yoCompositeSelectedTopic);

      // CSS style doesn't get applied immediately
      ChangeListener<? super Border> borderInitializer = new ChangeListener<Border>()
      {
         @Override
         public void changed(ObservableValue<? extends Border> o, Border oldValue, Border newValue)
         {
            defaultBorder = newValue;
            dynamicLineChart.borderProperty().removeListener(this);
         }
      };
      dynamicLineChart.borderProperty().addListener(borderInitializer);
   }

   public void setChartConfiguration(YoChartConfigurationDefinition definition)
   {
      clear();

      if (definition == null)
         return;

      if (definition.getYoVariables() != null)
      {
         definition.getYoVariables().forEach(this::addYoVariableToPlot);
      }
      else
      {
         dynamicLineChart.setChartStyle(ChartStyle.RAW);
         return;
      }

      if (definition.getChartStyle() != null)
      {
         try
         {
            dynamicLineChart.setChartStyle(ChartStyle.valueOf(definition.getChartStyle()));
         }
         catch (IllegalArgumentException e)
         {
            dynamicLineChart.setChartStyle(ChartStyle.RAW);
         }
      }
      else
      {
         dynamicLineChart.setChartStyle(ChartStyle.RAW);
      }

      for (YoVariableChartPackage pack : charts.values())
      {
         int definitionIndex = definition.getYoVariables().indexOf(pack.getYoVariable().getFullNameString());
         if (definitionIndex == -1)
            continue;

         if (definition.getYBounds() != null && definition.getYBounds().size() > definitionIndex)
         {
            ChartDoubleBoundsDefinition yBounds = definition.getYBounds().get(definitionIndex);
            pack.series.setCustomYBounds(ChartTools.toChartDoubleBounds(yBounds));
         }

         if (definition.getNegates() != null && definition.getNegates().size() > definitionIndex)
         {
            Boolean negate = definition.getNegates().get(definitionIndex);
            pack.series.setNegated(negate);
         }
      }
   }

   private final ObjectProperty<YoChartOptionController> activeChartOptionControllerProperty = new SimpleObjectProperty<>(this,
                                                                                                                          "activeChartOptionController",
                                                                                                                          null);

   @FXML
   public void openChartOptionDialog()
   {
      if (activeChartOptionControllerProperty.get() != null)
      {
         activeChartOptionControllerProperty.get().setInput(yoNumberSeriesList, dynamicLineChart.chartStyleProperty(), userMarkers);
         activeChartOptionControllerProperty.get().showWindow();
         return;
      }

      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_OPTION_DIALOG_URL);
         loader.load();
         YoChartOptionController controller = loader.getController();
         controller.initialize(toolkit);
         controller.setInput(yoNumberSeriesList, dynamicLineChart.chartStyleProperty(), userMarkers);
         activeChartOptionControllerProperty.set(controller);
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void clear()
   {
      new ArrayList<>(charts.keySet()).forEach(this::removeYoVariableFromPlot);
   }

   public void addYoVariableToPlot(String yoVariableFullName)
   {
      YoVariableDatabase rootRegistryDatabase = yoManager.getRootRegistryDatabase();
      YoVariable yoVariable = rootRegistryDatabase.searchExact(yoVariableFullName);
      if (yoVariable == null)
      {
         LogTools.warn("Incompatible variable name, searching similar variables to " + yoVariableFullName);
         yoVariable = rootRegistryDatabase.searchSimilar(yoVariableFullName, 0.90);
      }
      if (yoVariable == null)
      {
         LogTools.warn("Could not find YoVariable: " + yoVariableFullName);
         return;
      }
      addYoVariableToPlot(yoVariable);
   }

   public void addYoVariableToPlot(YoVariable yoVariable)
   {
      if (charts.containsKey(yoVariable))
         return;
      charts.put(yoVariable, new YoVariableChartPackage(yoVariable));
   }

   public void addYoVariablesToPlot(Collection<? extends YoVariable> yoVariables)
   {
      yoVariables.forEach(this::addYoVariableToPlot);
   }

   public void addYoCompositeToPlot(YoComposite yoComposite)
   {
      addYoVariablesToPlot(yoComposite.getYoComponents());
   }

   public void removeYoVariableFromPlot(String yoVariableFullName)
   {
      removeYoVariableFromPlot(yoManager.getRootRegistry().getVariable(yoVariableFullName));
   }

   public void removeYoVariableFromPlot(YoVariable yoVariable)
   {
      YoVariableChartPackage chart = charts.remove(yoVariable);
      if (chart != null)
         chart.close();
   }

   public void closeAndDispose()
   {
      stop();

      if (activeChartOptionControllerProperty.get() != null)
         activeChartOptionControllerProperty.get().close();

      ArrayList<YoVariableChartPackage> chartsCopy = new ArrayList<>(charts.values());
      charts.clear();
      chartsCopy.forEach(YoVariableChartPackage::close);

      messager.removeInput(topics.getYoBufferCurrentProperties(), newBufferProperties);
      messager.removeJavaFXSyncedTopicListener(topics.getCurrentKeyFrames(), keyFrameMarkerListener);
   }

   public boolean isEmpty()
   {
      return charts.isEmpty();
   }

   private void updateKeyFrameMarkers(int[] newKeyFrames)
   {
      keyFrameMarkers.clear();

      if (newKeyFrames == null)
         return;

      for (int keyFrame : newKeyFrames)
      {
         ChartMarker newMarker = new ChartMarker(new SimpleDoubleProperty(this, "keyFrameMarkerCoordinate" + keyFrameMarkers.size(), keyFrame));
         newMarker.getStyleClass().add(KEYFRAME_MARKER_STYLECLASS);
         keyFrameMarkers.add(newMarker);
      }
   }

   private long legendUpdateLastTime = -1L;

   @Override
   public void handleImpl(long now)
   {
      ChartIntegerBounds chartsBounds = toolkit.getChartZoomManager().chartBoundsProperty().getValue();
      YoBufferPropertiesReadOnly bufferProperties = newBufferProperties.getAndSet(null);

      if (bufferProperties != null)
      {
         lastBufferProperties = bufferProperties;

         if (bufferProperties.getInPoint() != inPointMarker.coordinateProperty().intValue())
            inPointMarker.setCoordinate(bufferProperties.getInPoint());
         if (bufferProperties.getOutPoint() != outPointMarker.coordinateProperty().intValue())
            outPointMarker.setCoordinate(bufferProperties.getOutPoint());
         if (bufferProperties.getCurrentIndex() != bufferIndexMarker.coordinateProperty().intValue())
            bufferIndexMarker.setCoordinate(bufferProperties.getCurrentIndex());
         if (chartsBounds == null)
         {
            double scale = 0.001;
            xAxis.setLowerBound(-scale * bufferProperties.getSize());
            xAxis.setUpperBound((1.0 + scale) * bufferProperties.getSize());
         }

         boolean updateLegends = legendUpdateLastTime == -1L || now - legendUpdateLastTime >= LEGEND_UPDATE_PERIOD;
         if (updateLegends)
         {
            legendUpdateLastTime = now;
            charts.values().forEach(YoVariableChartPackage::updateLegend);
         }

         bufferProperties = null;
      }

      if (chartsBounds != null)
      {
         double scale = 0.001;
         xAxis.setLowerBound(chartsBounds.getLower() - scale * chartsBounds.length());
         xAxis.setUpperBound(chartsBounds.getUpper() + scale * chartsBounds.length());
      }

      charts.values().forEach(YoVariableChartPackage::updateChart);
      if (!dynamicLineChart.markerAutoUpdateProperty().get())
         dynamicLineChart.updateMarkers();
   }

   private ContextMenu newGraphContextMenu()
   {
      ContextMenu contextMenu = new ContextMenu();
      for (YoVariable yoVariable : charts.keySet())
      {
         MenuItem menuItem = new MenuItem("Remove " + yoVariable.getName());
         menuItem.setMnemonicParsing(false);
         menuItem.setOnAction(e -> removeYoVariableFromPlot(yoVariable));
         contextMenu.getItems().add(menuItem);
      }
      return contextMenu;
   }

   private void hideContextMenu()
   {
      if (contextMenuProperty.get() != null)
         contextMenuProperty.set(null);
   }

   private void handleMousePressed(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY)
      {
         hideContextMenu();

         if (lastBufferProperties != null)
         {
            Node intersectedNode = event.getPickResult().getIntersectedNode();

            if (intersectedNode == null || intersectedNode instanceof DynamicChartLegend || intersectedNode instanceof Text || intersectedNode instanceof Label)
               return; // Don't perform scroll when clicking on the legend

            int index = screenToBufferIndex(event.getScreenX(), event.getScreenY());
            messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), index);
            event.consume();
         }
      }
      else if (event.isMiddleButtonDown() && event.getPickResult().getIntersectedNode() instanceof Text)
      { // TODO The legend's name needs to be unique within a single graph
         String pickedName = ((Text) event.getPickResult().getIntersectedNode()).getText();
         Optional<YoVariableChartPackage> chartData = charts.values()
                                                            .stream()
                                                            .filter(dataPackage -> dataPackage.series.getSeriesName().equals(pickedName))
                                                            .findFirst();
         if (chartData.isPresent())
         {
            removeYoVariableFromPlot(chartData.get().getYoVariable());
            messager.submitMessage(topics.getYoCompositeSelected(), Arrays.asList(YoCompositeTools.YO_VARIABLE, null));
         }
      }
   }

   private Point2D lastMouseScreenPosition = null;

   private void handleMouseDrag(MouseEvent event)
   {
      handleMousePressed(event);

      if (event.isSecondaryButtonDown())
      {
         Point2D newMouseScreenPosition = new Point2D(event.getScreenX(), event.getScreenY());

         if (!event.isStillSincePress())
         {
            hideContextMenu();

            if (lastMouseScreenPosition != null)
            {
               int drag = screenToBufferIndex(lastMouseScreenPosition) - screenToBufferIndex(newMouseScreenPosition);
               messager.submitMessage(topics.getYoChartRequestShift(), new Pair<>(toolkit.getWindow(), drag));
            }
         }

         lastMouseScreenPosition = newMouseScreenPosition;
         event.consume();
      }
   }

   private void handleMouseReleased(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY)
      {
         if (event.isStillSincePress())
         {
            Node intersectedNode = event.getPickResult().getIntersectedNode();
            if (intersectedNode instanceof Text)
            {
               Text legend = (Text) intersectedNode;
               String yoVariableName = legend.getText().split("\\s+")[0];
               YoVariable yoVariableSelected = charts.keySet()
                                                     .stream()
                                                     .filter(yoVariable -> yoVariable.getName().equals(yoVariableName))
                                                     .findFirst()
                                                     .orElse(null);
               if (yoVariableSelected == null)
                  return;
               messager.submitMessage(yoCompositeSelectedTopic, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariableSelected.getFullNameString()));
            }
         }
      }
      else if (event.getButton() == MouseButton.SECONDARY)
      {
         lastMouseScreenPosition = null;

         if (event.isStillSincePress())
         {
            ContextMenu contextMenu = newGraphContextMenu();
            if (!contextMenu.getItems().isEmpty())
            {
               contextMenuProperty.set(contextMenu);
               contextMenu.show(dynamicLineChart, event.getScreenX(), event.getScreenY());
            }
            event.consume();
         }
      }
      else if (event.getButton() == MouseButton.MIDDLE)
      {
         if (yoCompositeSelected.get() != null)
         {
            String type = yoCompositeSelected.get().get(0);
            // TODO For now only handling single YoVariable
            if (YoCompositeTools.YO_VARIABLE.equals(type))
            {
               String fullname = yoCompositeSelected.get().get(1);
               if (fullname != null)
               {
                  YoComposite yoComposite = yoCompositeSearchManager.getYoComposite(type, fullname);
                  if (yoComposite != null)
                  {
                     addYoCompositeToPlot(yoComposite);
                     messager.submitMessage(yoCompositeSelectedTopic, null);
                  }
               }
            }
         }
      }
   }

   private int screenToBufferIndex(Tuple2DReadOnly screenPosition)
   {
      return screenToBufferIndex(screenPosition.getX(), screenPosition.getY());
   }

   private int screenToBufferIndex(double screenX, double screenY)
   {
      if (lastBufferProperties == null)
         return -1;
      double xLocal = xAxis.screenToLocal(screenX, screenY).getX();
      int index = (int) Math.round(xAxis.getValueForDisplay(xLocal));
      return MathTools.clamp(index, 0, lastBufferProperties.getSize());
   }

   private void handleScroll(ScrollEvent event)
   {
      if (lastBufferProperties != null)
      {
         int scrollDelta = event.isControlDown() ? 10 : 1;
         if (event.getDeltaY() == 0.0)
            return;

         hideContextMenu();

         if (event.getDeltaY() < 0.0)
            messager.submitMessage(topics.getYoBufferDecrementCurrentIndexRequest(), scrollDelta);
         else
            messager.submitMessage(topics.getYoBufferIncrementCurrentIndexRequest(), scrollDelta);
      }
   }

   public void handleDragDetected(MouseEvent event)
   {
      if ((event == null) || !event.isPrimaryButtonDown())
         return;

      PickResult pickResult = event.getPickResult();

      if (pickResult == null)
         return;

      Node intersectedNode = pickResult.getIntersectedNode();

      if (intersectedNode == null)
         return;

      if (intersectedNode instanceof Text)
      {
         Text legend = (Text) intersectedNode;
         String yoVariableName = legend.getText().split("\\s+")[0];
         YoVariable yoVariableSelected = charts.keySet().stream().filter(yoVariable -> yoVariable.getName().equals(yoVariableName)).findFirst().orElse(null);
         if (yoVariableSelected == null)
            return;
         Dragboard dragBoard = legend.startDragAndDrop(TransferMode.COPY);
         ClipboardContent clipboardContent = new ClipboardContent();
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_REFERENCE, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariableSelected.getFullNameString()));
         dragBoard.setContent(clipboardContent);
      }

      event.consume();
   }

   public void setSelectionHighlight(boolean isSelected)
   {
      if (isSelected)
         dynamicLineChart.setBorder(new Border(new BorderStroke(Color.GREEN, BorderStrokeStyle.SOLID, new CornerRadii(5), BorderWidths.DEFAULT)));
      else
         dynamicLineChart.setBorder(defaultBorder);
   }

   public void handleDragEntered(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         setSelectionHighlight(true);
      event.consume();
   }

   public void handleDragExited(DragEvent event)
   {
      if (acceptDragEventForDrop(event))
         setSelectionHighlight(false);
      event.consume();
   }

   public void handleDragOver(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
      {
         if (isEmpty())
            event.acceptTransferModes(TransferMode.ANY);
         else
            event.acceptTransferModes(TransferMode.COPY);
      }
      event.consume();
   }

   public void handleDragDropped(DragEvent event)
   {
      if (event.isAccepted())
         return;

      Dragboard db = event.getDragboard();
      boolean success = false;
      List<YoComposite> yoComposites = DragAndDropTools.retrieveYoCompositesFromDragBoard(db, yoCompositeSearchManager);
      if (yoComposites != null)
      {
         for (YoComposite yoComposite : yoComposites)
            yoComposite.getYoComponents().forEach(this::addYoVariableToPlot);
         success = true;
      }
      event.setDropCompleted(success);
      event.consume();
   }

   private boolean acceptDragEventForDrop(DragEvent event)
   {
      if (event.getGestureSource() == dynamicLineChart || event.getGestureSource() == chartMoveIcon)
         return false;

      if (event.getTransferMode() == TransferMode.MOVE && !isEmpty())
         return false;

      Dragboard dragboard = event.getDragboard();
      return DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, yoCompositeSearchManager) != null;
   }

   public ObservableSet<YoVariable> getPlottedVariables()
   {
      return plottedVariables;
   }

   public Button getCloseButton()
   {
      return closeButton;
   }

   public FontAwesomeIconView getChartMoveIcon()
   {
      return chartMoveIcon;
   }

   public AnchorPane getMainPane()
   {
      return chartMainPane;
   }

   public DynamicLineChart getLineChart()
   {
      return dynamicLineChart;
   }

   public YoChartConfigurationDefinition toYoChartConfigurationDefinition()
   {
      return toYoChartConfigurationDefinition(new ChartIdentifier(-1, -1));
   }

   public YoChartConfigurationDefinition toYoChartConfigurationDefinition(ChartIdentifier chartIdentifier)
   {
      YoChartConfigurationDefinition definition = new YoChartConfigurationDefinition();
      definition.setIdentifier(ChartTools.toYoChartIdentifierDefinition(chartIdentifier));
      definition.setChartStyle(dynamicLineChart.getChartStyle().name());
      definition.setYoVariables(charts.keySet().stream().map(YoVariable::getFullNameString).collect(Collectors.toList()));
      definition.setYBounds(charts.values()
                                  .stream()
                                  .map(pack -> ChartTools.toChartDoubleBoundsDefinition(pack.getSeries().getCustomYBounds()))
                                  .collect(Collectors.toList()));
      definition.setNegates(charts.values().stream().map(pack -> pack.getSeries().isNegated()).collect(Collectors.toList()));
      return definition;
   }

   private class YoVariableChartPackage
   {
      private final YoNumberSeries series;
      private final YoVariableChartData chartData;
      private final Object callerID = YoChartPanelController.this;

      public YoVariableChartPackage(YoVariable yoVariable)
      {
         series = new YoNumberSeries(yoVariable, legendPrecision);
         chartData = chartDataManager.getYoVariableChartData(callerID, yoVariable);
         dynamicLineChart.addSeries(series);
      }

      public void updateLegend()
      {
         chartData.updateVariableData();
         series.updateLegend();
      }

      private int lastUpdateEndIndex = -1;

      public void updateChart()
      {
         ChartDataUpdate newData = chartData.pollChartData(callerID);
         if (newData != null)
         {
            newData.readUpdate(series, lastUpdateEndIndex);
            lastUpdateEndIndex = newData.getUpdateEndIndex();
         }
      }

      public YoVariable getYoVariable()
      {
         return series.getYoVariable();
      }

      public YoNumberSeries getSeries()
      {
         return series;
      }

      public void close()
      {
         dynamicLineChart.removeSeries(series);
         chartData.removeCaller(callerID);
      }
   }
}