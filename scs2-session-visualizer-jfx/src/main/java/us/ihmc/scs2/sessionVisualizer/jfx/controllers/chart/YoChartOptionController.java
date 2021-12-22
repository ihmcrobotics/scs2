package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import java.io.IOException;
import java.util.Iterator;
import java.util.Objects;
import java.util.Optional;

import com.jfoenix.controls.JFXComboBox;
import com.jfoenix.controls.JFXTextField;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.InvalidationListener;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TitledPane;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartDoubleBounds;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartMarker;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.DynamicLineChart.ChartStyle;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class YoChartOptionController
{
   public static final String UNDEFINED = "N/A";

   @FXML
   private VBox mainPane;
   @FXML
   private TitledPane masterSettingsPane;
   @FXML
   private JFXComboBox<ChartScalingMode> scalingComboBox;
   @FXML
   private JFXTextField manualRangeMinTextField, manualRangeMaxTextField;
   @FXML
   private Label actualRangeMinLabel, actualRangeMaxLabel;
   @FXML
   private YoChartBaselinesOptionPaneController yoChartBaselinesOptionPaneController;

   public enum ChartScalingMode
   {
      AUTO, INDIVIDUAL, MANUAL
   };

   private SessionVisualizerWindowToolkit toolkit;
   private Stage window;
   private ObservableList<YoChartVariableOptionController> subControllers = FXCollections.observableArrayList();
   private ObservableList<YoNumberSeries> yoNumberSeriesList = null;
   private ObjectProperty<ChartStyle> chartStyleProperty;

   private final ObjectProperty<ChartDoubleBounds> actualYBoundsProperty = new SimpleObjectProperty<>(this, "actualYBounds", null);
   private final ObjectProperty<ChartDoubleBounds> manualYBoundsProperty = new SimpleObjectProperty<>(this, "manualYBounds", null);
   private final TextFormatter<Double> minFormatter = new TextFormatter<>(new DoubleStringConverter(), 0.0);
   private final TextFormatter<Double> maxFormatter = new TextFormatter<>(new DoubleStringConverter(), 0.0);

   private final ChangeListener<Double> manualMinListener = (o, oldValue, newValue) ->
   {
      if (manualRangeMinTextField.isDisabled())
         return;
      if (newValue > maxFormatter.getValue())
      {
         minFormatter.setValue(oldValue);
         return;
      }

      if (manualYBoundsProperty.get() == null)
         manualYBoundsProperty.set(new ChartDoubleBounds(newValue, newValue));
      else
         manualYBoundsProperty.set(new ChartDoubleBounds(newValue.doubleValue(), manualYBoundsProperty.get().getUpper()));
   };

   private final ChangeListener<Double> manualMaxListener = (o, oldValue, newValue) ->
   {
      if (manualRangeMaxTextField.isDisabled())
         return;
      if (newValue < minFormatter.getValue())
      {
         maxFormatter.setValue(oldValue);
         return;
      }

      if (manualYBoundsProperty.get() == null)
         manualYBoundsProperty.set(new ChartDoubleBounds(newValue, newValue));
      else
         manualYBoundsProperty.set(new ChartDoubleBounds(manualYBoundsProperty.get().getLower(), newValue.doubleValue()));
   };

   private final ChangeListener<ChartDoubleBounds> customBoundsUpdater = (o,
                                                                          oldValue,
                                                                          newValue) -> yoNumberSeriesList.forEach(series -> series.setCustomYBounds(newValue));

   private final ChangeListener<ChartScalingMode> scalingModeListener = (o, oldValue, newValue) ->
   {
      manualRangeMinTextField.setDisable(newValue != ChartScalingMode.MANUAL);
      manualRangeMaxTextField.setDisable(newValue != ChartScalingMode.MANUAL);

      if (newValue == ChartScalingMode.MANUAL)
      {
         if (manualYBoundsProperty.get() == null)
            manualYBoundsProperty.set(new ChartDoubleBounds(actualYBoundsProperty.get()));
         minFormatter.setValue(manualYBoundsProperty.get().getLower());
         maxFormatter.setValue(manualYBoundsProperty.get().getUpper());
         manualYBoundsProperty.addListener(customBoundsUpdater);
         customBoundsUpdater.changed(null, null, manualYBoundsProperty.get());
      }
      else
      {
         manualYBoundsProperty.removeListener(customBoundsUpdater);
      }

      if (newValue == ChartScalingMode.AUTO)
      {
         if (yoNumberSeriesList != null)
         {
            yoNumberSeriesList.forEach(series ->
            {
               series.customYBoundsProperty().unbind();
               series.setCustomYBounds(null);
            });
         }
      }

      if (chartStyleProperty != null)
      {
         if (newValue == ChartScalingMode.INDIVIDUAL)
            chartStyleProperty.set(ChartStyle.NORMALIZED);
         else
            chartStyleProperty.set(ChartStyle.RAW);
      }
   };

   private final InvalidationListener actualBoundsUpdater = o -> updateActualBounds();
   private final ChangeListener<Object> resizeWindowListener = (o, oldValue, newValue) -> resizeWindow();

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      manualRangeMinTextField.setTextFormatter(minFormatter);
      manualRangeMaxTextField.setTextFormatter(maxFormatter);
      actualRangeMinLabel.setText(UNDEFINED);
      actualRangeMaxLabel.setText(UNDEFINED);

      scalingComboBox.setItems(FXCollections.observableArrayList(ChartScalingMode.values()));
      minFormatter.valueProperty().addListener(manualMinListener);
      maxFormatter.valueProperty().addListener(manualMaxListener);
      scalingComboBox.valueProperty().addListener(scalingModeListener);
      scalingComboBox.setValue(ChartScalingMode.AUTO);

      subControllers.addListener((ListChangeListener<YoChartVariableOptionController>) change ->
      {
         while (change.next())
         {
            if (change.wasAdded())
            {
               for (int i = change.getAddedSize() - 1; i >= 0; i--)
               {
                  YoChartVariableOptionController subController = change.getAddedSubList().get(i);
                  mainPane.getChildren().add(change.getFrom() + 1, subController.getMainPane());
                  subController.getMainPane().expandedProperty().addListener(resizeWindowListener);
               }
            }

            if (change.wasRemoved())
            {
               change.getRemoved().forEach(subController ->
               {
                  subController.getMainPane().expandedProperty().removeListener(resizeWindowListener);
                  mainPane.getChildren().remove(subController.getMainPane());
                  subController.detachListeners();
               });
            }
         }

         resizeWindow();
      });

      yoChartBaselinesOptionPaneController.initialize(toolkit);
      yoChartBaselinesOptionPaneController.getMainPane().heightProperty().addListener(resizeWindowListener);

      window = new Stage(StageStyle.UTILITY);
      window.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ESCAPE)
            window.close();
      });
      toolkit.getWindow().addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            window.close();
      });
      window.setTitle("YoChart properties");
      Scene scene = new Scene(mainPane);
      masterSettingsPane.expandedProperty().addListener((o, oldValue, newValue) -> resizeWindow());
      window.setResizable(false);
      window.setScene(scene);
      window.initOwner(toolkit.getWindow());
   }

   public void showWindow()
   {
      window.setOpacity(0.0);
      window.toFront();
      window.show();
      Timeline timeline = new Timeline();
      KeyFrame key = new KeyFrame(Duration.seconds(0.125), new KeyValue(window.opacityProperty(), 1.0));
      timeline.getKeyFrames().add(key);
      timeline.play();
   }

   public void close()
   {
      window.close();
   }

   public void setInput(ObservableList<YoNumberSeries> yoNumberSeriesList,
                        ObjectProperty<ChartStyle> chartStyleProperty,
                        ObservableList<ChartMarker> userMarkers)
   {
      this.yoNumberSeriesList = yoNumberSeriesList;
      this.chartStyleProperty = chartStyleProperty;

      yoNumberSeriesList.forEach(series -> series.yBoundsProperty().addListener(actualBoundsUpdater));

      yoNumberSeriesList.addListener((ListChangeListener<YoNumberSeries>) change ->
      {
         while (change.next())
         {
            if (change.wasRemoved())
               change.getRemoved().forEach(series -> series.yBoundsProperty().removeListener(actualBoundsUpdater));
            if (change.wasAdded())
               change.getAddedSubList().forEach(series -> series.yBoundsProperty().addListener(actualBoundsUpdater));
         }

         if (!change.getList().isEmpty())
            updateActualBounds();
      });

      if (chartStyleProperty.get() == ChartStyle.RAW)
      {
         Optional<ChartDoubleBounds> customBounds = yoNumberSeriesList.stream().map(YoNumberSeries::getCustomYBounds).filter(Objects::nonNull).findFirst();
         manualYBoundsProperty.set(customBounds.isPresent() ? customBounds.get() : null);
         scalingComboBox.setValue(customBounds.isPresent() ? ChartScalingMode.MANUAL : ChartScalingMode.AUTO);
      }
      else
      {
         scalingComboBox.setValue(ChartScalingMode.INDIVIDUAL);
      }

      subControllers.clear();

      yoNumberSeriesList.forEach(this::loadAndInitializeSubController);

      yoNumberSeriesList.addListener((ListChangeListener<YoNumberSeries>) change ->
      {
         while (change.next())
         {
            if (change.wasAdded())
               change.getAddedSubList().forEach(series -> loadAndInitializeSubController(series, change.getFrom()));
            if (change.wasRemoved())
               change.getRemoved().forEach(series -> unloadController(series));
         }
      });

      actualYBoundsProperty.addListener((o, oldValue, newValue) ->
      {
         if (newValue == null)
            return;

         if (manualYBoundsProperty.get() == null)
         {
            minFormatter.setValue(newValue.getLower());
            maxFormatter.setValue(newValue.getUpper());
         }
         actualRangeMinLabel.setText(Double.toString(newValue.getLower()));
         actualRangeMaxLabel.setText(Double.toString(newValue.getUpper()));
      });

      scalingModeListener.changed(null, null, scalingComboBox.getValue());
      updateActualBounds();

      yoChartBaselinesOptionPaneController.setInput(userMarkers);

      resizeWindow();
   }

   private void resizeWindow()
   {
      JavaFXMissingTools.runLater(getClass(), () -> window.sizeToScene());
   }

   private void updateActualBounds()
   {
      ChartDoubleBounds newBounds = null;

      for (YoNumberSeries series : yoNumberSeriesList)
      {
         ChartDoubleBounds dataYBounds = series.yBoundsProperty().getValue();
         if (dataYBounds == null)
            continue;
         if (newBounds == null)
            newBounds = new ChartDoubleBounds(dataYBounds);
         else
            newBounds = newBounds.union(dataYBounds);
      }

      actualYBoundsProperty.set(newBounds);
   }

   private void unloadController(YoNumberSeries series)
   {
      Iterator<YoChartVariableOptionController> iterator = subControllers.iterator();

      while (iterator.hasNext())
      {
         YoChartVariableOptionController subController = iterator.next();

         if (subController.getSeries() == series)
         {
            iterator.remove();
            return;
         }
      }
   }

   private void loadAndInitializeSubController(YoNumberSeries series)
   {
      loadAndInitializeSubController(series, subControllers.size());
   }

   private void loadAndInitializeSubController(YoNumberSeries series, int insertionIndex)
   {
      YoChartVariableOptionController subController = loadSubController();
      subController.initialize(toolkit);
      subController.setInput(series, scalingComboBox.valueProperty());
      subControllers.add(insertionIndex, subController);
   }

   private YoChartVariableOptionController loadSubController()
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_VARIABLE_OPTION_PANE_URL);
         loader.load();
         return loader.getController();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }
}