package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import com.jfoenix.controls.JFXCheckBox;
import com.jfoenix.controls.JFXComboBox;
import com.jfoenix.controls.JFXTextField;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TitledPane;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartDoubleBounds;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartOptionController.ChartScalingMode;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoChartVariableOptionController
{
   @FXML
   private TitledPane mainPane;
   @FXML
   private JFXComboBox<ChartVariableScalingMode> scalingComboBox;
   @FXML
   private JFXCheckBox negateCheckBox;
   @FXML
   private JFXTextField manualRangeMinTextField, manualRangeMaxTextField;
   @FXML
   private Label actualRangeMinLabel, actualRangeMaxLabel;

   public enum ChartVariableScalingMode
   {
      AUTO, MANUAL
   };

   private final ObjectProperty<ChartDoubleBounds> actualYBoundsProperty = new SimpleObjectProperty<>(this, "actualYBounds", null);
   private final ObjectProperty<ChartDoubleBounds> manualYBoundsProperty = new SimpleObjectProperty<>(this, "manualYBounds", null);
   private final TextFormatter<Double> minFormatter = new TextFormatter<>(new DoubleStringConverter(), 0.0);
   private final TextFormatter<Double> maxFormatter = new TextFormatter<>(new DoubleStringConverter(), 0.0);

   private YoNumberSeries series;
   private Property<ChartScalingMode> masterScalingModeProperty;

   private final ChangeListener<Boolean> negateUpdater = (o, oldValue, newValue) -> series.setNegated(newValue);
   private final ChangeListener<ChartScalingMode> globalScalingListener = (o, oldValue, newValue) -> setGlobalScaling(newValue);
   private final ChangeListener<ChartDoubleBounds> customBoundsUpdater = (o, oldValue, newValue) -> series.setCustomYBounds(newValue);
   private final ChangeListener<ChartVariableScalingMode> localScalingListener = (o, oldValue, newValue) -> setLocalScaling(newValue);
   private final ChangeListener<ChartDoubleBounds> actualYBoundsUpdater = (o, oldValue, newValue) -> actualYBoundsProperty.set(newValue);

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      scalingComboBox.setItems(FXCollections.observableArrayList(ChartVariableScalingMode.values()));
      scalingComboBox.setValue(ChartVariableScalingMode.AUTO);
      manualRangeMinTextField.setTextFormatter(minFormatter);
      manualRangeMaxTextField.setTextFormatter(maxFormatter);
      actualRangeMinLabel.setText(YoChartOptionController.UNDEFINED);
      actualRangeMaxLabel.setText(YoChartOptionController.UNDEFINED);
      minFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (manualRangeMinTextField.isDisabled())
            return;
         if (newValue > manualYBoundsProperty.get().getUpper())
         {
            minFormatter.setValue(oldValue);
            return;
         }

         manualYBoundsProperty.set(new ChartDoubleBounds(newValue.doubleValue(), manualYBoundsProperty.get().getUpper()));
      });
      maxFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (manualRangeMinTextField.isDisabled())
            return;
         if (newValue < manualYBoundsProperty.get().getLower())
         {
            maxFormatter.setValue(oldValue);
            return;
         }

         manualYBoundsProperty.set(new ChartDoubleBounds(manualYBoundsProperty.get().getLower(), newValue.doubleValue()));
      });
   }

   public void setInput(YoNumberSeries series, Property<ChartScalingMode> masterScalingModeProperty)
   {
      this.series = series;
      this.masterScalingModeProperty = masterScalingModeProperty;
      mainPane.setText(series.getYoVariable().getName());

      manualYBoundsProperty.set(series.getCustomYBounds());

      if (masterScalingModeProperty.getValue() == ChartScalingMode.INDIVIDUAL && manualYBoundsProperty.get() != null)
         scalingComboBox.setValue(ChartVariableScalingMode.MANUAL);
      else
         scalingComboBox.setValue(ChartVariableScalingMode.AUTO);

      negateCheckBox.setSelected(series.isNegated());
      negateCheckBox.selectedProperty().addListener(negateUpdater);

      setGlobalScaling(masterScalingModeProperty.getValue());
      masterScalingModeProperty.addListener(globalScalingListener);

      setLocalScaling(scalingComboBox.getValue());
      scalingComboBox.valueProperty().addListener(localScalingListener);

      series.yBoundsProperty().addListener(actualYBoundsUpdater);

      actualYBoundsProperty.addListener((o, oldValue, newValue) ->
      {
         if (newValue == null)
         {
            actualRangeMinLabel.setText(YoChartOptionController.UNDEFINED);
            actualRangeMaxLabel.setText(YoChartOptionController.UNDEFINED);
         }
         else
         {
            if (manualYBoundsProperty.get() == null)
            {
               minFormatter.setValue(newValue.getLower());
               maxFormatter.setValue(newValue.getUpper());
            }
            actualRangeMinLabel.setText(Double.toString(newValue.getLower()));
            actualRangeMaxLabel.setText(Double.toString(newValue.getUpper()));
         }
      });

      if (series.yBoundsProperty().getValue() != null)
         actualYBoundsProperty.set(new ChartDoubleBounds(series.yBoundsProperty().getValue()));
      else
         actualYBoundsProperty.set(null);
   }

   private void setGlobalScaling(ChartScalingMode mode)
   {
      scalingComboBox.setDisable(mode != ChartScalingMode.INDIVIDUAL);

      if (mode != ChartScalingMode.INDIVIDUAL)
      {
         manualRangeMinTextField.setDisable(true);
         manualRangeMaxTextField.setDisable(true);
      }
   }

   private void setLocalScaling(ChartVariableScalingMode mode)
   {
      manualRangeMinTextField.setDisable(mode == ChartVariableScalingMode.AUTO);
      manualRangeMaxTextField.setDisable(mode == ChartVariableScalingMode.AUTO);

      if (mode == ChartVariableScalingMode.MANUAL)
      {
         if (manualYBoundsProperty.get() == null)
            manualYBoundsProperty.set(new ChartDoubleBounds(actualYBoundsProperty.get()));
         minFormatter.setValue(manualYBoundsProperty.get().getLower());
         maxFormatter.setValue(manualYBoundsProperty.get().getUpper());
         manualYBoundsProperty.addListener(customBoundsUpdater);
         series.setCustomYBounds(manualYBoundsProperty.get());
      }
      else
      {
         manualYBoundsProperty.removeListener(customBoundsUpdater);
         series.setCustomYBounds(null);
      }
   }

   public void detachListeners()
   {
      masterScalingModeProperty.removeListener(globalScalingListener);
      scalingComboBox.valueProperty().removeListener(localScalingListener);
      manualYBoundsProperty.removeListener(customBoundsUpdater);
      series.yBoundsProperty().removeListener(actualYBoundsUpdater);
   }

   public YoNumberSeries getSeries()
   {
      return series;
   }

   public TitledPane getMainPane()
   {
      return mainPane;
   }
}