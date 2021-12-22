package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXColorPicker;
import com.jfoenix.controls.JFXTextField;

import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.TextFormatter;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartMarker;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.UIElement;

public class YoChartBaselineEditorPaneController implements UIElement
{
   @FXML
   private HBox mainPane;
   @FXML
   private JFXTextField coordinateTextField;
   @FXML
   private JFXColorPicker colorPicker;

   private final TextFormatter<Double> coordinateFormatter = new TextFormatter<>(new DoubleStringConverter());
   private ChartMarker marker;

   private Runnable closeRequestListener = null;
   private MutableBoolean updatingColor = new MutableBoolean(false);
   private ChangeListener<Color> markerStrokeUpdater = (o, oldValue, newValue) ->
   {
      if (updatingColor.isTrue())
         return;
      updatingColor.setTrue();
      marker.setStroke(newValue);
      updatingColor.setFalse();
   };

   private ChangeListener<Paint> colorPickerUpdater = (o, oldValue, newValue) ->
   {
      if (updatingColor.isTrue())
         return;
      updatingColor.setTrue();
      if (newValue instanceof Color)
         colorPicker.setValue((Color) newValue);
      updatingColor.setFalse();
   };

   private MutableBoolean updateCoordinate = new MutableBoolean(false);

   private ChangeListener<? super Number> coordinateFormatterUpdater = (o, oldValue, newValue) ->
   {
      if (updateCoordinate.isTrue())
         return;
      updateCoordinate.setTrue();
      coordinateFormatter.setValue(newValue.doubleValue());
      updateCoordinate.setFalse();
   };

   private ChangeListener<? super Number> coordinateMarkerUpdater = (o, oldValue, newValue) ->
   {
      if (updateCoordinate.isTrue())
         return;
      updateCoordinate.setTrue();
      marker.setCoordinate(newValue.doubleValue());
      updateCoordinate.setFalse();
   };

   public void setInput(ChartMarker marker, Runnable closeRequestListener)
   {
      this.marker = marker;
      this.closeRequestListener = closeRequestListener;
      coordinateTextField.setTextFormatter(coordinateFormatter);
      // TODO The use of bidirectional bindings to bind a DoubleProperty via DoubleProperty.asObject() doesn't work as the binding and/or the object returned by DoubleProperty.asObject() gets garbage collected, kinda silly.
      // See: https://bugs.openjdk.java.net/browse/JDK-8093223
      //      marker.coordinateProperty().asObject().bindBidirectional(coordinateFormatter.valueProperty());
      marker.coordinateProperty().addListener(coordinateFormatterUpdater);
      coordinateFormatter.valueProperty().addListener(coordinateMarkerUpdater);
      coordinateFormatterUpdater.changed(null, null, marker.getCoordinate());

      colorPicker.valueProperty().addListener(markerStrokeUpdater);
      marker.strokeProperty().addListener(colorPickerUpdater);
      colorPickerUpdater.changed(null, null, marker.getStroke());
   }

   @FXML
   public void requestClose()
   {
      if (closeRequestListener != null)
         closeRequestListener.run();
   }

   @Override
   public Pane getMainPane()
   {
      return mainPane;
   }

   public ChartMarker getMarker()
   {
      return marker;
   }

   public void dispose()
   {
      System.out.println("Disposing controller");
      marker.coordinateProperty().removeListener(coordinateFormatterUpdater);
      coordinateFormatter.valueProperty().removeListener(coordinateMarkerUpdater);
      colorPicker.valueProperty().removeListener(markerStrokeUpdater);
      marker.strokeProperty().removeListener(colorPickerUpdater);
      closeRequestListener = null;
   }
}
