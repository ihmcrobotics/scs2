package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.definition.yoChart.YoChartIdentifierDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.UIElement;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ChartTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.IntegerConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;

public class YoChartIdentifierEditorController implements UIElement
{
   @FXML
   private Pane mainPane;
   @FXML
   private Label chartIdLabel, rowLabel, columnLabel;
   @FXML
   private TextField rowTextField, columnTextField;

   private final ObjectProperty<YoChartIdentifierDefinition> chartIdentifierProperty = new SimpleObjectProperty<>(new YoChartIdentifierDefinition());

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      TextFormatter<Integer> rowTextFormatter = new TextFormatter<>(new IntegerConverter(), 0, new PositiveIntegerValueFilter());
      TextFormatter<Integer> columnTextFormatter = new TextFormatter<>(new IntegerConverter(), 0, new PositiveIntegerValueFilter());
      rowTextField.setTextFormatter(rowTextFormatter);
      columnTextField.setTextFormatter(columnTextFormatter);

      ObjectProperty<Integer> rowProperty = rowTextFormatter.valueProperty();
      ObjectProperty<Integer> columnProperty = columnTextFormatter.valueProperty();
      rowProperty.addListener((o, oldValue, newValue) -> chartIdentifierProperty.set(new YoChartIdentifierDefinition(newValue, columnProperty.get())));
      columnProperty.addListener((o, oldValue, newValue) -> chartIdentifierProperty.set(new YoChartIdentifierDefinition(rowProperty.get(), newValue)));
      chartIdentifierProperty.addListener((o, oldValue, newValue) ->
      {
         rowProperty.set(newValue.getRow());
         columnProperty.set(newValue.getColumn());
      });
   }

   public void setInput(int row, int column)
   {
      chartIdentifierProperty.set(new YoChartIdentifierDefinition(row, column));
   }

   public void setInput(YoChartIdentifierDefinition chartIdentifier)
   {
      setInput(ChartTools.toChartIdentifier(chartIdentifier));
   }

   public void setInput(ChartIdentifier chartIdentifier)
   {
      chartIdentifierProperty.set(ChartTools.toYoChartIdentifierDefinition(chartIdentifier));
   }

   public ObjectProperty<YoChartIdentifierDefinition> chartIdentifierProperty()
   {
      return chartIdentifierProperty;
   }

   public Label getChartIdLabel()
   {
      return chartIdLabel;
   }

   public Label getRowLabel()
   {
      return rowLabel;
   }

   public Label getColumnLabel()
   {
      return columnLabel;
   }

   public void closeAndDispose()
   {
      
   }

   @Override
   public Pane getMainPane()
   {
      return mainPane;
   }
}
