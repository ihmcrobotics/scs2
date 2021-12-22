package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import javafx.beans.property.StringProperty;
import javafx.scene.control.Label;
import javafx.scene.layout.HBox;
import javafx.scene.paint.Paint;

public class DynamicChartLegendItem extends HBox
{
   private final Label seriesNameLabel = new Label();
   private final CurrentValueLabel currentValueLabel = new CurrentValueLabel();

   public DynamicChartLegendItem()
   {
      getChildren().addAll(seriesNameLabel, currentValueLabel);
      getStyleClass().add("chart-legend-item");
      seriesNameLabel.getStyleClass().add("chart-legend-item-name");
      currentValueLabel.getStyleClass().add("chart-legend-item-value");
   }

   public void setTextFill(Paint paint)
   {
      seriesNameLabel.setTextFill(paint);
      currentValueLabel.setTextFill(paint);
   }

   public StringProperty seriesNameProperty()
   {
      return seriesNameLabel.textProperty();
   }

   public StringProperty currentValueProperty()
   {
      return currentValueLabel.textProperty();
   }

   private static class CurrentValueLabel extends Label
   {
      private double maxPrefWidth = -1.0;

      public CurrentValueLabel()
      {
      }

      @Override
      protected double computePrefWidth(double height)
      {
         // Value displayed may change in size, i.e. switching positive/negative or exponent changes. Retaining the max pref width to avoid flickering of the layout. 
         double prefWidth = super.computePrefWidth(height);
         maxPrefWidth = Math.max(prefWidth, maxPrefWidth);
         return maxPrefWidth;
      }
   }
}