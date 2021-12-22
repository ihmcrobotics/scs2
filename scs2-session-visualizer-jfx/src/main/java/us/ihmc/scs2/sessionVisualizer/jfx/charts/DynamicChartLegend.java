package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.DoublePropertyBase;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ObjectPropertyBase;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.scene.layout.FlowPane;

// TODO Modify the gap to use CSS
public class DynamicChartLegend extends FlowPane
{
   private final ListChangeListener<DynamicChartLegendItem> itemsListener = change ->
   {
      getChildren().clear();
      getChildren().addAll(change.getList());
      if (isVisible())
         requestLayout();
   };

   private final ObjectProperty<ObservableList<DynamicChartLegendItem>> items = new ObjectPropertyBase<ObservableList<DynamicChartLegendItem>>()
   {
      ObservableList<DynamicChartLegendItem> oldItems = null;

      @Override
      protected void invalidated()
      {
         if (oldItems != null)
            oldItems.removeListener(itemsListener);
         getChildren().clear();
         ObservableList<DynamicChartLegendItem> newItems = get();
         if (newItems != null)
         {
            newItems.addListener(itemsListener);
            getChildren().addAll(newItems);
         }
         oldItems = get();
         requestLayout();
      }

      @Override
      public Object getBean()
      {
         return DynamicChartLegend.this;
      }

      @Override
      public String getName()
      {
         return "items";
      }
   };

   public final void setItems(ObservableList<DynamicChartLegendItem> value)
   {
      itemsProperty().set(value);
   }

   public final ObservableList<DynamicChartLegendItem> getItems()
   {
      return items.get();
   }

   public final ObjectProperty<ObservableList<DynamicChartLegendItem>> itemsProperty()
   {
      return items;
   }

   public DynamicChartLegend()
   {
      setItems(FXCollections.observableArrayList());
      getStyleClass().add("chart-legend");
   }

   @Override
   protected double computePrefWidth(double forHeight)
   {
      return getItems().isEmpty() ? 0 : super.computePrefWidth(forHeight);
   }

   @Override
   protected double computePrefHeight(double forWidth)
   {
      return getItems().isEmpty() ? 0 : super.computePrefHeight(forWidth);
   }

   private DoubleProperty gapProperty = null;

   public double getGap()
   {
      return gapProperty != null ? gapProperty.get() : 8;
   }

   public void setGap(double value)
   {
      gapProperty().set(value);
   }

   public DoubleProperty gapProperty()
   {
      if (gapProperty == null)
      {
         gapProperty = new DoublePropertyBase(8)
         {
            @Override
            protected void invalidated()
            {
               DynamicChartLegend.this.setHgap(get());
               DynamicChartLegend.this.setVgap(get());
            }

            @Override
            public String getName()
            {
               return "gap";
            }

            @Override
            public Object getBean()
            {
               return DynamicChartLegend.this;
            }
         };
      }
      return gapProperty;
   }
}
