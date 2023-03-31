package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.Optional;
import java.util.concurrent.Executor;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.geometry.Orientation;
import javafx.geometry.Side;
import javafx.scene.Group;
import javafx.scene.chart.FastAxisBase;
import us.ihmc.javaFXExtensions.chart.DynamicXYChart;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ChartRenderManager;

public class DynamicLineChart extends DynamicXYChart
{
   public enum ChartStyle
   {
      RAW, NORMALIZED
   }

   private final Group seriesGroup = new Group()
   {
      @Override
      public void requestLayout()
      {
      } // suppress layout requests
   };
   private final Group markerGroup = new Group()
   {
      @Override
      public void requestLayout()
      {
      } // suppress layout requests
   };

   private final BooleanProperty markerAutoUpdateProperty = new SimpleBooleanProperty(this, "markerAutoUpdate", true);
   private final ObjectProperty<ChartStyle> chartStyleProperty = new SimpleObjectProperty<>(this, "chartStyle", ChartStyle.RAW);
   private final ObservableList<NumberSeriesLayer> seriesLayers = FXCollections.observableArrayList();
   private final ChangeListener<Object> chartUpdaterListener = (o, oldValue, newValue) -> requestChartLayout();
   private final ObservableList<ChartMarker> markers = FXCollections.observableArrayList();
   private final DynamicChartLegend legend = new DynamicChartLegend();

   private final BooleanProperty updateIndexMarkersVisible = new SimpleBooleanProperty(this, "updateIndexMarkersVisible", false);

   private final Executor backgroundExecutor;
   private final ChartRenderManager chartRenderManager;
   private final ChangeListener<? super Boolean> autoRangingListener = (ov, t, t1) -> updateAxisRange();

   public DynamicLineChart(FastAxisBase xAxis, FastAxisBase yAxis, Executor backgroundExecutor, ChartRenderManager chartRenderManager)
   {
      super(xAxis, yAxis);

      this.chartRenderManager = chartRenderManager;

      this.backgroundExecutor = backgroundExecutor;

      ChangeListener<? super FastAxisBase> xAxisChangeListener = (o, oldAxis, newAxis) ->
      {
         if (oldAxis != null)
            oldAxis.autoRangingProperty().removeListener(autoRangingListener);

         if (newAxis.getSide() == null)
            newAxis.setSide(Side.BOTTOM);
         newAxis.setEffectiveOrientation(Orientation.HORIZONTAL);
         newAxis.autoRangingProperty().addListener(autoRangingListener);
      };
      xAxisProperty().addListener(xAxisChangeListener);
      xAxisChangeListener.changed(null, null, xAxis);

      ChangeListener<? super FastAxisBase> yAxisChangeListener = (o, oldAxis, newAxis) ->
      {
         if (oldAxis != null)
            oldAxis.autoRangingProperty().removeListener(autoRangingListener);
         if (newAxis.getSide() == null)
            newAxis.setSide(Side.LEFT);
         newAxis.setEffectiveOrientation(Orientation.VERTICAL);
         newAxis.autoRangingProperty().addListener(autoRangingListener);
      };
      yAxisProperty().addListener(yAxisChangeListener);
      yAxisChangeListener.changed(null, null, yAxis);

      chartStyleProperty.addListener(chartUpdaterListener);

      plotContent.getChildren().addAll(seriesGroup, markerGroup);

      // We don't want seriesGroup/markerGroup to autoSize or do layout
      seriesGroup.setAutoSizeChildren(false);
      markerGroup.setAutoSizeChildren(false);
      // setup css style classes
      seriesGroup.getStyleClass().setAll("series-group");
      markerGroup.getStyleClass().setAll("marker-group");
      // mark seriesGroup/markerGroup as unmanaged as its preferred size changes do not effect our layout
      seriesGroup.setManaged(false);
      markerGroup.setManaged(false);

      ChangeListener<Object> updateMarkerListener = (o, oldValue, newValue) -> updateMarkers();
      markers.addListener((ListChangeListener<ChartMarker>) change ->
      {
         while (change.next())
         {
            if (change.wasAdded())
            {
               for (ChartMarker newMarker : change.getAddedSubList())
               {
                  markerGroup.getChildren().add(newMarker);
                  if (markerAutoUpdateProperty.get())
                     newMarker.addListener(updateMarkerListener);
               }
               updateMarkers();
            }

            if (change.wasRemoved())
            {
               for (ChartMarker oldMarker : change.getRemoved())
               {
                  markerGroup.getChildren().remove(oldMarker);
                  oldMarker.destroy();
               }
            }
         }
      });
      markerAutoUpdateProperty.addListener((o, oldValue, newValue) ->
      {
         if (newValue)
            markers.forEach(marker -> marker.addListener(updateMarkerListener));
         else
            markers.forEach(marker -> marker.removeListener(updateMarkerListener));
      });

      setLegend(legend);
   }

   public void addSeries(NumberSeries series)
   {
      int seriesIndex = seriesLayers.size();

      series.negatedProperty().addListener(chartUpdaterListener);
      series.customYBoundsProperty().addListener(chartUpdaterListener);
      series.dirtyProperty().addListener(chartUpdaterListener);
      NumberSeriesLayer layer = new NumberSeriesLayer(xAxis, yAxis, series, backgroundExecutor, chartRenderManager);
      layer.chartStyleProperty().bind(chartStyleProperty);
      layer.updateIndexMarkerVisibleProperty().bind(updateIndexMarkersVisible);
      setSeriesDefaultStyleClass(layer, seriesIndex);
      seriesLayers.add(layer);
      legend.getItems().add(layer.getLegendNode());
      // Add the plot under the markers
      seriesGroup.getChildren().add(layer);
      chartUpdaterListener.changed(null, null, null);
   }

   public void removeSeries(NumberSeries series)
   {
      Optional<NumberSeriesLayer> containingLayer = seriesLayers.stream().filter(layer -> layer.getNumberSeries() == series).findFirst();

      if (containingLayer.isPresent())
      {
         int indexOf = seriesLayers.indexOf(containingLayer.get());
         NumberSeriesLayer removedLayer = seriesLayers.remove(indexOf);
         removedLayer.chartStyleProperty().unbind();
         removedLayer.updateIndexMarkerVisibleProperty().unbind();
         series.negatedProperty().removeListener(chartUpdaterListener);
         series.customYBoundsProperty().removeListener(chartUpdaterListener);
         series.dirtyProperty().removeListener(chartUpdaterListener);
         containingLayer.get().chartStyleProperty().unbind();
         seriesGroup.getChildren().remove(containingLayer.get());
         legend.getItems().remove(containingLayer.get().getLegendNode());

         for (int i = indexOf; i < seriesLayers.size(); i++)
            setSeriesDefaultStyleClass(seriesLayers.get(i), i);

         chartUpdaterListener.changed(null, null, null);
      }
   }

   public void addMarker(ChartMarker marker)
   {
      markers.add(marker);
   }

   public void removeMarker(ChartMarker marker)
   {
      markers.remove(marker);
   }

   private void updateSeriesList(double top, double left, double width, double height)
   {
      for (NumberSeriesLayer seriesLayer : seriesLayers)
      {
         seriesLayer.scheduleRender();
      }
   }

   @Override
   protected void layoutPlotChildren(double top, double left, double width, double height)
   {
      // position plot group, its origin is the bottom left corner of the plot area
      seriesGroup.setLayoutX(left);
      seriesGroup.setLayoutY(top);
      seriesGroup.requestLayout(); // Note: not sure this is right, maybe plotContent should be resizeable
      markerGroup.setLayoutX(left);
      markerGroup.setLayoutY(top);
      markerGroup.requestLayout(); // Note: not sure this is right, maybe plotContent should be resizeable

      updateSeriesList(top, left, width, height);
      if (markerAutoUpdateProperty.get())
         updateMarkers();
   }

   public void updateMarkers()
   {
      markers.forEach(marker -> marker.updateMarker(getXAxis(), getYAxis()));
   }

   @Override
   protected void updateAxisRange()
   {
      updateXAxisRange();
      updateYAxisRange();
   }

   protected void updateXAxisRange()
   {
      ChartIntegerBounds xBounds = null;

      FastAxisBase xAxis = getXAxis();

      if (xAxis.isAutoRanging())
      {
         for (NumberSeriesLayer layer : seriesLayers)
         {
            NumberSeries series = layer.getNumberSeries();

            ChartIntegerBounds dataXBounds = series.xBoundsProperty().getValue();

            if (dataXBounds == null)
               continue;

            if (xBounds == null)
               xBounds = new ChartIntegerBounds(dataXBounds);
            else
               xBounds = xBounds.union(dataXBounds);
         }

         if (xBounds != null)
            xAxis.invalidateRange(xBounds.getLower(), xBounds.getUpper());
      }
   }

   protected void updateYAxisRange()
   {
      ChartDoubleBounds yBounds = null;

      FastAxisBase yAxis = getYAxis();

      if (yAxis.isAutoRanging())
      {
         if (chartStyleProperty.get() == ChartStyle.NORMALIZED)
         {
            yAxis.invalidateRange(0.0, 1.0);
         }
         else
         {
            for (NumberSeriesLayer layer : seriesLayers)
            {
               NumberSeries series = layer.getNumberSeries();

               ChartDoubleBounds dataYBounds = series.yBoundsProperty().getValue();

               if (dataYBounds == null)
                  continue;

               if (series.getCustomYBounds() != null)
                  dataYBounds = series.getCustomYBounds();
               if (series.isNegated())
                  dataYBounds = dataYBounds.negate();

               if (yBounds == null)
                  yBounds = dataYBounds;
               else
                  yBounds = yBounds.union(dataYBounds);
            }

            if (yBounds != null)
            {
               yAxis.invalidateRange(yBounds.getLower(), yBounds.getUpper());
            }
         }
      }
   }

   /**
    * Property controlling visibility of markers used to indicate up to what index the charts have been
    * updated.
    */
   public BooleanProperty updateIndexMarkersVisible()
   {
      return updateIndexMarkersVisible;
   }

   public BooleanProperty markerAutoUpdateProperty()
   {
      return markerAutoUpdateProperty;
   }

   public void setChartStyle(ChartStyle style)
   {
      chartStyleProperty.set(style);
   }

   public ChartStyle getChartStyle()
   {
      return chartStyleProperty.get();
   }

   public ObjectProperty<ChartStyle> chartStyleProperty()
   {
      return chartStyleProperty;
   }

   private static void setSeriesDefaultStyleClass(NumberSeriesLayer seriesLayer, int seriesIndex)
   {
      seriesLayer.getStyleClass().setAll("chart-series-line", "series" + seriesIndex, getDefaultColorStyle(seriesIndex));
   }

   public static String getDefaultColorStyle(int index)
   {
      return "default-color" + index % 8;
   }
}