package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.List;

import javafx.beans.property.DoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.css.CssMetaData;
import javafx.css.Styleable;
import javafx.css.StyleableObjectProperty;
import javafx.css.StyleablePropertyFactory;
import javafx.scene.CacheHint;
import javafx.scene.chart.InvisibleNumberAxis;
import javafx.scene.shape.Line;

public final class ChartMarker extends Line
{
   private static final StyleablePropertyFactory<ChartMarker> FACTORY = new StyleablePropertyFactory<>(ChartMarker.getClassCssMetaData());
   private static final CssMetaData<ChartMarker, ChartMarkerType> MARKER_TYPE = FACTORY.createEnumCssMetaData(ChartMarkerType.class,
                                                                                                              "-marker-type",
                                                                                                              s -> s.typeProperty,
                                                                                                              ChartMarkerType.HORIZONTAL);

   private final StyleableObjectProperty<ChartMarkerType> typeProperty = new StyleableObjectProperty<ChartMarkerType>(ChartMarkerType.HORIZONTAL)
   {
      @Override
      public String getName()
      {
         return "markerType";
      }

      @Override
      public Object getBean()
      {
         return ChartMarker.this;
      }

      @Override
      public CssMetaData<? extends Styleable, ChartMarkerType> getCssMetaData()
      {
         return MARKER_TYPE;
      }
   };

   private final DoubleProperty coordinate;
   private final ObservableList<ChangeListener<Object>> listeners = FXCollections.observableArrayList();

   public ChartMarker(DoubleProperty coordinate)
   {
      this.coordinate = coordinate;
      listeners.addListener((ListChangeListener<ChangeListener<Object>>) change ->
      {
         while (change.next())
         {
            if (change.wasAdded())
            {
               change.getAddedSubList().forEach(listener ->
               { // Workaround for updating the marker when the style changes but the coordinate does not.
                  coordinate.addListener(listener);
                  typeProperty.addListener(listener);
                  strokeProperty().addListener(listener);
                  strokeWidthProperty().addListener(listener);
                  strokeTypeProperty().addListener(listener);
                  strokeLineCapProperty().addListener(listener);
                  strokeDashOffsetProperty().addListener(listener);
                  strokeDashOffsetProperty().addListener(listener);
               });
            }

            if (change.wasRemoved())
            {
               change.getRemoved().forEach(listener ->
               {
                  coordinate.removeListener(listener);
                  typeProperty.removeListener(listener);
                  strokeProperty().removeListener(listener);
                  strokeWidthProperty().removeListener(listener);
                  strokeTypeProperty().removeListener(listener);
                  strokeLineCapProperty().removeListener(listener);
                  strokeDashOffsetProperty().removeListener(listener);
                  strokeDashOffsetProperty().removeListener(listener);
               });
            }
         }
      });

      setSmooth(false);
      setManaged(false);
      setCache(true);
      setCacheHint(CacheHint.SPEED);
   }

   public ChartMarker(ChartMarkerType type, DoubleProperty coordinate)
   {
      this(coordinate);
      this.typeProperty.set(type);
   }

   public void setCoordinate(double coordinate)
   {
      this.coordinate.set(coordinate);
   }

   public double getCoordinate()
   {
      return coordinate.get();
   }

   public DoubleProperty coordinateProperty()
   {
      return coordinate;
   }

   public void addListener(ChangeListener<Object> listener)
   {
      listeners.add(listener);
   }

   public void removeListener(ChangeListener<Object> listener)
   {
      listeners.remove(listener);
   }

   public void updateMarker(InvisibleNumberAxis xAxis, InvisibleNumberAxis yAxis)
   {
      if (typeProperty.get() == ChartMarkerType.HORIZONTAL)
      {
         setStartX(0.0);
         setEndX(xAxis.getWidth());
         setStartY(0.0);
         setEndY(0.0);
         setTranslateX(0.0);
         setTranslateY(Math.ceil(yAxis.getDisplayPosition(coordinate.get())));
      }
      else if (typeProperty.get() == ChartMarkerType.VERTICAL)
      {
         setStartY(0.0);
         setEndY(yAxis.getHeight());
         setStartX(0.0);
         setEndX(0.0);
         setEndX(0.0);

         setTranslateY(0.0);
         setTranslateX(Math.ceil(xAxis.getDisplayPosition(coordinate.get())));
      }
   }

   public void destroy()
   {
      listeners.clear();
   }

   @Override
   public List<CssMetaData<? extends Styleable, ?>> getCssMetaData()
   {
      return FACTORY.getCssMetaData();
   }
}