package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.List;

import javafx.beans.property.DoubleProperty;
import javafx.collections.ObservableList;
import javafx.css.CssMetaData;
import javafx.css.Styleable;
import javafx.css.StyleableDoubleProperty;
import javafx.css.StyleablePropertyFactory;
import javafx.scene.shape.Polygon;

public class TrimPolygonMarker extends Polygon
{
   private static final StyleablePropertyFactory<TrimPolygonMarker> FACTORY = new StyleablePropertyFactory<>(TrimPolygonMarker.getClassCssMetaData());
   private static final CssMetaData<TrimPolygonMarker, Number> MARKER_WIDTH = FACTORY.createSizeCssMetaData("-fx-marker-width",
                                                                                                            s -> s.markerWidth,
                                                                                                            1.0,
                                                                                                            false);
   private static final CssMetaData<TrimPolygonMarker, Number> MARKER_HEIGHT = FACTORY.createSizeCssMetaData("-fx-marker-height",
                                                                                                             s -> s.markerHeight,
                                                                                                             1.0,
                                                                                                             false);

   private final StyleableDoubleProperty markerWidth = new StyleableDoubleProperty(1.0)
   {
      @Override
      public String getName()
      {
         return "markerWidth";
      }

      @Override
      public Object getBean()
      {
         return TrimPolygonMarker.this;
      }

      @Override
      public CssMetaData<? extends Styleable, Number> getCssMetaData()
      {
         return MARKER_WIDTH;
      }
   };

   private final StyleableDoubleProperty markerHeight = new StyleableDoubleProperty(1.0)
   {
      @Override
      public String getName()
      {
         return "markerHeight";
      }

      @Override
      public Object getBean()
      {
         return TrimPolygonMarker.this;
      }

      @Override
      public CssMetaData<? extends Styleable, Number> getCssMetaData()
      {
         return MARKER_HEIGHT;
      }
   };

   public TrimPolygonMarker()
   {
      super();
      initListeners();
   }

   public TrimPolygonMarker(double... points)
   {
      super(points);
      initListeners();
   }

   public void setPoints(double... points)
   {
      if (points != null)
      {
         getPoints().clear();

         for (double d : points)
         {
            getPoints().add(d);
         }

         updateWidth(1.0, markerWidth.get());
         updateHeight(1.0, markerHeight.get());
      }
   }

   private void initListeners()
   {
      markerWidth.addListener((o, oldValue, newValue) -> updateWidth(oldValue, newValue));
      markerHeight.addListener((o, oldValue, newValue) -> updateHeight(oldValue, newValue));
   }

   private void updateWidth(Number oldWidth, Number newWidth)
   {
      double scale = newWidth.doubleValue() / oldWidth.doubleValue();

      ObservableList<Double> points = getPoints();

      for (int i = 0; i < points.size(); i += 2)
      {
         points.set(i, scale * points.get(i));
      }
   }

   private void updateHeight(Number oldHeight, Number newHeight)
   {
      double scale = newHeight.doubleValue() / oldHeight.doubleValue();

      ObservableList<Double> points = getPoints();

      for (int i = 1; i < points.size(); i += 2)
      {
         points.set(i, scale * points.get(i));
      }
   }

   public void setMarkerWidth(double width)
   {
      markerWidth.set(width);
   }

   public double getMarkerWidth()
   {
      return markerWidth.get();
   }

   public DoubleProperty markWidthProperty()
   {
      return markerWidth;
   }

   public void setMarkerHeight(double height)
   {
      markerHeight.set(height);
   }

   public double getMarkerHeight()
   {
      return markerHeight.get();
   }

   public DoubleProperty markHeightProperty()
   {
      return markerHeight;
   }

   @Override
   public List<CssMetaData<? extends Styleable, ?>> getCssMetaData()
   {
      return FACTORY.getCssMetaData();
   }
}
