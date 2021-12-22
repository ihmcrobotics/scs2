package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import us.ihmc.euclid.tuple2D.Point2D;

public class NumberSeries
{
   /** The user displayable name for this series */
   private final StringProperty seriesNameProperty = new SimpleStringProperty(this, "seriesName", null);
   private final StringProperty currentValueProperty = new SimpleStringProperty(this, "currentValue", null);

   private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

   private final Property<ChartIntegerBounds> xBoundsProperty = new SimpleObjectProperty<>(this, "xBounds", null);
   private final Property<ChartDoubleBounds> yBoundsProperty = new SimpleObjectProperty<>(this, "yBounds", null);
   private final List<Point2D> data = new ArrayList<>();
   private final IntegerProperty bufferCurrentIndexProperty = new SimpleIntegerProperty(this, "bufferCurrentIndex", -1);

   private final BooleanProperty dirtyProperty = new SimpleBooleanProperty(this, "dirtyFlag", false);

   // User properties
   private final BooleanProperty negatedProperty = new SimpleBooleanProperty(this, "negated", false);
   private final ObjectProperty<ChartDoubleBounds> customYBoundsProperty = new SimpleObjectProperty<>(this, "customYBounds", null);

   public NumberSeries(String name)
   {
      setSeriesName(name);
      negatedProperty.addListener((InvalidationListener) -> markDirty());
      customYBoundsProperty.addListener((InvalidationListener) -> markDirty());
   }

   public final String getSeriesName()
   {
      return seriesNameProperty.get();
   }

   public final void setSeriesName(String value)
   {
      seriesNameProperty.set(value);
   }

   public final StringProperty seriesNameProperty()
   {
      return seriesNameProperty;
   }

   public final void setCurrentValue(String value)
   {
      currentValueProperty.set(value);
   }

   public final String getCurrentValue()
   {
      return currentValueProperty.get();
   }

   public final StringProperty currentValueProperty()
   {
      return currentValueProperty;
   }

   public ReentrantReadWriteLock getLock()
   {
      return lock;
   }

   public Property<ChartIntegerBounds> xBoundsProperty()
   {
      return xBoundsProperty;
   }

   public Property<ChartDoubleBounds> yBoundsProperty()
   {
      return yBoundsProperty;
   }

   public List<Point2D> getData()
   {
      return data;
   }

   public IntegerProperty bufferCurrentIndexProperty()
   {
      return bufferCurrentIndexProperty;
   }

   public void markDirty()
   {
      dirtyProperty.set(true);
   }

   public boolean peekDirty()
   {
      return dirtyProperty.get();
   }

   public boolean pollDirty()
   {
      boolean result = dirtyProperty.get();
      dirtyProperty.set(false);
      return result;
   }

   public BooleanProperty dirtyProperty()
   {
      return dirtyProperty;
   }

   public final boolean isNegated()
   {
      return negatedProperty.get();
   }

   public final void setNegated(boolean value)
   {
      negatedProperty.set(value);
   }

   public final BooleanProperty negatedProperty()
   {
      return negatedProperty;
   }

   public final ChartDoubleBounds getCustomYBounds()
   {
      return customYBoundsProperty.get();
   }

   public final void setCustomYBounds(ChartDoubleBounds customYBounds)
   {
      customYBoundsProperty.set(customYBounds);
   }

   public ObjectProperty<ChartDoubleBounds> customYBoundsProperty()
   {
      return customYBoundsProperty;
   }
}