package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import javafx.beans.property.Property;
import javafx.beans.value.ChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.NumberSeries;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ChartTools;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.concurrent.atomic.AtomicBoolean;

public class YoNumberSeries extends NumberSeries
{
   private final YoVariable yoVariable;
   private final AtomicBoolean dirty = new AtomicBoolean(true);
   private final Property<Integer> precision;
   private final YoVariableChangedListener dirtyYoListener = v -> dirty.set(true);
   private final ChangeListener<Number> dirtyPropertyListener = (o, oldValue, newValue) -> dirty.set(true);

   public YoNumberSeries(YoVariable yoVariable, Property<Integer> precision)
   {
      super(yoVariable.getName());

      this.yoVariable = yoVariable;
      this.precision = precision;

      yoVariable.addListener(dirtyYoListener);
      precision.addListener(dirtyPropertyListener);
   }

   public void updateLegend()
   {
      if (dirty.getAndSet(false))
         setCurrentValue(ChartTools.defaultYoVariableValueFormatter(yoVariable, Math.max(5, precision.getValue())));
   }

   public YoVariable getYoVariable()
   {
      return yoVariable;
   }

   public void close()
   {
      yoVariable.removeListener(dirtyYoListener);
      precision.removeListener(dirtyPropertyListener);
   }
}