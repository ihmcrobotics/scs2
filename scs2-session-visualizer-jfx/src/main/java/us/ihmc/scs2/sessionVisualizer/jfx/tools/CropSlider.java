package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import com.jfoenix.controls.JFXSlider;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.DoublePropertyBase;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.AccessibleAttribute;
import javafx.scene.control.Skin;
import us.ihmc.commons.MathTools;

public class CropSlider extends JFXSlider
{
   private static final String DEFAULT_STYLE_CLASS = "crop-slider";

   public CropSlider()
   {
      super();
      initialize();
   }

   public CropSlider(double min, double max, double value)
   {
      super(min, max, value);
      initialize();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected Skin<?> createDefaultSkin()
   {
      return new CropSliderSkin(this);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String getUserAgentStylesheet()
   {
      return getClass().getClassLoader().getResource("css/crop-slider.css").toExternalForm();
   }

   private void initialize()
   {
      getStyleClass().add(DEFAULT_STYLE_CLASS);
   }

   private BooleanProperty showTrim;

   public boolean isShowTrim()
   {
      return showTrim == null ? false : showTrim.get();
   }

   public void setShowTrim(boolean value)
   {
      showTrimProperty().set(value);
   }

   public BooleanProperty showTrimProperty()
   {
      if (showTrim == null)
         showTrim = new SimpleBooleanProperty(this, "showTrim", false);

      return showTrim;
   }

   private DoubleProperty trimGap;

   public double getTrimGap()
   {
      return trimGap == null ? 3.0 : trimGap.get();
   }

   public void setTrimGap(double value)
   {
      trimGapProperty().set(value);
   }

   public DoubleProperty trimGapProperty()
   {
      if (trimGap == null)
         trimGap = new SimpleDoubleProperty(this, "trimGap", 3.0);

      return trimGap;
   }

   private DoubleProperty trimStartValue;

   public final void setTrimStartValue(double value)
   {
      if (!trimStartValueProperty().isBound())
         trimStartValueProperty().set(value);
   }

   public final double getTrimStartValue()
   {
      return trimStartValue == null ? 0 : trimStartValue.get();
   }

   public final DoubleProperty trimStartValueProperty()
   {
      if (trimStartValue == null)
      {
         trimStartValue = new DoublePropertyBase(0)
         {
            @Override
            protected void invalidated()
            {
               if (getTrimStartValue() < getMin() || getTrimStartValue() > getTrimEndValue())
                  setTrimStartValue(MathTools.clamp(getTrimStartValue(), getMin(), getTrimEndValue()));

               notifyAccessibleAttributeChanged(AccessibleAttribute.VALUE);
            }

            @Override
            public Object getBean()
            {
               return CropSlider.this;
            }

            @Override
            public String getName()
            {
               return "trimStartValue";
            }
         };
      }
      return trimStartValue;
   }

   private DoubleProperty trimEndValue;

   public final void setTrimEndValue(double value)
   {
      if (!trimEndValueProperty().isBound())
         trimEndValueProperty().set(value);
   }

   public final double getTrimEndValue()
   {
      return trimEndValue == null ? 0 : trimEndValue.get();
   }

   public final DoubleProperty trimEndValueProperty()
   {
      if (trimEndValue == null)
      {
         trimEndValue = new DoublePropertyBase(getMax())
         {
            @Override
            protected void invalidated()
            {
               if (getTrimEndValue() < getTrimStartValue() || getTrimEndValue() > getMax())
                  setTrimEndValue(MathTools.clamp(getTrimEndValue(), getTrimStartValue(), getMax()));
               notifyAccessibleAttributeChanged(AccessibleAttribute.VALUE);
            }

            @Override
            public Object getBean()
            {
               return CropSlider.this;
            }

            @Override
            public String getName()
            {
               return "trimEndValue";
            }
         };
      }
      return trimEndValue;
   }

   /**
    * When true, indicates the current trimStartValue of this Slider is changing. It provides
    * notification that the trimStartValue is changing. Once the trimStartValue is computed, it is
    * reset back to false.
    */
   private BooleanProperty trimStartValueChanging;

   public final void setTrimStartValueChanging(boolean trimStartValue)
   {
      trimStartValueChangingProperty().set(trimStartValue);
   }

   public final boolean isTrimStartValueChanging()
   {
      return trimStartValueChanging == null ? false : trimStartValueChanging.get();
   }

   public final BooleanProperty trimStartValueChangingProperty()
   {
      if (trimStartValueChanging == null)
      {
         trimStartValueChanging = new SimpleBooleanProperty(this, "trimStartValueChanging", false);
      }
      return trimStartValueChanging;
   }

   /**
    * When true, indicates the current trimEndValue of this Slider is changing. It provides
    * notification that the trimEndValue is changing. Once the trimEndValue is computed, it is reset
    * back to false.
    */
   private BooleanProperty trimEndValueChanging;

   public final void setTrimEndValueChanging(boolean trimEndValue)
   {
      trimEndValueChangingProperty().set(trimEndValue);
   }

   public final boolean isTrimEndValueChanging()
   {
      return trimEndValueChanging == null ? false : trimEndValueChanging.get();
   }

   public final BooleanProperty trimEndValueChangingProperty()
   {
      if (trimEndValueChanging == null)
      {
         trimEndValueChanging = new SimpleBooleanProperty(this, "trimEndValueChanging", false);
      }
      return trimEndValueChanging;
   }

   /**
    * When true, the current value of this slider will be automatically bound to the value of the trim
    * that is being dragged.
    */
   private BooleanProperty bindValueToActiveTrim;

   public final void setBindValueToActiveTrim(boolean bind)
   {
      bindValueToActiveTrimProperty().set(bind);
   }

   public final boolean isValueBoundToActiveTrim()
   {
      return bindValueToActiveTrim == null ? false : bindValueToActiveTrim.get();
   }

   public final BooleanProperty bindValueToActiveTrimProperty()
   {
      if (bindValueToActiveTrim == null)
      {
         bindValueToActiveTrim = new SimpleBooleanProperty(this, "bindValueToActiveTrim", false);
      }
      return bindValueToActiveTrim;
   }

   public void adjustTrimStartValue(double newValue)
   {
      adjustValue(newValue, trimStartValue);
   }

   public void adjustTrimEndValue(double newValue)
   {
      adjustValue(newValue, trimEndValue);
   }

   public void adjustValue(double newValue, DoubleProperty valueToAdjust)
   {
      // figure out the "value" associated with the specified position
      final double _min = getMin();
      final double _max = getMax();
      if (_max <= _min)
         return;
      newValue = newValue < _min ? _min : newValue;
      newValue = newValue > _max ? _max : newValue;

      valueToAdjust.set(snapValueToTicks(newValue));
   }

   /**
    * Utility function which, given the specified value, will position it either aligned with a tick,
    * or simply clamp between min & max value, depending on whether snapToTicks is set.
    *
    * @expert This function is intended to be used by experts, primarily by those implementing new
    *         Skins or Behaviors. It is not common for developers or designers to access this function
    *         directly.
    */
   private double snapValueToTicks(double val)
   {
      double v = val;
      if (isSnapToTicks())
      {
         double tickSpacing = 0;
         // compute the nearest tick to this value
         if (getMinorTickCount() != 0)
         {
            tickSpacing = getMajorTickUnit() / (Math.max(getMinorTickCount(), 0) + 1);
         }
         else
         {
            tickSpacing = getMajorTickUnit();
         }
         int prevTick = (int) ((v - getMin()) / tickSpacing);
         double prevTickValue = (prevTick) * tickSpacing + getMin();
         double nextTickValue = (prevTick + 1) * tickSpacing + getMin();
         v = nearest(prevTickValue, v, nextTickValue);
      }
      return MathTools.clamp(v, getMin(), getMax());
   }

   public static double nearest(double less, double value, double more)
   {
      double lessDiff = value - less;
      double moreDiff = more - value;
      if (lessDiff < moreDiff)
         return less;
      return more;
   }
}
