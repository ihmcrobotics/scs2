package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group;

import java.util.Collection;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import javafx.beans.property.DoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.CompositeProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

public class YoGroupFXEditorTools
{

   public static <T, F> void setField(Collection<T> collection, BiConsumer<T, F> fieldSetter, F fieldValue)
   {
      for (T item : collection)
      {
         fieldSetter.accept(item, fieldValue);
      }
   }

   public static <T, F> Collection<F> getField(Collection<T> collection, Function<T, F> getter)
   {
      return collection.stream().map(getter).collect(Collectors.toList());
   }

   public static String getCommonString(YoCompositeSearchManager yoCompositeSearchManager, Collection<DoubleProperty> doubleProperties)
   {
      if (doubleProperties == null || doubleProperties.isEmpty())
         return null;

      String commonString = null;

      for (DoubleProperty doubleProperty : doubleProperties)
      {
         String asString = toString(yoCompositeSearchManager, doubleProperty);

         if (commonString == null)
            commonString = asString;
         else if (!asString.equals(commonString))
            return null;
      }

      return commonString;
   }

   public static DoubleProperty getCommonDoubleProperty(YoCompositeSearchManager yoCompositeSearchManager, Collection<DoubleProperty> doubleProperties)
   {
      if (doubleProperties == null || doubleProperties.isEmpty())
         return null;

      DoubleProperty commonDoubleProperty = null;
      YoDoubleProperty commonYoDoubleProperty = null;

      for (DoubleProperty doubleProperty : doubleProperties)
      {
         if (commonDoubleProperty == null)
         {
            commonDoubleProperty = doubleProperty;
            if (commonDoubleProperty instanceof YoDoubleProperty)
               commonYoDoubleProperty = (YoDoubleProperty) commonDoubleProperty;
         }
         else
         {
            if (doubleProperty instanceof YoDoubleProperty)
            {
               if (commonYoDoubleProperty == null)
                  return null; // The common one is a YoDoubleProperty

               YoDoubleProperty yoDoubleProperty = (YoDoubleProperty) doubleProperty;

               if (!yoDoubleProperty.getYoVariable().getFullNameString().equals(commonYoDoubleProperty.getYoVariable().getFullNameString()))
                  return null;
            }
            else // doubleProperty is not a YoDoubleProperty
            {
               if (commonYoDoubleProperty != null)
                  return null; // The common one is a YoDoubleProperty
               else if (doubleProperty.get() != commonDoubleProperty.get())
                  return null;
            }
         }
      }

      return commonDoubleProperty;
   }

   public static String toString(YoCompositeSearchManager yoCompositeSearchManager, DoubleProperty doubleProperty)
   {
      if (doubleProperty instanceof YoDoubleProperty)
      {
         YoDoubleProperty yoDoubleProperty = (YoDoubleProperty) doubleProperty;
         return yoCompositeSearchManager.getYoDoubleCollection().getYoVariableUniqueName(yoDoubleProperty.getYoVariable());
      }
      else
      {
         return Double.toString(doubleProperty.get());
      }
   }

   public static Tuple3DProperty getCommonTuple3DProperty(YoCompositeSearchManager yoCompositeSearchManager, Collection<Tuple3DProperty> tuple3DProperties)
   {
      DoubleProperty commonX = getCommonDoubleProperty(yoCompositeSearchManager, getField(tuple3DProperties, Tuple3DProperty::xProperty));
      DoubleProperty commonY = getCommonDoubleProperty(yoCompositeSearchManager, getField(tuple3DProperties, Tuple3DProperty::yProperty));
      DoubleProperty commonZ = getCommonDoubleProperty(yoCompositeSearchManager, getField(tuple3DProperties, Tuple3DProperty::zProperty));

      if (commonX != null || commonY != null || commonZ != null)
         return new Tuple3DProperty(commonX, commonY, commonZ);
      else
         return null;
   }

   public static CompositeProperty getCommonCompositeProperty(YoCompositeSearchManager yoCompositeSearchManager,
                                                              Collection<? extends CompositeProperty> compositeProperties)
   {
      if (compositeProperties == null || compositeProperties.isEmpty())
         return null;

      CompositeProperty firstCompositeProperty = compositeProperties.iterator().next();

      String type = firstCompositeProperty.getType();
      String[] componentIdentifiers = firstCompositeProperty.getComponentIdentifiers();
      int numberOfElements = componentIdentifiers.length;
      DoubleProperty[] commonElements = new DoubleProperty[numberOfElements];
      boolean atLeastCommonValue = false;

      for (int i = 0; i < numberOfElements; i++)
      {
         int index = i;
         commonElements[i] = getCommonDoubleProperty(yoCompositeSearchManager, getField(compositeProperties, c -> c.componentValueProperties()[index]));
         atLeastCommonValue |= commonElements[i] != null;
      }

      if (atLeastCommonValue)
         return new CompositeProperty(type, componentIdentifiers, commonElements);
      else
         return null;
   }

   public static <T> T getCommonValue(Collection<T> values)
   {
      if (values == null || values.isEmpty())
         return null;

      T commonValue = null;

      for (T value : values)
      {
         if (commonValue == null)
            commonValue = value;
         else if (!commonValue.equals(value))
            return null;
      }

      return commonValue;
   }
}
