package us.ihmc.scs2.sessionVisualizer.jfx.properties;

import javafx.beans.property.Property;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoVariable;

public interface YoVariableProperty<T extends YoVariable, P> extends Property<P>
{
   Property<P> userInputProperty();

   T getYoVariable();

   LinkedYoVariable<T> getLinkedBuffer();

   void finalize();
}
