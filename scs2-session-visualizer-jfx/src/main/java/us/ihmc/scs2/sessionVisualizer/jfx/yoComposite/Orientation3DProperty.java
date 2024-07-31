package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;

public abstract class Orientation3DProperty extends CompositeProperty implements Orientation3DReadOnly
{
   public Orientation3DProperty(String type, String[] componentIdentifiers)
   {
      super(type, componentIdentifiers);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, double... componentValues)
   {
      super(type, componentIdentifiers, componentValues);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, ReferenceFrameWrapper referenceFrame, double... componentValues)
   {
      super(type, componentIdentifiers, referenceFrame, componentValues);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, DoubleProperty... componentValueProperties)
   {
      super(type, componentIdentifiers, componentValueProperties);
   }

   public Orientation3DProperty(String type,
                                String[] componentIdentifiers,
                                Property<ReferenceFrameWrapper> referenceFrameProperty,
                                DoubleProperty... componentValueProperties)
   {
      super(type, componentIdentifiers, referenceFrameProperty, componentValueProperties);
   }

   @Override
   public abstract Orientation3DProperty clone();

   public Quaternion toQuaternionInWorld()
   {
      return toWorld(new Quaternion(this));
   }
}
