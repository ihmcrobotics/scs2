package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public abstract class Orientation3DProperty extends CompositeProperty implements FrameOrientation3DReadOnly
{
   public Orientation3DProperty(String type, String[] componentIdentifiers)
   {
      super(type, componentIdentifiers);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, double... componentValues)
   {
      super(type, componentIdentifiers, componentValues);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, ReferenceFrame referenceFrame, double... componentValues)
   {
      super(type, componentIdentifiers, referenceFrame, componentValues);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, DoubleProperty... componentValueProperties)
   {
      super(type, componentIdentifiers, componentValueProperties);
   }

   public Orientation3DProperty(String type, String[] componentIdentifiers, Property<ReferenceFrame> referenceFrameProperty,
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
