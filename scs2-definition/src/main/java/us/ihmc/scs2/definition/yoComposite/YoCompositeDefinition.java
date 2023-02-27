package us.ihmc.scs2.definition.yoComposite;

import java.util.Arrays;

import javax.xml.bind.annotation.XmlSeeAlso;

@XmlSeeAlso({YoOrientation3DDefinition.class, YoTuple2DDefinition.class, YoTuple3DDefinition.class})
public abstract class YoCompositeDefinition
{
   public abstract String getType();

   public abstract String[] getComponentIdentifiers();

   public abstract String[] getComponentValues();

   public abstract String getReferenceFrame();

   @Override
   public final boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoCompositeDefinition)
      {
         YoCompositeDefinition other = (YoCompositeDefinition) object;

         if (getType() == null ? other.getType() != null : !getType().equals(other.getType()))
            return false;

         String[] thisIDs = getComponentIdentifiers();
         String[] otherIDs = other.getComponentIdentifiers();

         if (thisIDs == null ? otherIDs != null : !Arrays.equals(thisIDs, otherIDs))
            return false;

         String[] thisValues = getComponentValues();
         String[] otherValues = other.getComponentValues();

         if (thisValues == null ? otherValues != null : !Arrays.equals(thisValues, otherValues))
            return false;

         if (getReferenceFrame() == null ? other.getReferenceFrame() != null : !getReferenceFrame().equals(other.getReferenceFrame()))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public final String toString()
   {
      String description = "[" + getType();
      String[] ids = getComponentIdentifiers();
      String[] values = getComponentValues();
      for (int i = 0; i < ids.length; i++)
         description += ", " + ids[i] + ": " + values[i];
      description += ", frame: " + getReferenceFrame() + "]";
      return description;
   }
}
