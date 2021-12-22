package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicEllipsoid3D")
public class YoGraphicEllipsoid3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition position;
   private YoOrientation3DDefinition orientation;
   private YoTuple3DDefinition radii;

   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   @XmlElement
   public void setRadii(YoTuple3DDefinition radii)
   {
      this.radii = radii;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public YoTuple3DDefinition getRadii()
   {
      return radii;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (!super.equals(object))
      {
         return false;
      }
      else if (object instanceof YoGraphicEllipsoid3DDefinition)
      {
         YoGraphicEllipsoid3DDefinition other = (YoGraphicEllipsoid3DDefinition) object;

         if (position == null ? other.position != null : !position.equals(other.position))
            return false;
         if (orientation == null ? other.orientation != null : !orientation.equals(other.orientation))
            return false;
         if (radii == null ? other.radii != null : !radii.equals(other.radii))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "position: " + position + ", orientation: " + orientation + ", size: " + radii + ", color: " + color;
   }
}
