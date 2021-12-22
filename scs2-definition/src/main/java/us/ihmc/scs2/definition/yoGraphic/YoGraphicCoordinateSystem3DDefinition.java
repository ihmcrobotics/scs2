package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicCoordinateSystem3D")
public class YoGraphicCoordinateSystem3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition position;
   private YoOrientation3DDefinition orientation;
   private String bodyLength;
   private String headLength;
   private String bodyRadius;
   private String headRadius;

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

   public void setBodyLength(double bodyLength)
   {
      this.bodyLength = Double.toString(bodyLength);
   }

   @XmlElement
   public void setBodyLength(String bodyLength)
   {
      this.bodyLength = bodyLength;
   }

   public void setBodyRadius(double bodyRadius)
   {
      this.bodyRadius = Double.toString(bodyRadius);
   }

   @XmlElement
   public void setBodyRadius(String bodyRadius)
   {
      this.bodyRadius = bodyRadius;
   }

   public void setHeadLength(double headLength)
   {
      this.headLength = Double.toString(headLength);
   }

   @XmlElement
   public void setHeadLength(String headLength)
   {
      this.headLength = headLength;
   }

   public void setHeadRadius(double headRadius)
   {
      this.headRadius = Double.toString(headRadius);
   }

   @XmlElement
   public void setHeadRadius(String headRadius)
   {
      this.headRadius = headRadius;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public String getBodyLength()
   {
      return bodyLength;
   }

   public String getHeadLength()
   {
      return headLength;
   }

   public String getBodyRadius()
   {
      return bodyRadius;
   }

   public String getHeadRadius()
   {
      return headRadius;
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
      else if (object instanceof YoGraphicCoordinateSystem3DDefinition)
      {
         YoGraphicCoordinateSystem3DDefinition other = (YoGraphicCoordinateSystem3DDefinition) object;

         if (position == null ? other.position != null : !position.equals(other.position))
            return false;
         if (orientation == null ? other.orientation != null : !orientation.equals(other.orientation))
            return false;
         if (bodyLength == null ? other.bodyLength != null : !bodyLength.equals(other.bodyLength))
            return false;
         if (headLength == null ? other.headLength != null : !headLength.equals(other.headLength))
            return false;
         if (bodyRadius == null ? other.bodyRadius != null : !bodyRadius.equals(other.bodyRadius))
            return false;
         if (headRadius == null ? other.headRadius != null : !headRadius.equals(other.headRadius))
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
      return super.toString() + ", position: " + position + ", orientation: " + orientation + ", bodyLength: " + bodyLength + ", headLength: " + headLength
            + ", bodyRadius: " + bodyRadius + ", headRadius: " + headRadius + ", color: " + color;
   }
}
