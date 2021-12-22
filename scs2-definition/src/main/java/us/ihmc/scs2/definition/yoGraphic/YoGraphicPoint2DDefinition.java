package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

@XmlRootElement(name = "YoGraphicPoint2D")
public class YoGraphicPoint2DDefinition extends YoGraphic2DDefinition
{
   private YoTuple2DDefinition position;
   private String size;
   private String graphicName;

   @XmlElement
   public void setPosition(YoTuple2DDefinition position)
   {
      this.position = position;
   }

   public void setSize(double size)
   {
      this.size = Double.toString(size);
   }

   @XmlElement
   public void setSize(String size)
   {
      this.size = size;
   }

   @XmlElement
   public void setGraphicName(String graphicName)
   {
      this.graphicName = graphicName;
   }

   public YoTuple2DDefinition getPosition()
   {
      return position;
   }

   public String getSize()
   {
      return size;
   }

   public String getGraphicName()
   {
      return graphicName;
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
      else if (object instanceof YoGraphicPoint2DDefinition)
      {
         YoGraphicPoint2DDefinition other = (YoGraphicPoint2DDefinition) object;

         if (position == null ? other.position != null : !position.equals(other.position))
            return false;
         if (size == null ? other.size != null : !size.equals(other.size))
            return false;
         if (graphicName == null ? other.graphicName != null : !graphicName.equals(other.graphicName))
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
      return "position: " + position + ", size: " + size + ", fillColor: " + fillColor + ", strokeColor: " + strokeColor + ", strokeWidth: " + strokeWidth
            + ", graphic name: " + graphicName;
   }
}
