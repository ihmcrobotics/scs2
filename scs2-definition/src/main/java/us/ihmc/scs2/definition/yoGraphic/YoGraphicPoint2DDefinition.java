package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

@XmlRootElement(name = "YoGraphicPoint2D")
public class YoGraphicPoint2DDefinition extends YoGraphic2DDefinition
{
   private YoTuple2DDefinition position;
   private String size;
   private String graphicName;

   public YoGraphicPoint2DDefinition()
   {
      registerTuple2DField("position", this::getPosition, this::setPosition);
      registerField("size", this::getSize, this::setSize);
      registerField("graphicName", this::getGraphicName, this::setGraphicName);
   }

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
      else if (object instanceof YoGraphicPoint2DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(size, other.size))
            return false;
         if (!Objects.equals(graphicName, other.graphicName))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
