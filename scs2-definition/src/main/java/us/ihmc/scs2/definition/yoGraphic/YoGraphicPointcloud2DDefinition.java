package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;
import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

@XmlRootElement(name = "YoGraphicPointcloud3D")
public class YoGraphicPointcloud2DDefinition extends YoGraphic2DDefinition
{
   private List<YoTuple2DDefinition> points;
   private String numberOfPoints;
   private String size;
   private String graphicName;

   public YoGraphicPointcloud2DDefinition()
   {
      registerListField("points", this::getPoints, this::setPoints);
      registerField("numberOfPoints", this::getNumberOfPoints, this::setNumberOfPoints);
      registerField("size", this::getSize, this::setSize);
      registerField("graphicName", this::getGraphicName, this::setGraphicName);
   }

   @XmlElement
   public void setPoints(List<YoTuple2DDefinition> points)
   {
      this.points = points;
   }

   @XmlElement
   public void setNumberOfPoints(String numberOfPoints)
   {
      this.numberOfPoints = numberOfPoints;
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

   public List<YoTuple2DDefinition> getPoints()
   {
      return points;
   }

   public String getNumberOfPoints()
   {
      return numberOfPoints;
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
      else if (object instanceof YoGraphicPointcloud2DDefinition other)
      {
         if (!Objects.equals(points, other.points))
            return false;
         if (!Objects.equals(numberOfPoints, other.numberOfPoints))
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
