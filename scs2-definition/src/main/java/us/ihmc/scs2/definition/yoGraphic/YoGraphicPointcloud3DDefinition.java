package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicPointcloud3D")
public class YoGraphicPointcloud3DDefinition extends YoGraphic3DDefinition
{
   private List<YoTuple3DDefinition> points;
   private String numberOfPoints;
   private String size;
   private String graphicName;

   @XmlElement
   public void setPoints(List<YoTuple3DDefinition> points)
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

   public List<YoTuple3DDefinition> getPoints()
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
      else if (object instanceof YoGraphicPointcloud3DDefinition)
      {
         YoGraphicPointcloud3DDefinition other = (YoGraphicPointcloud3DDefinition) object;

         if (points == null ? other.points != null : !points.equals(other.points))
            return false;
         if (numberOfPoints == null ? other.numberOfPoints != null : !numberOfPoints.equals(other.numberOfPoints))
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
}
