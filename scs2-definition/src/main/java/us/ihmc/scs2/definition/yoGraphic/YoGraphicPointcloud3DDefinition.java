package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;
import java.util.Objects;

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

   public YoGraphicPointcloud3DDefinition()
   {
      registerListField("points", this::getPoints, this::setPoints, "p", Object::toString, YoTuple3DDefinition::parse);
      registerStringField("numberOfPoints", this::getNumberOfPoints, this::setNumberOfPoints);
      registerStringField("size", this::getSize, this::setSize);
      registerStringField("graphicName", this::getGraphicName, this::setGraphicName);
   }

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

   public void setSize(double size)
   {
      setSize(Double.toString(size));
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
      else if (object instanceof YoGraphicPointcloud3DDefinition other)
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

   @Override
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, color=%s, points=%s, numberOfPoints=%s, size=%s, graphicName=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           color,
                           indentedListString(indent, true, points, Object::toString),
                           numberOfPoints,
                           size,
                           graphicName);
   }
}
