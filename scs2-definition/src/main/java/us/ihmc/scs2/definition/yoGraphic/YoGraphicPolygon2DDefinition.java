package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;
import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

@XmlRootElement(name = "YoGraphicPolygon2D")
public class YoGraphicPolygon2DDefinition extends YoGraphic2DDefinition
{
   private List<YoTuple2DDefinition> vertices;
   private String numberOfVertices;

   public YoGraphicPolygon2DDefinition()
   {
      registerListField("vertices", this::getVertices, this::setVertices);
      registerField("numberOfVertices", this::getNumberOfVertices, this::setNumberOfVertices);
   }

   @XmlElement
   public void setVertices(List<YoTuple2DDefinition> vertices)
   {
      this.vertices = vertices;
   }

   public void setNumberOfVertices(int numberOfVertices)
   {
      this.numberOfVertices = Integer.toString(numberOfVertices);
   }

   @XmlElement
   public void setNumberOfVertices(String numberOfVertices)
   {
      this.numberOfVertices = numberOfVertices;
   }

   public List<YoTuple2DDefinition> getVertices()
   {
      return vertices;
   }

   public String getNumberOfVertices()
   {
      return numberOfVertices;
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
      else if (object instanceof YoGraphicPolygon2DDefinition other)
      {
         if (!Objects.equals(vertices, other.vertices))
            return false;
         if (!Objects.equals(numberOfVertices, other.numberOfVertices))
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
      return "number of vertices: " + numberOfVertices + ", fillColor: " + fillColor + ", strokeColor: " + strokeColor + ", strokeWidth: " + strokeWidth
            + ", vertices: " + EuclidCoreIOTools.getCollectionString("\n", vertices, YoTuple2DDefinition::toString);
   }
}
