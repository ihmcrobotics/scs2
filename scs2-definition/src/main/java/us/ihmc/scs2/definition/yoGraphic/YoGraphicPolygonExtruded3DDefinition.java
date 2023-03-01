package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;
import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicPolygonExtruded3D")
public class YoGraphicPolygonExtruded3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition position;
   private YoOrientation3DDefinition orientation;
   private List<YoTuple2DDefinition> vertices;
   private String numberOfVertices;
   private String thickness;

   public YoGraphicPolygonExtruded3DDefinition()
   {
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerListField("vertices", this::getVertices, this::setVertices);
      registerField("numberOfVectices", this::getNumberOfVertices, this::setNumberOfVertices);
      registerField("thickness", this::getThickness, this::setThickness);
   }

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

   public void setThickness(double thickness)
   {
      this.thickness = Double.toString(thickness);
   }

   @XmlElement
   public void setThickness(String thickness)
   {
      this.thickness = thickness;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public List<YoTuple2DDefinition> getVertices()
   {
      return vertices;
   }

   public String getNumberOfVertices()
   {
      return numberOfVertices;
   }

   public String getThickness()
   {
      return thickness;
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
      else if (object instanceof YoGraphicPolygonExtruded3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(orientation, other.orientation))
            return false;
         if (!Objects.equals(vertices, other.vertices))
            return false;
         if (!Objects.equals(numberOfVertices, other.numberOfVertices))
            return false;
         if (!Objects.equals(thickness, other.thickness))
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
      return "position: " + position + ", orientation: " + orientation + ", number of vertices: " + numberOfVertices + ", color: " + color + ", thickness: "
            + thickness + ", vertices: " + EuclidCoreIOTools.getCollectionString("\n", vertices, YoTuple2DDefinition::toString);
   }
}
