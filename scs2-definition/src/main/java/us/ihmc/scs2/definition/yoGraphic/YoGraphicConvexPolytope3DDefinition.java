package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

@XmlRootElement(name = "YoGraphicConvexPolytope3D")
public class YoGraphicConvexPolytope3DDefinition extends YoGraphic3DDefinition
{
   private YoTuple3DDefinition position;
   private YoOrientation3DDefinition orientation;
   private List<YoTuple3DDefinition> vertices;
   private String numberOfVertices;

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
   public void setVertices(List<YoTuple3DDefinition> vertices)
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

   public YoTuple3DDefinition getPosition()
   {
      return position;
   }

   public YoOrientation3DDefinition getOrientation()
   {
      return orientation;
   }

   public List<YoTuple3DDefinition> getVertices()
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
      else if (object instanceof YoGraphicConvexPolytope3DDefinition)
      {
         YoGraphicConvexPolytope3DDefinition other = (YoGraphicConvexPolytope3DDefinition) object;

         if (position == null ? other.position != null : !position.equals(other.position))
            return false;
         if (orientation == null ? other.orientation != null : !orientation.equals(other.orientation))
            return false;
         if (vertices == null ? other.vertices != null : !vertices.equals(other.vertices))
            return false;
         if (numberOfVertices == null ? other.numberOfVertices != null : !numberOfVertices.equals(other.numberOfVertices))
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
            + ", vertices: " + EuclidCoreIOTools.getCollectionString("\n", vertices, YoTuple3DDefinition::toString);
   }
}
