package us.ihmc.scs2.definition.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Definition for creating a 2D polygon.
 * <p>
 * It is assumed that the polygon is convex.
 * </p>
 */
public class Polygon2DDefinition extends GeometryDefinition
{
   private List<Point2DDefinition> polygonVertices = new ArrayList<>();
   private boolean counterClockwiseOrdered;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Polygon2DDefinition()
   {
      setName("polygon2D");
   }

   /**
    * Creates and initializes a definition for a 2D polygon.
    * 
    * @param polygonVertices         the 2D polygon's vertices.
    * @param counterClockwiseOrdered indicate the winding of the polygon: {@code true} if the polygon
    *                                is counter clockwise, {@code false} if clockwise.
    */
   public Polygon2DDefinition(List<Point2DDefinition> polygonVertices, boolean counterClockwiseOrdered)
   {
      this();
      this.polygonVertices = polygonVertices;
      this.counterClockwiseOrdered = counterClockwiseOrdered;
   }

   public Polygon2DDefinition(Polygon2DDefinition other)
   {
      setName(other.getName());
      if (other.polygonVertices != null)
         polygonVertices = other.polygonVertices.stream().map(Point2DDefinition::new).collect(Collectors.toList());
      counterClockwiseOrdered = other.counterClockwiseOrdered;
   }

   public static List<Point2DDefinition> toPoint2DDefinitionList(Collection<? extends Tuple2DReadOnly> tuple2DCollection)
   {
      return tuple2DCollection.stream().map(Point2DDefinition::new).collect(Collectors.toList());
   }

   public static List<Point2DDefinition> toPoint3DDefinitionList(Tuple2DReadOnly... tuple2Ds)
   {
      return toPoint2DDefinitionList(Arrays.asList(tuple2Ds));
   }

   /**
    * Sets the polygon's vertices.
    * 
    * @param polygonVertices the polygon's vertices.
    */
   @XmlElement
   public void setPolygonVertices(List<Point2DDefinition> polygonVertices)
   {
      this.polygonVertices = polygonVertices;
   }

   /**
    * Sets the flag for indicating the winding of the polygon.
    * 
    * @param counterClockwiseOrdered indicate the winding of the polygon: {@code true} if the polygon
    *                                is counter clockwise, {@code false} if clockwise.
    */
   @XmlElement
   public void setCounterClockwiseOrdered(boolean counterClockwiseOrdered)
   {
      this.counterClockwiseOrdered = counterClockwiseOrdered;
   }

   /**
    * Returns the polygon's vertices.
    * 
    * @return the polygon's vertices.
    */
   public List<Point2DDefinition> getPolygonVertices()
   {
      return polygonVertices;
   }

   /**
    * Returns the winding of the polygon.
    * 
    * @return {@code true} if the polygon is counter clockwise, {@code false} if clockwise.
    */
   public boolean isCounterClockwiseOrdered()
   {
      return counterClockwiseOrdered;
   }

   @Override
   public Polygon2DDefinition copy()
   {
      return new Polygon2DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, polygonVertices);
      bits = EuclidHashCodeTools.addToHashCode(bits, counterClockwiseOrdered);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      Polygon2DDefinition other = (Polygon2DDefinition) object;

      if (!Objects.equals(polygonVertices, other.polygonVertices))
         return false;
      if (counterClockwiseOrdered != other.counterClockwiseOrdered)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      String ordering = counterClockwiseOrdered ? "counter-clockwise" : "clockwise";
      return "Polygon 2D: [name: " + getName() + ", " + ordering + ", vertices: "
            + EuclidCoreIOTools.getCollectionString("[", "]", ", ", polygonVertices, Object::toString);
   }
}
