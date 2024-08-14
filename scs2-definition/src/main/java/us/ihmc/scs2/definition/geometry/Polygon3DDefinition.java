package us.ihmc.scs2.definition.geometry;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import jakarta.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Definition for creating a 3D polygon.
 * <p>
 * It is assumed that the polygon is convex.
 * </p>
 */
public class Polygon3DDefinition extends GeometryDefinition
{
   private List<Point3DDefinition> polygonVertices;
   private boolean counterClockwiseOrdered;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Polygon3DDefinition()
   {
      setName("polygon3D");
   }

   /**
    * Creates and initializes a definition for a 3D polygon.
    * 
    * @param polygonVertices         the 3D polygon's vertices.
    * @param counterClockwiseOrdered indicate the winding of the polygon: {@code true} if the polygon
    *                                is counter clockwise, {@code false} if clockwise.
    */
   public Polygon3DDefinition(List<Point3DDefinition> polygonVertices, boolean counterClockwiseOrdered)
   {
      this();
      this.polygonVertices = polygonVertices;
      this.counterClockwiseOrdered = counterClockwiseOrdered;
   }

   public Polygon3DDefinition(Polygon3DDefinition other)
   {
      setName(other.getName());
      if (other.polygonVertices != null)
         polygonVertices = other.polygonVertices.stream().map(Point3DDefinition::new).collect(Collectors.toList());
      counterClockwiseOrdered = other.counterClockwiseOrdered;
   }

   public static List<Point3DDefinition> toPoint3DDefinitionList(Collection<? extends Tuple3DReadOnly> tuple3DCollection)
   {
      return tuple3DCollection.stream().map(Point3DDefinition::new).collect(Collectors.toList());
   }

   public static List<Point3DDefinition> toPoint3DDefinitionList(Tuple3DReadOnly... tuple3Ds)
   {
      return toPoint3DDefinitionList(Arrays.asList(tuple3Ds));
   }

   /**
    * Sets the polygon's vertices.
    * 
    * @param polygonVertices the polygon's vertices.
    */
   @XmlElement
   public void setPolygonVertices(List<Point3DDefinition> polygonVertices)
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
   public List<Point3DDefinition> getPolygonVertices()
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
   public Polygon3DDefinition copy()
   {
      return new Polygon3DDefinition(this);
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

      Polygon3DDefinition other = (Polygon3DDefinition) object;

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
      return "Polygon 3D: [name: " + getName() + ", " + ordering + ", vertices: "
            + EuclidCoreIOTools.getCollectionString("[", "]", ", ", polygonVertices, Object::toString);
   }
}
