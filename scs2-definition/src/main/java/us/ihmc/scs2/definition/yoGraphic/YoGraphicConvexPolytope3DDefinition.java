package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.List;
import java.util.Objects;

/**
 * A {@code YoGraphicConvexPolytope3DDefinition} is a template for creating 3D convex polytope and
 * which components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoConvexPolytope3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicConvexPolytope3DDefinition} is to be passed before initialization of a
 * session (either before starting a simulation or when creating a yoVariable server), such that the
 * SCS GUI can use the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicConvexPolytope3D")
public class YoGraphicConvexPolytope3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the polytope. */
   private YoTuple3DDefinition position;
   /** The orientation of the polytope. */
   private YoOrientation3DDefinition orientation;
   /**
    * The vertices of the polytope. No particular ordering is required, {@link ConvexPolytope3D} is
    * used to guarantee the resulting shape is a convex polytope.
    */
   private List<YoTuple3DDefinition> vertices;
   /** The list of vertices for the polytope. */
   private String numberOfVertices;

   /**
    * Creates a new yoGraphic definition for rendering a convex polytope.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicConvexPolytope3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicConvexPolytope3DDefinition(YoGraphicConvexPolytope3DDefinition other)
   {
      super(other);
      position = other.position == null ? null : other.position.copy();
      orientation = other.orientation == null ? null : other.orientation.copy();
      vertices = other.vertices == null ? null : other.vertices.stream().map(YoTuple3DDefinition::copy).toList();
      numberOfVertices = other.numberOfVertices;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerListField("vertices", this::getVertices, this::setVertices, "v", Object::toString, YoTuple3DDefinition::parse);
      registerStringField("numberOfVertices", this::getNumberOfVertices, this::setNumberOfVertices);
   }

   /**
    * Sets the position for the polytope.
    *
    * @param position the position for the polytope.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the orientation for the polytope.
    *
    * @param orientation the orientation for the polytope.
    */
   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   /**
    * Sets the vertices for the polytope. No particular ordering is required, {@link ConvexPolytope3D}
    * is used to guarantee the resulting shape is a convex polytope.
    *
    * @param vertices the vertices for the polytope.
    */
   @XmlElement
   public void setVertices(List<YoTuple3DDefinition> vertices)
   {
      this.vertices = vertices;
   }

   /**
    * Sets the number of vertices to use from the vertices list to build the polytope.
    * <p>
    * Using this method allows to back the number of vertices with a {@code YoVariable} by giving the
    * variable name/fullname.
    * </p>
    *
    * @param numberOfVertices the number of vertices to use.
    */
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
   public YoGraphicConvexPolytope3DDefinition copy()
   {
      return new YoGraphicConvexPolytope3DDefinition(this);
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
      else if (object instanceof YoGraphicConvexPolytope3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
            return false;
         if (!Objects.equals(orientation, other.orientation))
            return false;
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
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, color=%s, position=%s, orientation=%s, vertices=%s, numberOfVertices=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           color,
                           position,
                           orientation,
                           indentedListString(indent, true, vertices, Object::toString),
                           numberOfVertices);
   }
}
