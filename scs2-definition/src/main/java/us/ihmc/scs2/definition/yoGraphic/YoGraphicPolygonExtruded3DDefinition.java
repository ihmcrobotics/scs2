package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.List;
import java.util.Objects;

/**
 * A {@code YoGraphicPolygonExtruded3DDefinition} is a template for creating 2D polygon that is
 * extruded into a 3D shape and which components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolygonExtrudedFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicPolygonExtruded3DDefinition} is to be passed before initialization of a
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
@XmlRootElement(name = "YoGraphicPolygonExtruded3D")
public class YoGraphicPolygonExtruded3DDefinition extends YoGraphic3DDefinition
{
   /** The position of the polygon base. */
   private YoTuple3DDefinition position;
   /** The orientation of the polygon. */
   private YoOrientation3DDefinition orientation;
   /** The list of vertices for the polygon. */
   private List<YoTuple2DDefinition> vertices;
   /** The number of vertices to use in the list {@link #vertices} or {@code null} to use all. */
   private String numberOfVertices;
   /** The thickness of the extrusion. */
   private String thickness;

   /**
    * Creates a new yoGraphic definition for rendering an extruded polygon.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicPolygonExtruded3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPolygonExtruded3DDefinition(YoGraphicPolygonExtruded3DDefinition other)
   {
      super(other);
      position = other.position == null ? null : other.position.copy();
      orientation = other.orientation == null ? null : other.orientation.copy();
      vertices = other.vertices == null ? null : other.vertices.stream().map(YoTuple2DDefinition::copy).toList();
      numberOfVertices = other.numberOfVertices;
      thickness = other.thickness;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerOrientation3DField("orientation", this::getOrientation, this::setOrientation);
      registerListField("vertices", this::getVertices, this::setVertices, "v", Object::toString, YoTuple2DDefinition::parse);
      registerStringField("numberOfVertices", this::getNumberOfVertices, this::setNumberOfVertices);
      registerStringField("thickness", this::getThickness, this::setThickness);
   }

   /**
    * Sets the position for the base of the extruded polygon.
    *
    * @param position the position for the extruded polygon.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the orientation for the extruded polygon.
    *
    * @param orientation the orientation for the extruded polygon.
    */
   @XmlElement
   public void setOrientation(YoOrientation3DDefinition orientation)
   {
      this.orientation = orientation;
   }

   /**
    * Sets the vertices for the extruded polygon.
    *
    * @param vertices the vertices for the extruded polygon.
    */
   @XmlElement
   public void setVertices(List<YoTuple2DDefinition> vertices)
   {
      this.vertices = vertices;
   }

   /**
    * Sets the number of vertices to use from the vertices list to build the extruded polygon.
    *
    * @param numberOfVertices the number of vertices to use.
    */
   public void setNumberOfVertices(int numberOfVertices)
   {
      this.numberOfVertices = Integer.toString(numberOfVertices);
   }

   /**
    * Sets the number of vertices to use from the vertices list to build the extruded polygon.
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

   /**
    * Sets the extrusion thickness to a constant value.
    *
    * @param thickness the extrusion thickness.
    */
   public void setThickness(double thickness)
   {
      this.thickness = Double.toString(thickness);
   }

   /**
    * Sets the extrusion thickness. Can be backed by a {@code YoVarable} by giving the variable's
    * name/fullname.
    *
    * @param thickness the extrusion thickness.
    */
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
   public YoGraphicPolygonExtruded3DDefinition copy()
   {
      return new YoGraphicPolygonExtruded3DDefinition(this);
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
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, color=%s, position=%s, orientation=%s, vertices=%s, numberOfVertices=%s, thickness=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           color,
                           position,
                           orientation,
                           indentedListString(indent, true, vertices, Object::toString),
                           numberOfVertices,
                           thickness);
   }
}
