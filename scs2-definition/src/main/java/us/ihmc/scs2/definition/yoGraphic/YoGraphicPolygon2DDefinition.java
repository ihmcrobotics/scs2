package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.List;
import java.util.Objects;

/**
 * A {@code YoGraphicPolygon2DDefinition} is a template for creating 2D polygon and which components
 * can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolygonFX2D.png"
 * width=150px/>
 * <p>
 * A 2D yoGraphic is rendered in the overhead plotter panel and it can be back by
 * {@code YoVariable}s allowing it to move and change at runtime.
 * </p>
 * <p>
 * The {@code YoGraphicLine2DDefinition} is to be passed before initialization of a session (either
 * before starting a simulation or when creating a yoVariable server), such that the SCS GUI can use
 * the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicPolygon2D")
public class YoGraphicPolygon2DDefinition extends YoGraphic2DDefinition
{
   /** The list of vertices for the polygon. */
   private List<YoTuple2DDefinition> vertices;
   /** The number of vertices to use in the list {@link #vertices} or {@code null} to use all. */
   private String numberOfVertices;

   /**
    * Creates a new yoGraphic definition for rendering a line.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicPolygon2DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPolygon2DDefinition(YoGraphicPolygon2DDefinition other)
   {
      super(other);
      vertices = other.vertices == null ? null : other.vertices.stream().map(YoTuple2DDefinition::copy).toList();
      numberOfVertices = other.numberOfVertices;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerListField("vertices", this::getVertices, this::setVertices, "v", Object::toString, YoTuple2DDefinition::parse);
      registerStringField("numberOfVertices", this::getNumberOfVertices, this::setNumberOfVertices);
   }

   /**
    * Sets the vertices for the polygon.
    *
    * @param vertices the vertices for the polygon.
    */
   @XmlElement
   public void setVertices(List<YoTuple2DDefinition> vertices)
   {
      this.vertices = vertices;
   }

   /**
    * Sets the number of vertices to use in the list of vertices.
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

   public List<YoTuple2DDefinition> getVertices()
   {
      return vertices;
   }

   public String getNumberOfVertices()
   {
      return numberOfVertices;
   }

   @Override
   public YoGraphicPolygon2DDefinition copy()
   {
      return new YoGraphicPolygon2DDefinition(this);
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
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, fillColor=%s, strokeColor=%s, strokeWidth=%s, vertices=%s, numberOfVertices=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           fillColor,
                           strokeColor,
                           strokeWidth,
                           indentedListString(indent, true, vertices, Object::toString),
                           numberOfVertices);
   }
}
