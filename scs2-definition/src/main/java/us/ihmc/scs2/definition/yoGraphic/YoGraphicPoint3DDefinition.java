package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.Objects;

/**
 * A {@code YoGraphicPoint3DDefinition} is a template for creating 3D point and which components can
 * be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicPoint3DDefinition} is to be passed before initialization of a session (either
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
@XmlRootElement(name = "YoGraphicSphere3D")
public class YoGraphicPoint3DDefinition extends YoGraphic3DDefinition
{
   /** The position for the point. */
   private YoTuple3DDefinition position;
   /** The size of the graphic, when rendered as a sphere, it corresponds to the diameter. */
   private String size;
   /**
    * The graphic name used to retrieve the type of graphic to visualize the point as. Here are some
    * examples:
    * <ul>
    * <li>"sphere":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/sphere.png"
    * width=100px/>
    * <li>"cube":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/cube.png"
    * width=100px/>
    * <li>"tetrahedron":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/tetrahedron.png"
    * width=100px/>
    * <li>"icosahedron":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/icosahedron.png"
    * width=100px/>
    * </ul>
    */
   private String graphicName;

   /**
    * Creates a new yoGraphic definition for rendering a point.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicPoint3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPoint3DDefinition(YoGraphicPoint3DDefinition other)
   {
      super(other);
      position = other.position == null ? null : other.position.copy();
      size = other.size;
      graphicName = other.graphicName;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerTuple3DField("position", this::getPosition, this::setPosition);
      registerStringField("size", this::getSize, this::setSize);
      registerStringField("graphicName", this::getGraphicName, this::setGraphicName);
   }

   /**
    * Sets the position for the point.
    *
    * @param position the position for the point.
    */
   @XmlElement
   public void setPosition(YoTuple3DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the size of the graphic, when rendered as a sphere, it corresponds to the diameter.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param size the size of the graphic.
    */
   public void setSize(double size)
   {
      this.size = Double.toString(size);
   }

   /**
    * Sets the size of the graphic, when rendered as a sphere, it corresponds to the diameter.
    * <p>
    * Using this method allows to back the size with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    *
    * @param size the size of the graphic.
    */
   @XmlElement
   public void setSize(String size)
   {
      this.size = size;
   }

   /**
    * Sets the type of the graphic:
    * <ul>
    * <li>"sphere":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/sphere.png"
    * width=100px/>
    * <li>"cube":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/cube.png"
    * width=100px/>
    * <li>"tetrahedron":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/tetrahedron.png"
    * width=100px/>
    * <li>"icosahedron":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3DGraphics/icosahedron.png"
    * width=100px/>
    * </ul>
    *
    * @param graphicName the name of the graphic to use.
    */
   @XmlElement
   public void setGraphicName(String graphicName)
   {
      this.graphicName = graphicName;
   }

   public YoTuple3DDefinition getPosition()
   {
      return position;
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
   public YoGraphicDefinition copy()
   {
      return new YoGraphicPoint3DDefinition(this);
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
      else if (object instanceof YoGraphicPoint3DDefinition other)
      {
         if (!Objects.equals(position, other.position))
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
}
