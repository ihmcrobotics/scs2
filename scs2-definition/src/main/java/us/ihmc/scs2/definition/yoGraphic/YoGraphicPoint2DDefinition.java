package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.Objects;

/**
 * A {@code YoGraphicPoint2DDefinition} is a template for creating 2D point and which components can
 * be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2D.png"
 * width=150px/>
 * <p>
 * A 2D yoGraphic is rendered in the overhead plotter panel and it can be back by
 * {@code YoVariable}s allowing it to move and change at runtime.
 * </p>
 * <p>
 * The {@code YoGraphicPoint2DDefinition} is to be passed before initialization of a session (either
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
@XmlRootElement(name = "YoGraphicPoint2D")
public class YoGraphicPoint2DDefinition extends YoGraphic2DDefinition
{
   /** The position for the point. */
   private YoTuple2DDefinition position;
   /** The size of the graphic, when rendered as a circle, it corresponds to the diameter. */
   private String size;
   /**
    * The graphic name used to retrieve the type of graphic to visualize the point as. Here are some
    * examples:
    * <ul>
    * <li>"plus":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/plus_icon.png"
    * width=100px/>
    * <li>"cross":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/cross_icon.png"
    * width=100px/>
    * <li>"circle":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_icon.png"
    * width=100px/>
    * <li>"circle_plus":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_plus_icon.png"
    * width=100px/>
    * <li>"circle_cross":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_cross_icon.png"
    * width=100px/>
    * <li>"diamond":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_icon.png"
    * width=100px/>
    * <li>"diamond_plus":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_plus_icon.png"
    * width=100px/>
    * <li>"square":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_icon.png"
    * width=100px/>
    * <li>"square_cross":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_cross_icon.png"
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
   public YoGraphicPoint2DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPoint2DDefinition(YoGraphicPoint2DDefinition other)
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
      registerTuple2DField("position", this::getPosition, this::setPosition);
      registerStringField("size", this::getSize, this::setSize);
      registerStringField("graphicName", this::getGraphicName, this::setGraphicName);
   }

   /**
    * Sets the position for the point.
    *
    * @param position the position for the point.
    */
   @XmlElement
   public void setPosition(YoTuple2DDefinition position)
   {
      this.position = position;
   }

   /**
    * Sets the size of the graphic, when rendered as a circle, it corresponds to the diameter.
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
    * Sets the size of the graphic, when rendered as a circle, it corresponds to the diameter.
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
    * <li>"plus":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/plus_icon.png"
    * width=100px/>
    * <li>"cross":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/cross_icon.png"
    * width=100px/>
    * <li>"circle":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_icon.png"
    * width=100px/>
    * <li>"circle_plus":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_plus_icon.png"
    * width=100px/>
    * <li>"circle_cross":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_cross_icon.png"
    * width=100px/>
    * <li>"diamond":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_icon.png"
    * width=100px/>
    * <li>"diamond_plus":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_plus_icon.png"
    * width=100px/>
    * <li>"square":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_icon.png"
    * width=100px/>
    * <li>"square_cross":<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_cross_icon.png"
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

   public YoTuple2DDefinition getPosition()
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
   public YoGraphicPoint2DDefinition copy()
   {
      return new YoGraphicPoint2DDefinition(this);
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
      else if (object instanceof YoGraphicPoint2DDefinition other)
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
