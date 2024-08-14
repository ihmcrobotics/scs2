package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.List;
import java.util.Objects;

/**
 * A {@code YoGraphicPointcloud2DDefinition} is a template for creating multiple 2D points and which
 * components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX2D.png"
 * width=150px/>
 * <p>
 * A 2D yoGraphic is rendered in the overhead plotter panel and it can be back by
 * {@code YoVariable}s allowing it to move and change at runtime.
 * </p>
 * <p>
 * The {@code YoGraphicPointcloud2DDefinition} is to be passed before initialization of a session
 * (either before starting a simulation or when creating a yoVariable server), such that the SCS GUI
 * can use the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicPointcloud3D")
public class YoGraphicPointcloud2DDefinition extends YoGraphic2DDefinition
{
   /** The positions for the points. */
   private List<YoTuple2DDefinition> points;
   /**
    * The number of points to use in the {@link #points} list. When {@code null}, all the points will
    * be rendered.
    */
   private String numberOfPoints;
   /** The size of the graphics, when rendered as circles, it corresponds to the diameter. */
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
    * Creates a new yoGraphic definition for rendering a series of points.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicPointcloud2DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPointcloud2DDefinition(YoGraphicPointcloud2DDefinition other)
   {
      super(other);
      points = other.points != null ? other.points.stream().map(YoTuple2DDefinition::copy).toList() : null;
      numberOfPoints = other.numberOfPoints;
      size = other.size;
      graphicName = other.graphicName;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerListField("points", this::getPoints, this::setPoints, "p", Object::toString, YoTuple2DDefinition::parse);
      registerStringField("numberOfPoints", this::getNumberOfPoints, this::setNumberOfPoints);
      registerStringField("size", this::getSize, this::setSize);
      registerStringField("graphicName", this::getGraphicName, this::setGraphicName);
   }

   /**
    * Sets the positions for the points.
    *
    * @param points the position for the point.
    */
   @XmlElement
   public void setPoints(List<YoTuple2DDefinition> points)
   {
      this.points = points;
   }

   /**
    * Sets the number of points to use in the points list. When {@code null}, all the points will be
    * rendered.
    * <p>
    * This field can be backed with a {@code YoVariable} by giving the variable name/fullname.
    * </p>
    *
    * @param numberOfPoints the number of points to render.
    */
   @XmlElement
   public void setNumberOfPoints(String numberOfPoints)
   {
      this.numberOfPoints = numberOfPoints;
   }

   /**
    * Sets the size of the graphic, when rendered as circles, it corresponds to the diameter.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param size the size of the graphic.
    */
   public void setSize(double size)
   {
      setSize(Double.toString(size));
   }

   /**
    * Sets the size of the graphic, when rendered as circles, it corresponds to the diameter.
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

   public List<YoTuple2DDefinition> getPoints()
   {
      return points;
   }

   public String getNumberOfPoints()
   {
      return numberOfPoints;
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
   public YoGraphicPointcloud2DDefinition copy()
   {
      return new YoGraphicPointcloud2DDefinition(this);
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
      else if (object instanceof YoGraphicPointcloud2DDefinition other)
      {
         if (!Objects.equals(points, other.points))
            return false;
         if (!Objects.equals(numberOfPoints, other.numberOfPoints))
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

   @Override
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, fillColor=%s, strokeColor=%s, strokeWidth=%s, points=%s, numberOfPoints=%s, size=%s, graphicName=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           fillColor,
                           strokeColor,
                           strokeWidth,
                           indentedListString(indent, true, points, Object::toString),
                           numberOfPoints,
                           size,
                           graphicName);
   }
}
