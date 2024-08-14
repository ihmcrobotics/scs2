package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.List;
import java.util.Objects;

/**
 * A {@code YoGraphicPointcloud3DDefinition} is a template for creating multiple 3D points and which
 * components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicPointcloud3DDefinition} is to be passed before initialization of a session
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
public class YoGraphicPointcloud3DDefinition extends YoGraphic3DDefinition
{
   /** The positions for the points. */
   private List<YoTuple3DDefinition> points;
   /**
    * The number of points to use in the {@link #points} list. When {@code null}, all the points will
    * be rendered.
    */
   private String numberOfPoints;
   /** The size of the graphics, when rendered as spheres, it corresponds to the diameter. */
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
    * Creates a new yoGraphic definition for rendering a series of points.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicPointcloud3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPointcloud3DDefinition(YoGraphicPointcloud3DDefinition other)
   {
      super(other);
      points = other.points != null ? other.points.stream().map(YoTuple3DDefinition::copy).toList() : null;
      numberOfPoints = other.numberOfPoints;
      size = other.size;
      graphicName = other.graphicName;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerListField("points", this::getPoints, this::setPoints, "p", Object::toString, YoTuple3DDefinition::parse);
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
   public void setPoints(List<YoTuple3DDefinition> points)
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
    * Sets the size of the graphic, when rendered as spheres, it corresponds to the diameter.
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
    * Sets the size of the graphic, when rendered as spheres, it corresponds to the diameter.
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

   public List<YoTuple3DDefinition> getPoints()
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
   public YoGraphicPointcloud3DDefinition copy()
   {
      return new YoGraphicPointcloud3DDefinition(this);
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
      else if (object instanceof YoGraphicPointcloud3DDefinition other)
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
      String out = "%s [name=%s, visible=%b, color=%s, points=%s, numberOfPoints=%s, size=%s, graphicName=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           color,
                           indentedListString(indent, true, points, Object::toString),
                           numberOfPoints,
                           size,
                           graphicName);
   }
}
