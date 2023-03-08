package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBADoubleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBAIntDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class implements factories to assist in creating new {@link YoGraphicDefinition}s.
 * 
 * @author Sylvain Bertrand
 */
public class YoGraphicDefinitionFactory
{
   private static final double DEFAULT_STROKE_WIDTH = 1.5;
   public static final ColorDefinition DEFAULT_COLOR = ColorDefinitions.Gray();

   /**
    * Enum to gather the different options for rendering a point 2D.
    * 
    * @author Sylvain Bertrand
    */
   public enum DefaultPoint2DGraphic
   {
      // Graphics drawn with stroke and no fill
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/plus_icon.png"
       * width=150px/>
       */
      PLUS(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/cross_icon.png"
       * width=150px/>
       */
      CROSS(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_icon.png"
       * width=150px/>
       */
      CIRCLE(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circe_plus_icon.png"
       * width=150px/>
       */
      CIRCLE_PLUS(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_cross_icon.png"
       * width=150px/>
       */
      CIRCLE_CROSS(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_icon.png"
       * width=150px/>
       */
      DIAMOND(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_plus_icon.png"
       * width=150px/>
       */
      DIAMOND_PLUS(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_icon.png"
       * width=150px/>
       */
      SQUARE(false),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_cross_icon.png"
       * width=150px/>
       */
      SQUARE_CROSS(false),
      // Graphics drawn filled and no stroke
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/circle_filled_icon.png"
       * width=150px/>
       */
      CIRCLE_FILLED(true),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/diamond_filled_icon.png"
       * width=150px/>
       */
      DIAMOND_FILLED(true),
      /**
       * <img src=
       * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2DGraphics/square_filled_icon.png"
       * width=150px/>
       */
      SQUARE_FILLED(true);

      private final String graphicName;
      private final boolean filled;

      private DefaultPoint2DGraphic(boolean filled)
      {
         this.filled = filled;
         this.graphicName = name().charAt(0) + name().substring(1).replace("_FILLED", "").toLowerCase().replace("_", " ");
      }

      public boolean isFilled()
      {
         return filled;
      }

      public String getGraphicName()
      {
         return graphicName;
      }
   }

   /**
    * Creates a new yoGraphic that represents the given line-segment: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoLineFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param lineSegment the line-segment.
    * @param strokeColor the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicLine2DDefinition newYoGraphicLineSegment2DDefinition(String name, YoFrameLineSegment2D lineSegment, PaintDefinition strokeColor)
   {
      return newYoGraphicLineSegment2DDefinition(name,
                                                 (YoFramePoint2D) lineSegment.getFirstEndpoint(),
                                                 (YoFramePoint2D) lineSegment.getSecondEndpoint(),
                                                 strokeColor);
   }

   /**
    * Creates a new yoGraphic that represents the given line-segment: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoLineFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name           the name for yoGraphic.
    * @param firstEndpoint  the line-segment first endpoint.
    * @param secondEndpoint the line-segment second endpoint.
    * @param strokeColor    the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicLine2DDefinition newYoGraphicLineSegment2DDefinition(String name,
                                                                               YoFrameTuple2D firstEndpoint,
                                                                               YoFrameTuple2D secondEndpoint,
                                                                               PaintDefinition strokeColor)
   {
      YoGraphicLine2DDefinition definition = new YoGraphicLine2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setOrigin(newYoTuple2DDefinition(firstEndpoint));
      definition.setDestination(newYoTuple2DDefinition(secondEndpoint));
      definition.setStrokeColor(strokeColor);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param position    the position to track. Only the x and y coordinates will be used.
    * @param size        the graphic size.
    * @param color       the color.
    * @param graphicType the graphic type for displaying the point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(String name,
                                                                YoFrameTuple3D position,
                                                                double size,
                                                                PaintDefinition color,
                                                                DefaultPoint2DGraphic graphicType)
   {
      return newYoGraphicPoint2D(name, position, position.getReferenceFrame(), size, color, graphicType);
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param position    the position to track.
    * @param size        the graphic size.
    * @param color       the color.
    * @param graphicType the graphic type for displaying the point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(String name,
                                                                YoFrameTuple2D position,
                                                                double size,
                                                                PaintDefinition color,
                                                                DefaultPoint2DGraphic graphicType)
   {
      return newYoGraphicPoint2D(name, position, position.getReferenceFrame(), size, color, graphicType);
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name          the name for yoGraphic.
    * @param position      the position to track. Only the x and y coordinates will be used.
    * @param positionFrame the frame in which the position is expressed.
    * @param size          the graphic size.
    * @param color         the color.
    * @param graphicType   the graphic type for displaying the point. See
    *                      {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(String name,
                                                                YoTuple3D position,
                                                                ReferenceFrame positionFrame,
                                                                double size,
                                                                PaintDefinition color,
                                                                DefaultPoint2DGraphic graphicType)
   {
      return newYoGraphicPoint2D(name, new YoPoint2D(position.getYoX(), position.getYoY()), positionFrame, size, color, graphicType);
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name          the name for yoGraphic.
    * @param position      the position to track.
    * @param positionFrame the frame in which the position is expressed.
    * @param size          the graphic size.
    * @param color         the color.
    * @param graphicType   the graphic type for displaying the point. See
    *                      {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(String name,
                                                                YoTuple2D position,
                                                                ReferenceFrame positionFrame,
                                                                double size,
                                                                PaintDefinition color,
                                                                DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPoint2DDefinition definition = new YoGraphicPoint2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple2DDefinition(position, positionFrame));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(size);
      if (graphicType.isFilled())
         definition.setFillColor(color);
      else
         definition.setStrokeColor(color);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param definition3D the corresponding 3D definition to convert into a 2D yoGraphic.
    * @param graphicType  the graphic type for displaying the point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(YoGraphicPoint3DDefinition definition3D, DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPoint2DDefinition definition = new YoGraphicPoint2DDefinition();
      definition.setName(definition3D.getName());
      definition.setVisible(definition3D.isVisible());
      definition.setPosition(new YoTuple2DDefinition(definition3D.getPosition().getX(),
                                                     definition3D.getPosition().getY(),
                                                     definition3D.getPosition().getReferenceFrame()));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(definition3D.getSize());
      if (graphicType.isFilled())
         definition.setFillColor(definition3D.getColor());
      else
         definition.setStrokeColor(definition3D.getColor());
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param points      the list of positions to track.
    * @param size        the graphic size for each point.
    * @param color       the color.
    * @param graphicType the graphic type for displaying each point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud2DDefinition newYoGraphicPointcloud2D(String name,
                                                                          List<? extends YoFrameTuple2D> points,
                                                                          double size,
                                                                          PaintDefinition color,
                                                                          DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPointcloud2DDefinition definition = new YoGraphicPointcloud2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPoints(points.stream().map(p -> newYoTuple2DDefinition(p)).collect(Collectors.toList()));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(size);
      if (graphicType.isFilled())
         definition.setFillColor(color);
      else
         definition.setStrokeColor(color);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param points      the list of positions to track.
    * @param frame       the frame in which the positions are expressed.
    * @param size        the graphic size for each point.
    * @param color       the color.
    * @param graphicType the graphic type for displaying each point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud2DDefinition newYoGraphicPointcloud2D(String name,
                                                                          List<? extends YoTuple2D> points,
                                                                          ReferenceFrame frame,
                                                                          double size,
                                                                          PaintDefinition color,
                                                                          DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPointcloud2DDefinition definition = new YoGraphicPointcloud2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPoints(points.stream().map(p -> newYoTuple2DDefinition(p, frame)).collect(Collectors.toList()));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(size);
      if (graphicType.isFilled())
         definition.setFillColor(color);
      else
         definition.setStrokeColor(color);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param definition3D the corresponding 3D definition to convert into a 2D yoGraphic.
    * @param graphicType  the graphic type for displaying each point. See
    *                     {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud2DDefinition newYoGraphicPointcloud2D(YoGraphicPointcloud3DDefinition definition3D, DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPointcloud2DDefinition definition = new YoGraphicPointcloud2DDefinition();
      definition.setName(definition3D.getName());
      definition.setVisible(definition3D.isVisible());
      definition.setPoints(definition3D.getPoints().stream()
                                       .map(tuple3D -> new YoTuple2DDefinition(tuple3D.getX(), tuple3D.getY(), tuple3D.getReferenceFrame()))
                                       .collect(Collectors.toList()));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(definition3D.getSize());
      if (graphicType.isFilled())
         definition.setFillColor(definition3D.getColor());
      else
         definition.setStrokeColor(definition3D.getColor());
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param points      the list of positions to track.
    * @param size        the graphic size for each point.
    * @param color       the color.
    * @param graphicType the graphic type for displaying each point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud2DDefinition newYoGraphicPointcloud2DFrom3D(String name,
                                                                                List<? extends YoFrameTuple3D> points,
                                                                                double size,
                                                                                PaintDefinition color,
                                                                                DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPointcloud2DDefinition definition = new YoGraphicPointcloud2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPoints(points.stream().map(p -> newYoTuple2DDefinition(p.getYoX(), p.getYoY())).collect(Collectors.toList()));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(size);
      if (graphicType.isFilled())
         definition.setFillColor(color);
      else
         definition.setStrokeColor(color);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param points      the list of positions to track.
    * @param frame       the frame in which the positions are expressed.
    * @param size        the graphic size for each point.
    * @param color       the color.
    * @param graphicType the graphic type for displaying each point. See {@link DefaultPoint2DGraphic}.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud2DDefinition newYoGraphicPointcloud2DFrom3D(String name,
                                                                                List<? extends YoTuple3D> points,
                                                                                ReferenceFrame frame,
                                                                                double size,
                                                                                PaintDefinition color,
                                                                                DefaultPoint2DGraphic graphicType)
   {
      YoGraphicPointcloud2DDefinition definition = new YoGraphicPointcloud2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPoints(points.stream().map(p -> newYoTuple2DDefinition(p.getYoX(), p.getYoY(), frame)).collect(Collectors.toList()));
      definition.setGraphicName(graphicType.getGraphicName());
      definition.setSize(size);
      if (graphicType.isFilled())
         definition.setFillColor(color);
      else
         definition.setStrokeColor(color);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given polygon: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolygonFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name        the name for yoGraphic.
    * @param polygon     the convex polygon to visualize.
    * @param strokeColor the stroke color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolygon2DDefinition newYoGraphicPolygon2D(String name, YoFrameConvexPolygon2D polygon, PaintDefinition strokeColor)
   {
      return newYoGraphicPolygon2D(name, polygon, strokeColor, false);
   }

   /**
    * Creates a new yoGraphic that represents the given polygon: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolygonFX2D.png"
    * width=150px/>
    * <p>
    * The yoGraphic will appear in the 2D overhead plotter.
    * </p>
    * 
    * @param name    the name for yoGraphic.
    * @param polygon the convex polygon to visualize.
    * @param color   either the fill or the stroke color.
    * @param fill    whether to fill the polygon.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolygon2DDefinition newYoGraphicPolygon2D(String name, YoFrameConvexPolygon2D polygon, PaintDefinition color, boolean fill)
   {
      YoGraphicPolygon2DDefinition definition = new YoGraphicPolygon2DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setVertices(polygon.getVertexBuffer().stream().map(v -> newYoTuple2DDefinition(v)).toList());
      definition.setNumberOfVertices(toPropertyName(polygon.getYoNumberOfVertices()));
      if (fill)
         definition.setFillColor(color);
      else
         definition.setStrokeColor(color);
      definition.setStrokeWidth(DEFAULT_STROKE_WIDTH);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents an arrow: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoArrowFX3D.png"
    * width=150px/>
    * 
    * @param name      the name for yoGraphic.
    * @param origin    the position of the arrow's origin.
    * @param direction the arrow's direction. Its magnitude is also used to scale the length and radius
    *                  of the arrow.
    * @param scale     the factor used together with the direction's magnitude to scale the length and
    *                  radius.
    * @param color     the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoFrameTuple3D origin,
                                                                YoFrameTuple3D direction,
                                                                double scale,
                                                                PaintDefinition color)
   {
      return newYoGraphicArrow3D(name, origin, origin.getReferenceFrame(), direction, direction.getReferenceFrame(), scale, color);
   }

   /**
    * Creates a new yoGraphic that represents an arrow: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoArrowFX3D.png"
    * width=150px/>
    * 
    * @param name        the name for yoGraphic.
    * @param origin      the position of the arrow's origin.
    * @param direction   the arrow's direction.
    * @param scaleLength whether the length should scale with the direction's magnitude.
    * @param bodyLength  the default length for the arrow's body.
    * @param headLength  the default length for the arrow's head.
    * @param scaleRadius whether the radius should scale with the direction's magnitude.
    * @param bodyRadius  the default radius for the arrow's body.
    * @param headRadius  the default radius for the arrow's head.
    * @param color       the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoFrameTuple3D origin,
                                                                YoFrameTuple3D direction,
                                                                boolean scaleLength,
                                                                double bodyLength,
                                                                double headLength,
                                                                boolean scaleRadius,
                                                                double bodyRadius,
                                                                double headRadius,
                                                                PaintDefinition color)
   {
      return newYoGraphicArrow3D(name,
                                 origin,
                                 origin.getReferenceFrame(),
                                 direction,
                                 direction.getReferenceFrame(),
                                 scaleLength,
                                 bodyLength,
                                 headLength,
                                 scaleRadius,
                                 bodyRadius,
                                 headRadius,
                                 color);
   }

   /**
    * Creates a new yoGraphic that represents an arrow: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoArrowFX3D.png"
    * width=150px/>
    * 
    * @param name           the name for yoGraphic.
    * @param origin         the position of the arrow's origin.
    * @param originFrame    the reference frame in which the origin is expressed.
    * @param direction      the arrow's direction. Its magnitude is also used to scale the length and
    *                       radius of the arrow.
    * @param directionFrame the reference frame in which the direction is expressed.
    * @param scale          the factor used together with the direction's magnitude to scale the length
    *                       and radius.
    * @param color          the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoTuple3D origin,
                                                                ReferenceFrame originFrame,
                                                                YoTuple3D direction,
                                                                ReferenceFrame directionFrame,
                                                                double scale,
                                                                PaintDefinition color)
   {
      boolean scaleLength = true;
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      boolean scaleRadius = true;
      double bodyRadius = scale * 0.015;
      double headRadius = bodyRadius * 2.5;
      return newYoGraphicArrow3D(name,
                                 origin,
                                 originFrame,
                                 direction,
                                 directionFrame,
                                 scaleLength,
                                 bodyLength,
                                 headLength,
                                 scaleRadius,
                                 bodyRadius,
                                 headRadius,
                                 color);
   }

   /**
    * Creates a new yoGraphic that represents an arrow: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoArrowFX3D.png"
    * width=150px/>
    * 
    * @param name           the name for yoGraphic.
    * @param origin         the position of the arrow's origin.
    * @param originFrame    the reference frame in which the origin is expressed.
    * @param direction      the arrow's direction. Its magnitude is also used to scale the length and
    *                       radius of the arrow.
    * @param directionFrame the reference frame in which the direction is expressed.
    * @param scaleLength    whether the length should scale with the direction's magnitude.
    * @param bodyLength     the default length for the arrow's body.
    * @param headLength     the default length for the arrow's head.
    * @param scaleRadius    whether the radius should scale with the direction's magnitude.
    * @param bodyRadius     the default radius for the arrow's body.
    * @param headRadius     the default radius for the arrow's head.
    * @param color          the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoTuple3D origin,
                                                                ReferenceFrame originFrame,
                                                                YoTuple3D direction,
                                                                ReferenceFrame directionFrame,
                                                                boolean scaleLength,
                                                                double bodyLength,
                                                                double headLength,
                                                                boolean scaleRadius,
                                                                double bodyRadius,
                                                                double headRadius,
                                                                PaintDefinition color)
   {
      YoGraphicArrow3DDefinition definition = new YoGraphicArrow3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setOrigin(newYoTuple3DDefinition(origin, originFrame));
      definition.setDirection(newYoTuple3DDefinition(direction, directionFrame));
      definition.setScaleLength(scaleLength);
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setScaleRadius(scaleRadius);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name  the name for yoGraphic.
    * @param pose  the pose of the coordinate system.
    * @param scale the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long arrows.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name, YoFramePose3D pose, double scale)
   {
      return newYoGraphicCoordinateSystem3D(name, pose, scale, DEFAULT_COLOR);
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name  the name for yoGraphic.
    * @param pose  the pose of the coordinate system.
    * @param scale the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long arrows.
    * @param color the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name, YoFramePose3D pose, double scale, PaintDefinition color)
   {
      return newYoGraphicCoordinateSystem3D(name, pose.getPosition(), pose.getOrientation(), scale, color);
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name        the name for yoGraphic.
    * @param position    the position of the coordinate system origin.
    * @param orientation the orientation of the coordinate system.
    * @param scale       the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long arrows.
    * @param color       the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoint3D position,
                                                                                      YoFrameQuaternion orientation,
                                                                                      double scale,
                                                                                      PaintDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3D(name, position, orientation, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name        the name for yoGraphic.
    * @param position    the position of the coordinate system origin.
    * @param orientation the orientation of the coordinate system.
    * @param bodyLength  the length for the body for each arrow.
    * @param headLength  the length for the head for each arrow.
    * @param bodyRadius  the radius for the body for each arrow.
    * @param headRadius  the radius for the head for each arrow.
    * @param color       the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoint3D position,
                                                                                      YoFrameQuaternion orientation,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(position));
      definition.setOrientation(newYoQuaternionDefinition(orientation));
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name  the name for yoGraphic.
    * @param pose  the pose of the coordinate system.
    * @param scale the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long arrows.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name, YoFramePoseUsingYawPitchRoll pose, double scale)
   {
      return newYoGraphicCoordinateSystem3D(name, pose, scale, DEFAULT_COLOR);
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name  the name for yoGraphic.
    * @param pose  the pose of the coordinate system.
    * @param scale the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long arrows.
    * @param color the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoseUsingYawPitchRoll pose,
                                                                                      double scale,
                                                                                      PaintDefinition color)
   {
      return newYoGraphicCoordinateSystem3D(name, pose.getPosition(), pose.getYawPitchRoll(), scale, color);
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name        the name for yoGraphic.
    * @param position    the position of the coordinate system origin.
    * @param orientation the orientation of the coordinate system.
    * @param scale       the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long arrows.
    * @param color       the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoint3D position,
                                                                                      YoFrameYawPitchRoll orientation,
                                                                                      double scale,
                                                                                      PaintDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3D(name, position, orientation, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   /**
    * Creates a new yoGraphic that represents a coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name        the name for yoGraphic.
    * @param position    the position of the coordinate system origin.
    * @param orientation the orientation of the coordinate system.
    * @param bodyLength  the length for the body for each arrow.
    * @param headLength  the length for the head for each arrow.
    * @param bodyRadius  the radius for the body for each arrow.
    * @param headRadius  the radius for the head for each arrow.
    * @param color       the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoint3D position,
                                                                                      YoFrameYawPitchRoll orientation,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(position));
      definition.setOrientation(newYoYawPitchRollDefinition(orientation));
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name              the name for yoGraphic.
    * @param constantFramePose the pose of the coordinate system.
    * @param scale             the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long
    *                          arrows.
    * @param color             the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      FramePose3DReadOnly constantFramePose,
                                                                                      double scale,
                                                                                      PaintDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3D(name,
                                            constantFramePose.getPosition(),
                                            constantFramePose.getOrientation(),
                                            constantFramePose.getReferenceFrame(),
                                            bodyLength,
                                            headLength,
                                            bodyRadius,
                                            headRadius,
                                            color);
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name              the name for yoGraphic.
    * @param constantFramePose the pose of the coordinate system.
    * @param bodyLength        the length for the body for each arrow.
    * @param headLength        the length for the head for each arrow.
    * @param bodyRadius        the radius for the body for each arrow.
    * @param headRadius        the radius for the head for each arrow.
    * @param color             the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      FramePose3DReadOnly constantFramePose,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      return newYoGraphicCoordinateSystem3D(name,
                                            constantFramePose.getPosition(),
                                            constantFramePose.getOrientation(),
                                            constantFramePose.getReferenceFrame(),
                                            bodyLength,
                                            headLength,
                                            bodyRadius,
                                            headRadius,
                                            color);
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name              the name for yoGraphic.
    * @param constantFramePose the pose of the coordinate system expressed in world frame.
    * @param scale             the size for each arrow, i.e. a scale of 1 corresponds to 1 meter long
    *                          arrows.
    * @param color             the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Pose3DReadOnly constantFramePose,
                                                                                      double scale,
                                                                                      PaintDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3D(name,
                                            constantFramePose.getPosition(),
                                            constantFramePose.getOrientation(),
                                            null,
                                            bodyLength,
                                            headLength,
                                            bodyRadius,
                                            headRadius,
                                            color);
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name         the name for yoGraphic.
    * @param constantPose the pose of the coordinate system expressed in world frame.
    * @param bodyLength   the length for the body for each arrow.
    * @param headLength   the length for the head for each arrow.
    * @param bodyRadius   the radius for the body for each arrow.
    * @param headRadius   the radius for the head for each arrow.
    * @param color        the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Pose3DReadOnly constantPose,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      return newYoGraphicCoordinateSystem3D(name,
                                            constantPose.getPosition(),
                                            null,
                                            constantPose.getOrientation(),
                                            null,
                                            bodyLength,
                                            headLength,
                                            bodyRadius,
                                            headRadius,
                                            color);
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name         the name for yoGraphic.
    * @param constantPose the pose of the coordinate system.
    * @param poseFrame    the frame in which the pose is expressed.
    * @param bodyLength   the length for the body for each arrow.
    * @param headLength   the length for the head for each arrow.
    * @param bodyRadius   the radius for the body for each arrow.
    * @param headRadius   the radius for the head for each arrow.
    * @param color        the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Pose3DReadOnly constantPose,
                                                                                      ReferenceFrame poseFrame,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      return newYoGraphicCoordinateSystem3D(name,
                                            constantPose.getPosition(),
                                            poseFrame,
                                            constantPose.getOrientation(),
                                            poseFrame,
                                            bodyLength,
                                            headLength,
                                            bodyRadius,
                                            headRadius,
                                            color);
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name                the name for yoGraphic.
    * @param constantPosition    the position of the coordinate system origin.
    * @param constantOrientation the orientation of the coordinate system.
    * @param poseFrame           the frame in which the position and orientation are expressed.
    * @param bodyLength          the length for the body for each arrow.
    * @param headLength          the length for the head for each arrow.
    * @param bodyRadius          the radius for the body for each arrow.
    * @param headRadius          the radius for the head for each arrow.
    * @param color               the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Point3DReadOnly constantPosition,
                                                                                      Orientation3DReadOnly constantOrientation,
                                                                                      ReferenceFrame poseFrame,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      return newYoGraphicCoordinateSystem3D(name,
                                            constantPosition,
                                            poseFrame,
                                            constantOrientation,
                                            poseFrame,
                                            bodyLength,
                                            headLength,
                                            bodyRadius,
                                            headRadius,
                                            color);
   }

   /**
    * Creates a new yoGraphic that represents a constant coordinate system: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoCoordinateSystemFX3D.png"
    * width=150px/>
    * 
    * @param name                the name for yoGraphic.
    * @param constantPosition    the position of the coordinate system origin.
    * @param positionFrame       the frame in which the position is expressed.
    * @param constantOrientation the orientation of the coordinate system.
    * @param orientationFrame    the frame in which the orientation is expressed.
    * @param bodyLength          the length for the body for each arrow.
    * @param headLength          the length for the head for each arrow.
    * @param bodyRadius          the radius for the body for each arrow.
    * @param headRadius          the radius for the head for each arrow.
    * @param color               the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Point3DReadOnly constantPosition,
                                                                                      ReferenceFrame positionFrame,
                                                                                      Orientation3DReadOnly constantOrientation,
                                                                                      ReferenceFrame orientationFrame,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      PaintDefinition color)
   {
      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(constantPosition, positionFrame));
      definition.setOrientation(newYoOrientation3DDefinition(constantOrientation, orientationFrame));
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3D.png"
    * width=150px/>
    * 
    * @param name     the name for yoGraphic.
    * @param position the position to track.
    * @param size     the graphic size.
    * @param color    the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint3DDefinition newYoGraphicPoint3D(String name, YoFrameTuple3D position, double size, PaintDefinition color)
   {
      return newYoGraphicPoint3D(name, position, position.getReferenceFrame(), size, color);
   }

   /**
    * Creates a new yoGraphic that represents the given point: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointFX3D.png"
    * width=150px/>
    * 
    * @param name          the name for yoGraphic.
    * @param position      the position to track.
    * @param positionFrame the frame in which the position is expressed.
    * @param size          the graphic size.
    * @param color         the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPoint3DDefinition newYoGraphicPoint3D(String name,
                                                                YoTuple3D position,
                                                                ReferenceFrame positionFrame,
                                                                double size,
                                                                PaintDefinition color)
   {
      YoGraphicPoint3DDefinition definition = new YoGraphicPoint3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(position, positionFrame));
      definition.setSize(size);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX3D.png"
    * width=150px/>
    * 
    * @param name   the name for yoGraphic.
    * @param points the list of positions to track.
    * @param size   the graphic size for each point.
    * @param color  the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud3DDefinition newYoGraphicPointcloud3D(String name,
                                                                          List<? extends YoFrameTuple3D> points,
                                                                          double size,
                                                                          PaintDefinition color)
   {
      YoGraphicPointcloud3DDefinition definition = new YoGraphicPointcloud3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPoints(points.stream().map(p -> newYoTuple3DDefinition(p)).collect(Collectors.toList()));
      definition.setSize(size);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given list of points: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPointcloudFX3D.png"
    * width=150px/>
    * 
    * @param name   the name for yoGraphic.
    * @param points the list of positions to track.
    * @param frame  the frame in which the positions are expressed.
    * @param size   the graphic size for each point.
    * @param color  the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPointcloud3DDefinition newYoGraphicPointcloud3D(String name,
                                                                          List<? extends YoTuple3D> points,
                                                                          ReferenceFrame frame,
                                                                          double size,
                                                                          PaintDefinition color)
   {
      YoGraphicPointcloud3DDefinition definition = new YoGraphicPointcloud3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPoints(points.stream().map(p -> newYoTuple3DDefinition(p, frame)).collect(Collectors.toList()));
      definition.setSize(size);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given convex polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolygonExtrudedFX3D.png"
    * width=150px/>
    * 
    * @param name      the name for yoGraphic.
    * @param pose      the pose of the polygon's base.
    * @param polygon   the convex polygon to visualize.
    * @param thickness the extrusion thickness.
    * @param color     the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolygonExtruded3DDefinition newYoGraphicPolygonExtruded3DDefinition(String name,
                                                                                              YoFramePose3D pose,
                                                                                              YoFrameConvexPolygon2D polygon,
                                                                                              double thickness,
                                                                                              PaintDefinition color)
   {
      YoGraphicPolygonExtruded3DDefinition definition = new YoGraphicPolygonExtruded3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(pose.getPosition()));
      definition.setOrientation(newYoQuaternionDefinition(pose.getOrientation()));
      definition.setVertices(polygon.getVertexBuffer().stream().map(v -> newYoTuple2DDefinition(v)).toList());
      definition.setNumberOfVertices(toPropertyName(polygon.getYoNumberOfVertices()));
      definition.setThickness(thickness);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolygonExtrudedFX3D.png"
    * width=150px/>
    * 
    * @param name      the name for yoGraphic.
    * @param pose      the pose of the polygon's base.
    * @param vertices  the vertices of the polygon to visualize.
    * @param thickness the extrusion thickness.
    * @param color     the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolygonExtruded3DDefinition newYoGraphicPolygonExtruded3DDefinition(String name,
                                                                                              YoFramePose3D pose,
                                                                                              List<? extends Point2DReadOnly> vertices,
                                                                                              double thickness,
                                                                                              PaintDefinition color)
   {
      YoGraphicPolygonExtruded3DDefinition definition = new YoGraphicPolygonExtruded3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(pose.getPosition()));
      definition.setOrientation(newYoQuaternionDefinition(pose.getOrientation()));
      definition.setVertices(vertices.stream().map(v -> newYoTuple2DDefinition(v)).toList());
      definition.setNumberOfVertices(vertices.size());
      definition.setThickness(thickness);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new yoGraphic that represents the given polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolynomialFX3D.png"
    * width=150px/>
    * 
    * @param name                  the name for yoGraphic.
    * @param coefficientsX         the coefficients (from low to high order) for the x-axis polynomial.
    * @param numberOfCoefficientsX the number of coefficients to use for the x-axis polynomial.
    * @param coefficientsY         the coefficients (from low to high order) for the y-axis polynomial.
    * @param numberOfCoefficientsY the number of coefficients to use for the y-axis polynomial.
    * @param coefficientsZ         the coefficients (from low to high order) for the z-axis polynomial.
    * @param numberOfCoefficientsZ the number of coefficients to use for the z-axis polynomial.
    * @param startTime             the initial time for the trajectory. The polynomial is rendered in
    *                              the time range <tt>t&in;[startTime, endTime]</tt>
    * @param endTime               the final time for the trajectory. The polynomial is rendered in the
    *                              time range <tt>t&in;[startTime, endTime]</tt>
    * @param size                  the radius of the trajectory graphic.
    * @param color                 the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoVariable[] coefficientsX,
                                                                          YoInteger numberOfCoefficientsX,
                                                                          YoVariable[] coefficientsY,
                                                                          YoInteger numberOfCoefficientsY,
                                                                          YoVariable[] coefficientsZ,
                                                                          YoInteger numberOfCoefficientsZ,
                                                                          YoDouble startTime,
                                                                          YoDouble endTime,
                                                                          double size,
                                                                          PaintDefinition color)
   {
      return newYoGraphicPolynomial3D(name,
                                      coefficientsX,
                                      numberOfCoefficientsX,
                                      coefficientsY,
                                      numberOfCoefficientsY,
                                      coefficientsZ,
                                      numberOfCoefficientsZ,
                                      startTime,
                                      endTime,
                                      size,
                                      50,
                                      10,
                                      color);
   }

   /**
    * Creates a new yoGraphic that represents the given polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolynomialFX3D.png"
    * width=150px/>
    * 
    * @param name                  the name for yoGraphic.
    * @param coefficientsX         the coefficients (from low to high order) for the x-axis polynomial.
    * @param numberOfCoefficientsX the number of coefficients to use for the x-axis polynomial.
    * @param coefficientsY         the coefficients (from low to high order) for the y-axis polynomial.
    * @param numberOfCoefficientsY the number of coefficients to use for the y-axis polynomial.
    * @param coefficientsZ         the coefficients (from low to high order) for the z-axis polynomial.
    * @param numberOfCoefficientsZ the number of coefficients to use for the z-axis polynomial.
    * @param startTime             the initial time for the trajectory. The polynomial is rendered in
    *                              the time range <tt>t&in;[startTime, endTime]</tt>
    * @param endTime               the final time for the trajectory. The polynomial is rendered in the
    *                              time range <tt>t&in;[startTime, endTime]</tt>
    * @param size                  the radius of the trajectory graphic.
    * @param timeResolution        the number of divisions for the time.
    * @param numberOfDivisions     the number of radial divisions.
    * @param color                 the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoVariable[] coefficientsX,
                                                                          YoInteger numberOfCoefficientsX,
                                                                          YoVariable[] coefficientsY,
                                                                          YoInteger numberOfCoefficientsY,
                                                                          YoVariable[] coefficientsZ,
                                                                          YoInteger numberOfCoefficientsZ,
                                                                          YoDouble startTime,
                                                                          YoDouble endTime,
                                                                          double size,
                                                                          int timeResolution,
                                                                          int numberOfDivisions,
                                                                          PaintDefinition color)
   {
      return newYoGraphicPolynomial3D(name,
                                      toYoListDefinition(coefficientsX, numberOfCoefficientsX),
                                      toYoListDefinition(coefficientsY, numberOfCoefficientsY),
                                      toYoListDefinition(coefficientsZ, numberOfCoefficientsZ),
                                      startTime,
                                      0,
                                      endTime,
                                      0,
                                      size,
                                      timeResolution,
                                      numberOfDivisions,
                                      color);
   }

   /**
    * Creates a new yoGraphic that represents the given polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolynomialFX3D.png"
    * width=150px/>
    * 
    * @param name          the name for yoGraphic.
    * @param coefficientsX the coefficients (from low to high order) for the x-axis polynomial.
    * @param coefficientsY the coefficients (from low to high order) for the y-axis polynomial.
    * @param coefficientsZ the coefficients (from low to high order) for the z-axis polynomial.
    * @param startTime     the initial time for the trajectory. The polynomial is rendered in the time
    *                      range <tt>t&in;[startTime, endTime]</tt>
    * @param endTime       the final time for the trajectory. The polynomial is rendered in the time
    *                      range <tt>t&in;[startTime, endTime]</tt>
    * @param size          the radius of the trajectory graphic.
    * @param color         the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoListDefinition coefficientsX,
                                                                          YoListDefinition coefficientsY,
                                                                          YoListDefinition coefficientsZ,
                                                                          YoDouble startTime,
                                                                          YoDouble endTime,
                                                                          double size,
                                                                          PaintDefinition color)
   {
      return newYoGraphicPolynomial3D(name, coefficientsX, coefficientsY, coefficientsZ, startTime, 0, endTime, 0, size, color);
   }

   /**
    * Creates a new yoGraphic that represents the given polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolynomialFX3D.png"
    * width=150px/>
    * 
    * @param name             the name for yoGraphic.
    * @param coefficientsX    the coefficients (from low to high order) for the x-axis polynomial.
    * @param coefficientsY    the coefficients (from low to high order) for the y-axis polynomial.
    * @param coefficientsZ    the coefficients (from low to high order) for the z-axis polynomial.
    * @param startTime        the initial time for the trajectory. The polynomial is rendered in the
    *                         time range <tt>t&in;[startTime, endTime]</tt>
    * @param defaultStartTime used if {@code startTime} is {@code null}.
    * @param endTime          the final time for the trajectory. The polynomial is rendered in the time
    *                         range <tt>t&in;[startTime, endTime]</tt>
    * @param defaultEndTime   used if {@code endTime} is {@code null}.
    * @param size             the radius of the trajectory graphic.
    * @param color            the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoListDefinition coefficientsX,
                                                                          YoListDefinition coefficientsY,
                                                                          YoListDefinition coefficientsZ,
                                                                          YoDouble startTime,
                                                                          double defaultStartTime,
                                                                          YoDouble endTime,
                                                                          double defaultEndTime,
                                                                          double size,
                                                                          PaintDefinition color)
   {
      return newYoGraphicPolynomial3D(name,
                                      coefficientsX,
                                      coefficientsY,
                                      coefficientsZ,
                                      startTime,
                                      defaultStartTime,
                                      endTime,
                                      defaultEndTime,
                                      size,
                                      50,
                                      10,
                                      color);
   }

   /**
    * Creates a new yoGraphic that represents the given polygon extruded: <br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolynomialFX3D.png"
    * width=150px/>
    * 
    * @param name              the name for yoGraphic.
    * @param coefficientsX     the coefficients (from low to high order) for the x-axis polynomial.
    * @param coefficientsY     the coefficients (from low to high order) for the y-axis polynomial.
    * @param coefficientsZ     the coefficients (from low to high order) for the z-axis polynomial.
    * @param startTime         the initial time for the trajectory. The polynomial is rendered in the
    *                          time range <tt>t&in;[startTime, endTime]</tt>
    * @param defaultStartTime  used if {@code startTime} is {@code null}.
    * @param endTime           the final time for the trajectory. The polynomial is rendered in the
    *                          time range <tt>t&in;[startTime, endTime]</tt>
    * @param defaultEndTime    used if {@code endTime} is {@code null}.
    * @param size              the radius of the trajectory graphic.
    * @param timeResolution    the number of divisions for the time.
    * @param numberOfDivisions the number of radial divisions.
    * @param color             the color.
    * @return the yoGraphic definition to submit to the SCS GUI.
    */
   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoListDefinition coefficientsX,
                                                                          YoListDefinition coefficientsY,
                                                                          YoListDefinition coefficientsZ,
                                                                          YoDouble startTime,
                                                                          double defaultStartTime,
                                                                          YoDouble endTime,
                                                                          double defaultEndTime,
                                                                          double size,
                                                                          int timeResolution,
                                                                          int numberOfDivisions,
                                                                          PaintDefinition color)
   {
      YoGraphicPolynomial3DDefinition definition = new YoGraphicPolynomial3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setCoefficientsX(coefficientsX);
      definition.setCoefficientsY(coefficientsY);
      definition.setCoefficientsZ(coefficientsZ);
      definition.setStartTime(toPropertyName(startTime, defaultStartTime));
      definition.setEndTime(toPropertyName(endTime, defaultEndTime));
      definition.setSize(size);
      definition.setTimeResolution(timeResolution);
      definition.setNumberOfDivisions(numberOfDivisions);
      definition.setColor(color);
      return definition;
   }

   /**
    * Creates a new tuple 2D definition that represents the given tuple.
    * <p>
    * The returned definition is backed by constant values and is assumed to be expressed in world.
    * </p>
    * 
    * @param tuple2D the tuple to create the definition for.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(Tuple2DReadOnly tuple2D)
   {
      return newYoTuple2DDefinition(tuple2D, null);
   }

   /**
    * Creates a new tuple 2D definition that represents the given tuple expressed in the given frame.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param tuple2D the tuple to create the definition for.
    * @param frame   the frame in which the tuple is expressed.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(Tuple2DReadOnly tuple2D, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(tuple2D == null ? 0 : tuple2D.getX(), tuple2D == null ? 0 : tuple2D.getY(), frame);
   }

   /**
    * Creates a new tuple 2D definition that represents the given frame tuple.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param frameTuple2D the tuple to create the definition for.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(FrameTuple2DReadOnly frameTuple2D)
   {
      return newYoTuple2DDefinition(frameTuple2D, frameTuple2D == null ? null : frameTuple2D.getReferenceFrame());
   }

   /**
    * Creates a new tuple 2D definition that represents the given tuple.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param tuple2D the tuple to create the definition for.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D)
   {
      return newYoTuple2DDefinition(tuple2D, null);
   }

   /**
    * Creates a new tuple 2D definition that represents the given tuple.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param tuple2D the tuple to create the definition for.
    * @param frame   the frame in which the tuple is expressed.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(tuple2D == null ? null : tuple2D.getYoX(), tuple2D == null ? null : tuple2D.getYoY(), frame);
   }

   /**
    * Creates a new tuple 2D definition that represents the given tuple.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param frameTuple2D the tuple to create the definition for.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoFrameTuple2D frameTuple2D)
   {
      return newYoTuple2DDefinition(frameTuple2D, frameTuple2D == null ? null : frameTuple2D.getReferenceFrame());
   }

   /**
    * Creates a new tuple 2D definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param yoVariables the yoVariables to get the tuple components from.
    * @param startIndex  the index of the first component in the array.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return newYoTuple2DDefinition(yoVariables[startIndex++], yoVariables[startIndex]);
   }

   /**
    * Creates a new tuple 2D definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param yoX the yoVariable for the x-component of the tuple.
    * @param yoY the yoVariable for the y-component of the tuple.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoVariable yoX, YoVariable yoY)
   {
      return newYoTuple2DDefinition(yoX, yoY, null);
   }

   /**
    * Creates a new tuple 2D definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param yoX   the yoVariable for the x-component of the tuple.
    * @param yoY   the yoVariable for the y-component of the tuple.
    * @param frame the frame in which the tuple is expressed.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoVariable yoX, YoVariable yoY, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(yoX, 0, yoY, 0, frame);
   }

   /**
    * Creates a new tuple 2D definition from the given constants.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param x     the value for the x-component.
    * @param y     the value for the y-component.
    * @param frame the frame in which the tuple is expressed.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(double x, double y, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(null, x, null, y, frame);
   }

   /**
    * Creates a new tuple 2D definition from the given components.
    * 
    * @param yoX      the yoVariable for the x-component of the tuple.
    * @param defaultX the default constant value if {@code yoX} is {@code null}.
    * @param yoY      the yoVariable for the y-component of the tuple.
    * @param defaultY the default constant value if {@code yoY} is {@code null}.
    * @param frame    the frame in which the tuple is expressed.
    * @return the tuple 2D definition.
    */
   public static YoTuple2DDefinition newYoTuple2DDefinition(YoVariable yoX, double defaultX, YoVariable yoY, double defaultY, ReferenceFrame frame)
   {
      YoTuple2DDefinition definition = new YoTuple2DDefinition();
      definition.setX(toPropertyName(yoX, defaultX));
      definition.setY(toPropertyName(yoY, defaultY));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   /**
    * Creates a new tuple 3D definition that represents the given tuple.
    * <p>
    * The returned definition is backed by constant values and is assumed to be expressed in world.
    * </p>
    * 
    * @param tuple3D the tuple to create the definition for.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(Tuple3DReadOnly tuple3D)
   {
      return newYoTuple3DDefinition(tuple3D, null);
   }

   /**
    * Creates a new tuple 3D definition that represents the given tuple expressed in the given frame.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param tuple3D the tuple to create the definition for.
    * @param frame   the frame in which the tuple is expressed.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(Tuple3DReadOnly tuple3D, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(tuple3D == null ? 0 : tuple3D.getX(), tuple3D == null ? 0 : tuple3D.getY(), tuple3D == null ? 0 : tuple3D.getZ(), frame);
   }

   /**
    * Creates a new tuple 3D definition that represents the given frame tuple.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param frameTuple3D the tuple to create the definition for.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(FrameTuple3DReadOnly frameTuple3D)
   {
      return newYoTuple3DDefinition(frameTuple3D, frameTuple3D == null ? null : frameTuple3D.getReferenceFrame());
   }

   /**
    * Creates a new tuple 3D definition that represents the given tuple.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param tuple3D the tuple to create the definition for.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D)
   {
      return newYoTuple3DDefinition(tuple3D, null);
   }

   /**
    * Creates a new tuple 3D definition that represents the given tuple.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param tuple3D the tuple to create the definition for.
    * @param frame   the frame in which the tuple is expressed.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(tuple3D == null ? null : tuple3D.getYoX(),
                                    tuple3D == null ? null : tuple3D.getYoY(),
                                    tuple3D == null ? null : tuple3D.getYoZ(),
                                    frame);
   }

   /**
    * Creates a new tuple 3D definition that represents the given tuple.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param frameTuple3D the tuple to create the definition for.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoFrameTuple3D frameTuple3D)
   {
      return newYoTuple3DDefinition(frameTuple3D, frameTuple3D == null ? null : frameTuple3D.getReferenceFrame());
   }

   /**
    * Creates a new tuple 3D definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param yoVariables the yoVariables to get the tuple components from.
    * @param startIndex  the index of the first component in the array.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return newYoTuple3DDefinition(yoVariables[startIndex++], yoVariables[startIndex++], yoVariables[startIndex]);
   }

   /**
    * Creates a new tuple 3D definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param yoX the yoVariable for the x-component of the tuple.
    * @param yoY the yoVariable for the y-component of the tuple.
    * @param yoZ the yoVariable for the z-component of the tuple.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoVariable yoX, YoVariable yoY, YoVariable yoZ)
   {
      return newYoTuple3DDefinition(yoX, yoY, yoZ, null);
   }

   /**
    * Creates a new tuple 3D definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param yoX   the yoVariable for the x-component of the tuple.
    * @param yoY   the yoVariable for the y-component of the tuple.
    * @param yoZ   the yoVariable for the z-component of the tuple.
    * @param frame the frame in which the tuple is expressed.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoVariable yoX, YoVariable yoY, YoVariable yoZ, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(yoX, 0.0, yoY, 0.0, yoZ, 0.0, frame);
   }

   /**
    * Creates a new tuple 3D definition from the given constants.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param x     the value for the x-component.
    * @param y     the value for the y-component.
    * @param z     the value for the z-component.
    * @param frame the frame in which the tuple is expressed.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(double x, double y, double z, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(null, x, null, y, null, z, frame);
   }

   /**
    * Creates a new tuple 3D definition from the given components.
    * 
    * @param yoX      the yoVariable for the x-component of the tuple.
    * @param defaultX the default constant value if {@code yoX} is {@code null}.
    * @param yoY      the yoVariable for the y-component of the tuple.
    * @param defaultY the default constant value if {@code yoY} is {@code null}.
    * @param yoZ      the yoVariable for the z-component of the tuple.
    * @param defaultZ the default constant value if {@code yoZ} is {@code null}.
    * @param frame    the frame in which the tuple is expressed.
    * @return the tuple 3D definition.
    */
   public static YoTuple3DDefinition newYoTuple3DDefinition(YoVariable yoX,
                                                            double defaultX,
                                                            YoVariable yoY,
                                                            double defaultY,
                                                            YoVariable yoZ,
                                                            double defaultZ,
                                                            ReferenceFrame frame)
   {
      YoTuple3DDefinition definition = new YoTuple3DDefinition();
      definition.setX(toPropertyName(yoX, defaultX));
      definition.setY(toPropertyName(yoY, defaultY));
      definition.setZ(toPropertyName(yoZ, defaultZ));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   /**
    * Creates a new orientation 3D definition from the given orientation.
    * <p>
    * The returned definition is backed by constant values and is assumed to be expressed in world.
    * </p>
    * 
    * @param orientation3D the orientation to create the definition for.
    * @return the orientation 3D definition.
    */
   public static YoOrientation3DDefinition newYoOrientation3DDefinition(Orientation3DReadOnly orientation3D)
   {
      return newYoOrientation3DDefinition(orientation3D, null);
   }

   /**
    * Creates a new orientation 3D definition from the given orientation.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param orientation3D the orientation to create the definition for.
    * @param frame         the frame in which the orientation is expressed.
    * @return the orientation 3D definition.
    */
   public static YoOrientation3DDefinition newYoOrientation3DDefinition(Orientation3DReadOnly orientation3D, ReferenceFrame frame)
   {
      if (orientation3D == null)
         return newYoYawPitchRollDefinition(null, frame);
      if (orientation3D instanceof QuaternionReadOnly)
         return newYoQuaternionDefinition((QuaternionReadOnly) orientation3D, frame);
      if (orientation3D instanceof YawPitchRollReadOnly)
         return newYoYawPitchRollDefinition((YawPitchRollReadOnly) orientation3D, frame);
      throw new UnsupportedOperationException("Orientation type [" + orientation3D.getClass().getSimpleName() + "] is not supported yet.");
   }

   /**
    * Creates a new orientation 3D definition from the given frame orientation.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param frameOrientation3D the orientation to create the definition for.
    * @return the orientation 3D definition.
    */
   public static YoOrientation3DDefinition newYoOrientation3DDefinition(FrameOrientation3DReadOnly frameOrientation3D)
   {
      return newYoOrientation3DDefinition(frameOrientation3D, frameOrientation3D == null ? null : frameOrientation3D.getReferenceFrame());
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given yaw-pitch-roll.
    * <p>
    * The returned definition is backed by constant values and is assumed to be expressed in world.
    * </p>
    * 
    * @param yawPitchRoll the yaw-pitch-roll to create the definition for.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YawPitchRollReadOnly yawPitchRoll)
   {
      return newYoYawPitchRollDefinition(yawPitchRoll, null);
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given yaw-pitch-roll.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param yawPitchRoll the yaw-pitch-roll to create the definition for.
    * @param frame        the frame in which the orientation is expressed.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YawPitchRollReadOnly yawPitchRoll, ReferenceFrame frame)
   {
      return newYoYawPitchRollDefinition(yawPitchRoll == null ? 0 : yawPitchRoll.getYaw(),
                                         yawPitchRoll == null ? 0 : yawPitchRoll.getPitch(),
                                         yawPitchRoll == null ? 0 : yawPitchRoll.getRoll(),
                                         frame);
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given frame yaw-pitch-roll.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param frameYawPitchRoll the yaw-pitch-roll to create the definition for.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(FrameYawPitchRollReadOnly frameYawPitchRoll)
   {
      return newYoYawPitchRollDefinition(frameYawPitchRoll, frameYawPitchRoll == null ? null : frameYawPitchRoll.getReferenceFrame());
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given frame yaw-pitch-roll.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param frameYawPitchRoll the yaw-pitch-roll to create the definition for.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoFrameYawPitchRoll frameYawPitchRoll)
   {
      return newYoYawPitchRollDefinition(frameYawPitchRoll == null ? null : frameYawPitchRoll.getYoYaw(),
                                         frameYawPitchRoll == null ? null : frameYawPitchRoll.getYoPitch(),
                                         frameYawPitchRoll == null ? null : frameYawPitchRoll.getYoRoll(),
                                         frameYawPitchRoll == null ? null : frameYawPitchRoll.getReferenceFrame());
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the tuple is assumed
    * to be expressed in world.
    * </p>
    * 
    * @param yoVariables the yoVariables to get the yaw-pitch-roll components from.
    * @param startIndex  the index of the first component in the array.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return newYoYawPitchRollDefinition(yoVariables[startIndex++], yoVariables[startIndex++], yoVariables[startIndex]);
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the orientation is
    * assumed to be expressed in world.
    * </p>
    * 
    * @param yoYaw   the yoVariable for the yaw-component of the yaw-pitch-roll.
    * @param yoPitch the yoVariable for the pitch-component of the yaw-pitch-roll.
    * @param yoRoll  the yoVariable for the roll-component of the yaw-pitch-roll.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoVariable yoYaw, YoVariable yoPitch, YoVariable yoRoll)
   {
      return newYoYawPitchRollDefinition(yoYaw, yoPitch, yoRoll, null);
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the orientation is
    * assumed to be expressed in world.
    * </p>
    * 
    * @param yoYaw   the yoVariable for the yaw-component of the yaw-pitch-roll.
    * @param yoPitch the yoVariable for the pitch-component of the yaw-pitch-roll.
    * @param yoRoll  the yoVariable for the roll-component of the yaw-pitch-roll.
    * @param frame   the frame in which the yaw-pitch-roll is expressed.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoVariable yoYaw, YoVariable yoPitch, YoVariable yoRoll, ReferenceFrame frame)
   {
      return newYoYawPitchRollDefinition(yoYaw, 0.0, yoPitch, 0.0, yoRoll, 0.0, frame);
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given constants.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param yaw   the value for the yaw-component.
    * @param pitch the value for the pitch-component.
    * @param roll  the value for the roll-component.
    * @param frame the frame in which the yaw-pitch-roll is expressed.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(double yaw, double pitch, double roll, ReferenceFrame frame)
   {
      return newYoYawPitchRollDefinition(null, yaw, null, pitch, null, roll, frame);
   }

   /**
    * Creates a new yaw-pitch-roll definition from the given components.
    * 
    * @param yoYaw        the yoVariable for the yaw-component of the yaw-pitch-roll.
    * @param defaultYaw   the default constant value if {@code yoYaw} is {@code null}.
    * @param yoPitch      the yoVariable for the pitch-component of the yaw-pitch-roll.
    * @param defaultPitch the default constant value if {@code yoPitch} is {@code null}.
    * @param yoRoll       the yoVariable for the roll-component of the yaw-pitch-roll.
    * @param defaultRoll  the default constant value if {@code yoRoll} is {@code null}.
    * @param frame        the frame in which the yaw-pitch-roll is expressed.
    * @return the yaw-pitch-roll definition.
    */
   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoVariable yoYaw,
                                                                      double defaultYaw,
                                                                      YoVariable yoPitch,
                                                                      double defaultPitch,
                                                                      YoVariable yoRoll,
                                                                      double defaultRoll,
                                                                      ReferenceFrame frame)
   {
      YoYawPitchRollDefinition definition = new YoYawPitchRollDefinition();
      definition.setYaw(toPropertyName(yoYaw, defaultYaw));
      definition.setPitch(toPropertyName(yoPitch, defaultPitch));
      definition.setRoll(toPropertyName(yoRoll, defaultRoll));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   /**
    * Creates a new quaternion definition that represents the given quaternion.
    * <p>
    * The returned definition is backed by constant values and is assumed to be expressed in world.
    * </p>
    * 
    * @param quaternion the tuple to create the definition for.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(QuaternionReadOnly quaternion)
   {
      return newYoQuaternionDefinition(quaternion, null);
   }

   /**
    * Creates a new quaternion definition that represents the given quaternion expressed in the given
    * frame.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param quaternion the tuple to create the definition for.
    * @param frame      the frame in which the quaternion is expressed.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(QuaternionReadOnly quaternion, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(quaternion == null ? null : quaternion.getX(),
                                       quaternion == null ? null : quaternion.getY(),
                                       quaternion == null ? null : quaternion.getZ(),
                                       quaternion == null ? null : quaternion.getS(),
                                       frame);
   }

   /**
    * Creates a new quaternion definition that represents the given frame quaternion.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param frameQuaternion the quaternion to create the definition for.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(FrameQuaternionReadOnly frameQuaternion)
   {
      return newYoQuaternionDefinition(frameQuaternion, frameQuaternion == null ? null : frameQuaternion.getReferenceFrame());
   }

   /**
    * Creates a new quaternion definition that represents the given quaternion.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the quaternion is
    * assumed to be expressed in world.
    * </p>
    * 
    * @param quaternion the quaternion to create the definition for.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoQuaternion quaternion)
   {
      return newYoQuaternionDefinition(quaternion, null);
   }

   /**
    * Creates a new quaternion definition that represents the given quaternion.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param quaternion the quaternion to create the definition for.
    * @param frame      the frame in which the quaternion is expressed.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoQuaternion quaternion, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(quaternion == null ? null : quaternion.getYoQx(),
                                       quaternion == null ? null : quaternion.getYoQy(),
                                       quaternion == null ? null : quaternion.getYoQz(),
                                       quaternion == null ? null : quaternion.getYoQs(),
                                       frame);
   }

   /**
    * Creates a new quaternion definition that represents the given quaternion.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param frameQuaternion the quaternion to create the definition for.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoFrameQuaternion frameQuaternion)
   {
      return newYoQuaternionDefinition(frameQuaternion, frameQuaternion == null ? null : frameQuaternion.getReferenceFrame());
   }

   /**
    * Creates a new quaternion definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the quaternion is
    * assumed to be expressed in world.
    * </p>
    * 
    * @param yoVariables the yoVariables to get the quaternion components from. Components are expected
    *                    to be stored in order [x, y, z, s].
    * @param startIndex  the index of the first component in the array.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return newYoQuaternionDefinition(yoVariables[startIndex++], yoVariables[startIndex++], yoVariables[startIndex++], yoVariables[startIndex]);
   }

   /**
    * Creates a new quaternion definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used and that the quaternion is
    * assumed to be expressed in world.
    * </p>
    * 
    * @param yoX the yoVariable for the x-component of the quaternion.
    * @param yoY the yoVariable for the y-component of the quaternion.
    * @param yoZ the yoVariable for the z-component of the quaternion.
    * @param yoS the yoVariable for the s-component of the quaternion.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoVariable yoX, YoVariable yoY, YoVariable yoZ, YoVariable yoS)
   {
      return newYoQuaternionDefinition(yoX, yoY, yoZ, yoS, null);
   }

   /**
    * Creates a new quaternion definition from the given yoVariables.
    * <p>
    * The returned definition indicates that yoVariables should be used.
    * </p>
    * 
    * @param yoX   the yoVariable for the x-component of the quaternion.
    * @param yoY   the yoVariable for the y-component of the quaternion.
    * @param yoZ   the yoVariable for the z-component of the quaternion.
    * @param yoS   the yoVariable for the s-component of the quaternion.
    * @param frame the frame in which the quaternion is expressed.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoVariable yoX, YoVariable yoY, YoVariable yoZ, YoVariable yoS, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(yoX, 0, yoY, 0, yoZ, 0, yoS, 0, frame);
   }

   /**
    * Creates a new quaternion definition from the given constants.
    * <p>
    * The returned definition is backed by constant values.
    * </p>
    * 
    * @param x     the value for the x-component.
    * @param y     the value for the y-component.
    * @param z     the value for the z-component.
    * @param s     the value for the s-component.
    * @param frame the frame in which the quaternion is expressed.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(double x, double y, double z, double s, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(null, x, null, y, null, z, null, s, frame);
   }

   /**
    * Creates a new quaternion definition from the given components.
    * 
    * @param yoX      the yoVariable for the x-component of the quaternion.
    * @param defaultX the default constant value if {@code yoX} is {@code null}.
    * @param yoY      the yoVariable for the y-component of the quaternion.
    * @param defaultY the default constant value if {@code yoY} is {@code null}.
    * @param yoZ      the yoVariable for the z-component of the quaternion.
    * @param defaultZ the default constant value if {@code yoZ} is {@code null}.
    * @param yoS      the yoVariable for the s-component of the quaternion.
    * @param defaultS the default constant value if {@code yoS} is {@code null}.
    * @param frame    the frame in which the quaternion is expressed.
    * @return the quaternion definition.
    */
   public static YoQuaternionDefinition newYoQuaternionDefinition(YoVariable yoX,
                                                                  double defaultX,
                                                                  YoVariable yoY,
                                                                  double defaultY,
                                                                  YoVariable yoZ,
                                                                  double defaultZ,
                                                                  YoVariable yoS,
                                                                  double defaultS,
                                                                  ReferenceFrame frame)
   {
      YoQuaternionDefinition definition = new YoQuaternionDefinition();
      definition.setX(toPropertyName(yoX, defaultX));
      definition.setY(toPropertyName(yoY, defaultY));
      definition.setZ(toPropertyName(yoZ, defaultZ));
      definition.setS(toPropertyName(yoS, defaultS));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   /**
    * Creates a new paint definition from the given yoVariables.
    * 
    * @param red   the yoVariable for the red component, expected to be in [0.0-1.0].
    * @param green the yoVariable for the green component, expected to be in [0.0-1.0].
    * @param blue  the yoVariable for the blue component, expected to be in [0.0-1.0].
    * @return the paint definition.
    */
   public static PaintDefinition toYoColorDefinition(YoDouble red, YoDouble green, YoDouble blue)
   {
      return toYoColorDefinition(red, green, blue, null);
   }

   /**
    * Creates a new paint definition from the given yoVariables.
    * 
    * @param red   the yoVariable for the red component, expected to be in [0.0-1.0].
    * @param green the yoVariable for the green component, expected to be in [0.0-1.0].
    * @param blue  the yoVariable for the blue component, expected to be in [0.0-1.0].
    * @param alpha the yoVariable for the alpha (opacity) component, expected to be in [0.0-1.0].
    * @return the paint definition.
    */
   public static PaintDefinition toYoColorDefinition(YoDouble red, YoDouble green, YoDouble blue, YoDouble alpha)
   {
      return new YoColorRGBADoubleDefinition(toPropertyName(red, 0.0), toPropertyName(green, 0.0), toPropertyName(blue, 0.0), toPropertyName(alpha, 1.0));
   }

   /**
    * Creates a new paint definition from the given yoVariables.
    * 
    * @param red   the yoVariable for the red component, expected to be in [0-255].
    * @param green the yoVariable for the green component, expected to be in [0-255].
    * @param blue  the yoVariable for the blue component, expected to be in [0-255].
    * @return the paint definition.
    */
   public static PaintDefinition toYoColorDefinition(YoInteger red, YoInteger green, YoInteger blue)
   {
      return toYoColorDefinition(red, green, blue, null);
   }

   /**
    * Creates a new paint definition from the given yoVariables.
    * 
    * @param red   the yoVariable for the red component, expected to be in [0-255].
    * @param green the yoVariable for the green component, expected to be in [0-255].
    * @param blue  the yoVariable for the blue component, expected to be in [0-255].
    * @param alpha the yoVariable for the alpha (opacity) component, expected to be in [0-255].
    * @return the paint definition.
    */
   public static PaintDefinition toYoColorDefinition(YoInteger red, YoInteger green, YoInteger blue, YoInteger alpha)
   {
      return new YoColorRGBAIntDefinition(toPropertyName(red, 0), toPropertyName(green, 0), toPropertyName(blue, 0), toPropertyName(alpha, 255));
   }

   /**
    * Creates a new paint definition from the given yoVariable.
    * 
    * @param rgba the yoVariable for the rgba component.
    * @return the paint definition.
    * @see ColorDefinition#toRGBA()
    * @see ColorDefinitions#rgba(int)
    */
   public static PaintDefinition toYoColorDefinition(YoInteger rgba)
   {
      return new YoColorRGBASingleDefinition(toPropertyName(rgba, 0));
   }

   /**
    * Creates a new list definition from the given yoVariables.
    * 
    * @param yoVariables the yoVariables composing the list.
    * @param size        the size of the active part of the list.
    * @return the list definition.
    */
   public static YoListDefinition toYoListDefinition(YoVariable[] yoVariables, YoInteger size)
   {
      return toYoListDefinition(yoVariables, null, size);
   }

   /**
    * Creates a new list definition from the given yoVariables.
    * 
    * @param yoVariables   the yoVariables composing the list.
    * @param defaultValues the default constant values to use if {@code yoVariables} is {@code null}.
    * @param size          the size of the active part of the list.
    * @return the list definition.
    */
   public static YoListDefinition toYoListDefinition(YoVariable[] yoVariables, double[] defaultValues, YoInteger size)
   {
      return new YoListDefinition(toPropertyNames(yoVariables, defaultValues), toPropertyName(size));
   }

   /**
    * Extracts the fullname of the yoVariables and returned them as a list.
    * 
    * @param yoVariables the yoVariables to get the fullname of.
    * @return the list of fullnames.
    */
   public static List<String> toPropertyNames(YoVariable[] yoVariables)
   {
      return toPropertyNames(yoVariables, null);
   }

   /**
    * Extracts the fullname of the yoVariables and returned them as a list.
    * 
    * @param yoVariables   the yoVariables to get the fullname of.
    * @param defaultValues the default constant values to use if {@code yoVariables} is {@code null}.
    * @return the list of fullnames.
    */
   public static List<String> toPropertyNames(YoVariable[] yoVariables, double[] defaultValues)
   {
      List<String> propertyNames = new ArrayList<>();

      if (yoVariables == null)
      {
         for (int i = 0; i < defaultValues.length; i++)
         {
            propertyNames.add(Double.toString(defaultValues[i]));
         }
      }
      else if (defaultValues == null)
      {
         for (int i = 0; i < yoVariables.length; i++)
         {
            propertyNames.add(toPropertyName(yoVariables[i]));
         }
      }
      else
      {
         for (int i = 0; i < yoVariables.length; i++)
         {
            propertyNames.add(toPropertyName(yoVariables[i], defaultValues[i]));
         }
      }

      return propertyNames;
   }

   /**
    * Extracts and returns the name id from the given frame.
    * 
    * @param referenceFrame the reference frame to get the frame id of.
    * @return the name id.
    */
   public static String toPropertyName(ReferenceFrame referenceFrame)
   {
      return referenceFrame == null ? null : referenceFrame.getNameId();
   }

   /**
    * Extracts and returns the fullname of the given yoVariable.
    * 
    * @param yoVariable the yoVariable to get the fullname of.
    * @return the fullname.
    */
   public static String toPropertyName(YoVariable yoVariable)
   {
      return yoVariable == null ? null : yoVariable.getFullNameString();
   }

   /**
    * Extracts and returns the fullname of the given yoVariable.
    * 
    * @param yoVariable   the yoVariable to get the fullname of.
    * @param defaultValue the default constant value to return if {@code yoVariable} is {@code null}.
    * @return the fullname or constant.
    */
   public static String toPropertyName(YoVariable yoVariable, double defaultValue)
   {
      return yoVariable == null ? Double.toString(defaultValue) : yoVariable.getFullNameString();
   }

   /**
    * Extracts and returns the fullname of the given yoVariable.
    * 
    * @param yoVariable   the yoVariable to get the fullname of.
    * @param defaultValue the default constant value to return if {@code yoVariable} is {@code null}.
    * @return the fullname or constant.
    */
   public static String toPropertyName(YoInteger yoInteger, int defaultValue)
   {
      return yoInteger == null ? Integer.toString(defaultValue) : yoInteger.getFullNameString();
   }
}
