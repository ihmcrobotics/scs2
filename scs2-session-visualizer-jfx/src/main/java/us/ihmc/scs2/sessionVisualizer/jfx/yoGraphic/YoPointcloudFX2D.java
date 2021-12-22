package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Shape;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import net.javainthebox.caraibe.svg.SVGContent;
import net.javainthebox.caraibe.svg.SVGLoader;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;

public class YoPointcloudFX2D extends YoGraphicFX2D
{
   private final Group pointcloudNode = new Group();

   private List<Tuple2DProperty> points = new ArrayList<>();
   private IntegerProperty numberOfPoints = null;
   private DoubleProperty size = new SimpleDoubleProperty(0.01);
   private YoGraphicFXResource graphicResource = YoGraphicFXResourceManager.DEFAULT_POINT2D_GRAPHIC_RESOURCE;

   private final Scale scale = new Scale();
   private final List<Translate> translates = new ArrayList<>();
   private final IntegerProperty numberOfPointsProperty = new SimpleIntegerProperty(this, "numberOfPoints", 0);
   private final ObjectProperty<Paint> fillProperty = new SimpleObjectProperty<>(this, "fillProperty", null);
   private final ObjectProperty<Paint> strokeProperty = new SimpleObjectProperty<>(this, "strokeProperty", Color.BLACK);
   private final DoubleProperty strokeWidthProperty = new SimpleDoubleProperty(this, "strokeWidthProperty", 0.0);

   private final BooleanProperty refreshGraphicsProperty = new SimpleBooleanProperty(this, "refreshGraphics", true);

   public YoPointcloudFX2D()
   {
      numberOfPointsProperty.addListener((observable, oldValue, newValue) -> refreshGraphicsProperty.set(true));
   }

   public YoPointcloudFX2D(ReferenceFrame worldFrame)
   {
      this();
   }

   @Override
   public void render()
   {
      scale.setX(YoGraphicFXResourceManager.SVG_SCALE * size.get());
      scale.setY(YoGraphicFXResourceManager.SVG_SCALE * size.get());

      fillProperty.set(fillColor == null ? null : fillColor.get());
      strokeProperty.set(strokeColor == null ? null : strokeColor.get());

      if (strokeWidth == null)
         strokeWidth = YoGraphicFX2D.DEFAULT_STROKE_WIDTH;

      double scale = YoGraphicFXResourceManager.SVG_SCALE * size.get() * pointcloudNode.getLocalToSceneTransform().deltaTransform(1.0, 0.0).getX();
      strokeWidthProperty.set(strokeWidth.get() / scale);

      if (points == null)
         numberOfPointsProperty.set(0);
      else if (numberOfPoints != null)
         numberOfPointsProperty.set(Math.min(numberOfPoints.get(), points.size()));
      else
         numberOfPointsProperty.set(points.size());

      if (refreshGraphicsProperty.get())
      {
         refreshGraphicsProperty.set(false);
         refreshGraphics();
      }

      for (int i = 0; i < numberOfPointsProperty.get(); i++)
      {
         Point2D position = points.get(i).toPoint2DInWorld();
         Translate translate = translates.get(i);
         translate.setX(position.getX());
         translate.setY(position.getY());
      }
   }

   private void refreshGraphics()
   {
      pointcloudNode.getChildren().clear();
      translates.clear();

      if (numberOfPointsProperty.get() == 0 || graphicResource == null || graphicResource.getResourceURL() == null)
         return;

      for (int i = 0; i < numberOfPointsProperty.get(); i++)
      {
         Translate translate = new Translate();
         translates.add(translate);

         SVGContent graphic = SVGLoader.load(graphicResource.getResourceURL());
         List<Shape> shapes = YoGraphicTools.extractShapes(graphic);

         for (Shape shape : shapes)
         {
            shape.fillProperty().bind(fillProperty);
            shape.strokeProperty().bind(strokeProperty);
            shape.strokeWidthProperty().bind(strokeWidthProperty);
            shape.idProperty().bind(nameProperty().concat(" (").concat(Integer.toString(i)).concat(")"));
         }

         graphic.getTransforms().addAll(translate, scale);
         pointcloudNode.getChildren().add(graphic);
      }
   }

   public void setPoints(List<Tuple2DProperty> points)
   {
      this.points = points;
   }

   public void addPoint(DoubleProperty x, DoubleProperty y)
   {
      addPoint(new Tuple2DProperty(null, x, y));
   }

   public void addPoint(Tuple2DProperty point)
   {
      points.add(point);
   }

   public void setNumberOfPoints(IntegerProperty numberOfPoints)
   {
      this.numberOfPoints = numberOfPoints;
   }

   public void setNumberOfPoints(int numberOfPoints)
   {
      this.numberOfPoints = new SimpleIntegerProperty(numberOfPoints);
   }

   public void setSize(DoubleProperty size)
   {
      this.size = size;
   }

   public void setSize(double size)
   {
      this.size = new SimpleDoubleProperty(size);
   }

   public void setGraphicResource(YoGraphicFXResource graphicResource)
   {
      this.graphicResource = graphicResource;
      refreshGraphicsProperty.set(true);
   }

   @Override
   public void clear()
   {
      pointcloudNode.getChildren().clear();
      points.clear();
      points = null;
      numberOfPoints = null;
      size = null;
      graphicResource = null;
      translates.clear();
   }

   @Override
   public YoGraphicFX clone()
   {
      YoPointcloudFX2D clone = new YoPointcloudFX2D();
      clone.setName(getName());
      clone.setPoints(new ArrayList<>(points));
      clone.setNumberOfPoints(numberOfPoints);
      clone.setSize(size);
      clone.setFillColor(fillColor);
      clone.setStrokeColor(strokeColor);
      clone.setStrokeWidth(strokeWidth);
      clone.setGraphicResource(graphicResource);
      return clone;
   }

   public List<Tuple2DProperty> getPoints()
   {
      return points;
   }

   public IntegerProperty getNumberOfPoints()
   {
      return numberOfPoints;
   }

   public DoubleProperty getSize()
   {
      return size;
   }

   public YoGraphicFXResource getGraphicResource()
   {
      return graphicResource;
   }

   @Override
   public Node getNode()
   {
      return pointcloudNode;
   }
}
