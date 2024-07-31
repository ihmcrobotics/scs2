package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.geometry.Bounds;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Shape;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import net.javainthebox.caraibe.svg.SVGContent;
import net.javainthebox.caraibe.svg.SVGLoader;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;

import java.util.List;

public class YoPointFX2D extends YoGraphicFX2D
{
   private final Group pointNode = new Group();

   private Tuple2DProperty position = new Tuple2DProperty(null, 0.0, 0.0);
   private DoubleProperty size = new SimpleDoubleProperty(0.1);
   private final Translate translate = new Translate();
   private final Scale scale = new Scale();
   private final ObjectProperty<Paint> fillProperty = new SimpleObjectProperty<>(this, "fillProperty", null);
   private final ObjectProperty<Paint> strokeProperty = new SimpleObjectProperty<>(this, "strokeProperty", Color.BLACK);
   private final DoubleProperty strokeWidthProperty = new SimpleDoubleProperty(this, "strokeWidthProperty", 0.0);
   private YoGraphicFXResource graphicResource;

   public YoPointFX2D()
   {
      pointNode.getTransforms().addAll(translate, scale);
      pointNode.idProperty().bind(nameProperty());
      pointNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
      setGraphicResource(YoGraphicFXResourceManager.DEFAULT_POINT2D_GRAPHIC_RESOURCE);
   }

   public YoPointFX2D(ReferenceFrameWrapper worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
   }

   private SVGContent graphic = null;

   public void setGraphicResource(YoGraphicFXResource graphicResource)
   {
      this.graphicResource = graphicResource;
      pointNode.getChildren().clear();

      if (graphicResource == null || graphicResource.getResourceURL() == null)
         return;

      graphic = SVGLoader.load(graphicResource.getResourceURL());

      List<Shape> shapes = YoGraphicTools.extractShapes(graphic);
      Translate graphicCentering = new Translate();
      Bounds layoutBounds = graphic.getLayoutBounds();
      graphicCentering.setX(-0.5 * (layoutBounds.getMinX() + layoutBounds.getMaxX()));
      graphicCentering.setY(-0.5 * (layoutBounds.getMinY() + layoutBounds.getMaxY()));

      for (Shape shape : shapes)
      {
         shape.getTransforms().add(graphicCentering);
         shape.fillProperty().bind(fillProperty);
         shape.strokeProperty().bind(strokeProperty);
         shape.strokeWidthProperty().bind(strokeWidthProperty);
         shape.idProperty().bind(nameProperty());
      }
   }

   @Override
   public void render()
   {
      // If the SVG is a path, it seems that hiding it by setting the translation to a large number and setting the scale to 0 may result in the following exception:
      // java.lang.IllegalStateException: Trying to remove a cached item that's not in the cache
      // Removing and adding the graphic back to the pointNode group seems to avoid that issue.
      if (position.containsNaN())
      {
         pointNode.getChildren().clear();
         //         translate.setX(Double.NEGATIVE_INFINITY);
         //         translate.setY(Double.NEGATIVE_INFINITY);
         //         scale.setX(0.0);
         //         scale.setY(0.0);
         return;
      }
      else
      {
         if (pointNode.getChildren().isEmpty())
            pointNode.getChildren().add(graphic);
         else if (pointNode.getChildren().get(0) != graphic)
            pointNode.getChildren().set(0, graphic);
      }

      Point2D positionInWorld = position.toPoint2DInWorld();
      translate.setX(positionInWorld.getX());
      translate.setY(positionInWorld.getY());
      scale.setX(YoGraphicFXResourceManager.SVG_SCALE * size.get());
      scale.setY(YoGraphicFXResourceManager.SVG_SCALE * size.get());

      fillProperty.set(fillColor == null ? null : fillColor.get());
      strokeProperty.set(strokeColor == null ? null : strokeColor.get());

      if (strokeWidth == null)
         strokeWidth = YoGraphicFX2D.DEFAULT_STROKE_WIDTH;

      double scale = pointNode.getLocalToSceneTransform().deltaTransform(1.0, 0.0).getX();
      strokeWidthProperty.set(strokeWidth.get() / scale);
   }

   public void setPosition(Tuple2DProperty position)
   {
      this.position = position;
   }

   public void setSize(DoubleProperty size)
   {
      this.size = size;
   }

   public void setSize(double size)
   {
      this.size = new SimpleDoubleProperty(size);
   }

   @Override
   public void clear()
   {
      position = null;
      size = null;
      fillColor = null;
      strokeColor = null;
      strokeWidth = null;
      graphicResource = null;
   }

   @Override
   public YoPointFX2D clone()
   {
      YoPointFX2D clone = new YoPointFX2D();
      clone.setName(getName());
      clone.setPosition(new Tuple2DProperty(position));
      clone.setSize(size);
      clone.setFillColor(fillColor);
      clone.setStrokeColor(strokeColor);
      clone.setStrokeWidth(strokeWidth);
      clone.setGraphicResource(graphicResource);
      return clone;
   }

   public Tuple2DProperty getPosition()
   {
      return position;
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
      return pointNode;
   }
}
