package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class YoPointcloudFX3D extends YoGraphicFX3D
{
   private final Group pointcloudNode = new Group();

   private List<Tuple3DProperty> points = new ArrayList<>();
   private IntegerProperty numberOfPoints = null;
   private DoubleProperty size = new SimpleDoubleProperty(0.01);
   private YoGraphicFXResource graphicResource = YoGraphicFXResourceManager.DEFAULT_POINT3D_GRAPHIC_RESOURCE;

   private final PhongMaterial material = new PhongMaterial(Color.BLUE);
   private final Scale scale = new Scale();
   private final List<Translate> translates = new ArrayList<>();
   private final IntegerProperty numberOfPointsProperty = new SimpleIntegerProperty(this, "numberOfPoints", 0);

   private final BooleanProperty refreshGraphicsProperty = new SimpleBooleanProperty(this, "refreshGraphics", true);

   public YoPointcloudFX3D()
   {
      drawModeProperty.addListener((o, oldValue, newValue) ->
                                   {
                                      if (newValue == null)
                                         drawModeProperty.setValue(DrawMode.FILL);
                                      JavaFXMissingTools.setDrawModeRecursive(pointcloudNode, newValue);
                                   });
      numberOfPointsProperty.addListener((observable, oldValue, newValue) -> refreshGraphicsProperty.set(true));
      pointcloudNode.idProperty().bind(nameProperty());
      pointcloudNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoPointcloudFX3D(ReferenceFrame worldFrame)
   {
      this();
   }

   @Override
   public void render()
   {
      if (size != null)
      {
         scale.setX(size.get());
         scale.setY(size.get());
         scale.setZ(size.get());
      }

      if (color != null)
         material.setDiffuseColor(color.get());

      if (graphicResource == null || graphicResource.getResourceURL() == null)
         return;

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
         Translate translate = translates.get(i);

         if (points.get(i).containsNaN())
         {
            translate.setX(Double.NEGATIVE_INFINITY);
            translate.setY(Double.NEGATIVE_INFINITY);
            translate.setZ(Double.NEGATIVE_INFINITY);
         }
         else
         {
            Point3D position = points.get(i).toPoint3DInWorld();
            translate.setX(position.getX());
            translate.setY(position.getY());
            translate.setZ(position.getZ());
         }
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

         Node[] nodes = JavaFXVisualTools.importModel(graphicResource.getResourceURL());

         List<Shape3D> shapes = YoGraphicTools.extractShape3Ds(Arrays.asList(nodes));

         DrawMode drawMode = getDrawMode() == null ? DrawMode.FILL : getDrawMode();
         for (Shape3D shape : shapes)
         {
            shape.setMaterial(material);
            shape.setDrawMode(drawMode);
            // The importer may have added transforms, we want to be before these.
            shape.getTransforms().addAll(0, Arrays.asList(translate, scale));
            shape.idProperty().bind(nameProperty().concat(" (").concat(Integer.toString(i)).concat(")"));
         }

         pointcloudNode.getChildren().addAll(shapes);
      }
   }

   public void setPoints(List<Tuple3DProperty> points)
   {
      this.points = points;
   }

   public void addPoint(DoubleProperty x, DoubleProperty y, DoubleProperty z)
   {
      addPoint(new Tuple3DProperty(null, x, y, z));
   }

   public void addPoint(Tuple3DProperty point)
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

   public void setGraphicResource(YoGraphicFXResource graphicResouce)
   {
      this.graphicResource = graphicResouce;
      refreshGraphicsProperty.set(true);
   }

   @Override
   public void clear()
   {
      points = null;
      numberOfPoints = null;
      size = null;
      graphicResource = null;
      color = null;
   }

   @Override
   public YoGraphicFX clone()
   {
      YoPointcloudFX3D clone = new YoPointcloudFX3D();
      clone.setName(getName());
      clone.setPoints(new ArrayList<>(points));
      clone.setNumberOfPoints(numberOfPoints);
      clone.setSize(size);
      clone.setGraphicResource(graphicResource);
      clone.setColor(color);
      return clone;
   }

   public List<Tuple3DProperty> getPoints()
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
