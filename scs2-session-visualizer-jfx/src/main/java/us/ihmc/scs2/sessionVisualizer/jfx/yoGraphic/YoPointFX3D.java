package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

import java.util.Arrays;
import java.util.List;

public class YoPointFX3D extends YoGraphicFX3D
{
   private final Group pointNode = new Group();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private DoubleProperty size = new SimpleDoubleProperty(0.1);
   private final Translate translate = new Translate();
   private final Scale scale = new Scale();
   private final PhongMaterial material = new PhongMaterial();
   private YoGraphicFXResource graphicResource;

   public YoPointFX3D()
   {
      pointNode.getTransforms().addAll(translate, scale);
      setGraphicResource(YoGraphicFXResourceManager.DEFAULT_POINT3D_GRAPHIC_RESOURCE);
      pointNode.idProperty().bind(nameProperty());
      pointNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoPointFX3D(ReferenceFrameWrapper worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
   }

   public void setGraphicResource(YoGraphicFXResource graphicResource)
   {
      if (graphicResource == null || graphicResource.getResourceURL() == null)
         return;

      this.graphicResource = graphicResource;
      pointNode.getChildren().clear();

      Node[] nodes = JavaFXVisualTools.importModel(graphicResource.getResourceURL());

      List<Shape3D> shapes = YoGraphicTools.extractShape3Ds(Arrays.asList(nodes));

      for (Shape3D shape : shapes)
      {
         shape.setMaterial(material);
         shape.idProperty().bind(nameProperty());
      }

      pointNode.getChildren().addAll(nodes);
   }

   @Override
   public void render()
   {
      if (position.containsNaN() || (size != null && Double.isNaN(size.get())))
      {
         scale.setX(0);
         scale.setY(0);
         scale.setZ(0);
         return;
      }

      Point3D positionInWorld = position.toPoint3DInWorld();
      translate.setX(positionInWorld.getX());
      translate.setY(positionInWorld.getY());
      translate.setZ(positionInWorld.getZ());
      if (size == null)
         size = new SimpleDoubleProperty(0.1);
      scale.setX(size.get());
      scale.setY(size.get());
      scale.setZ(size.get());
      material.setDiffuseColor(color.get());
   }

   public void setPosition(Tuple3DProperty position)
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
      color = null;
   }

   @Override
   public YoPointFX3D clone()
   {
      YoPointFX3D clone = new YoPointFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setSize(size);
      clone.setColor(color);
      return clone;
   }

   public Tuple3DProperty getPosition()
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
