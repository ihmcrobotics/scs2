package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class YoPointFX3D extends YoGraphicFX3D
{
   private final Group pointNode = new Group();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private DoubleProperty size = new SimpleDoubleProperty(0.1);
   private final Translate translate = new Translate();
   private final Scale scale = new Scale();
   private final PhongMaterial material = new PhongMaterial();
   private YoGraphicFXResource graphicResource;

   /**
    * Last values actually written to {@link #translate}/{@link #scale}/{@link #material}, so
    * {@link #render()} can skip the JavaFX property writes when nothing changed since the last frame --
    * writing a property still invalidates/dirties the node even when the new value equals the old one, and
    * this method is called on every JavaFX pulse (up to 60Hz) for every plotted point graphic. {@code NaN}
    * forces the first frame (and the frame right after the NaN-guard branch below) to always write.
    */
   private double lastTranslateX = Double.NaN, lastTranslateY = Double.NaN, lastTranslateZ = Double.NaN;
   private double lastScale = Double.NaN;
   private Color lastColor = null;

   public YoPointFX3D()
   {
      drawModeProperty.addListener((o, oldValue, newValue) ->
                                   {
                                      if (newValue == null)
                                         drawModeProperty.setValue(DrawMode.FILL);
                                      JavaFXMissingTools.setDrawModeRecursive(pointNode, newValue);
                                   });
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

      DrawMode drawMode = getDrawMode() == null ? DrawMode.FILL : getDrawMode();

      for (Shape3D shape : shapes)
      {
         shape.setMaterial(material);
         shape.setDrawMode(drawMode);
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
         // Cache must reflect this forced 0-scale, otherwise a later valid frame whose size happens to
         // equal the pre-NaN cached size would wrongly skip re-applying the scale, leaving the point
         // permanently invisible.
         lastScale = 0;
         return;
      }

      Point3D positionInWorld = position.toPoint3DInWorld();
      if (positionInWorld.getX() != lastTranslateX)
         translate.setX(lastTranslateX = positionInWorld.getX());
      if (positionInWorld.getY() != lastTranslateY)
         translate.setY(lastTranslateY = positionInWorld.getY());
      if (positionInWorld.getZ() != lastTranslateZ)
         translate.setZ(lastTranslateZ = positionInWorld.getZ());

      if (size == null)
         size = new SimpleDoubleProperty(0.1);
      double sizeValue = size.get();
      if (sizeValue != lastScale)
      {
         scale.setX(sizeValue);
         scale.setY(sizeValue);
         scale.setZ(sizeValue);
         lastScale = sizeValue;
      }

      Color colorValue = color.get();
      if (!Objects.equals(colorValue, lastColor))
      {
         material.setDiffuseColor(colorValue);
         lastColor = colorValue;
      }
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
