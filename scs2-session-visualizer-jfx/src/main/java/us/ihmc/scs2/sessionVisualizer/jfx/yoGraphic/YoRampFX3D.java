package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Scale;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class YoRampFX3D extends YoGraphicFX3D
{
   private final MeshView rampNode = new MeshView(JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(new Ramp3DDefinition(1.0, 1.0, 1.0)));

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private Tuple3DProperty size = new Tuple3DProperty(null, 0.0, 0.0, 0.0);

   private final Affine affine = new Affine();
   private final Scale scale = new Scale();
   private final PhongMaterial material = new PhongMaterial();

   public YoRampFX3D()
   {
      drawModeProperty.addListener((o, oldValue, newValue) ->
                                   {
                                      if (newValue == null)
                                         drawModeProperty.setValue(DrawMode.FILL);
                                      rampNode.setDrawMode(newValue);
                                   });
      rampNode.setMaterial(material);
      rampNode.getTransforms().addAll(affine, scale);
      rampNode.idProperty().bind(nameProperty());
      rampNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoRampFX3D(ReferenceFrameWrapper worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      orientation.setReferenceFrame(worldFrame);
   }

   @Override
   public void render()
   {
      if (position.containsNaN() || orientation.containsNaN() || size.containsNaN())
      {
         affine.setToIdentity();
         scale.setX(0.0);
         scale.setY(0.0);
         scale.setZ(0.0);
      }

      affine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      scale.setX(size.getX());
      scale.setY(size.getY());
      scale.setZ(size.getZ());
   }

   @Override
   public void clear()
   {
      position = null;
      orientation = null;
      size = null;
      color = null;
   }

   @Override
   public YoRampFX3D clone()
   {
      YoRampFX3D clone = new YoRampFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setOrientation(orientation.clone());
      clone.setSize(new Tuple3DProperty(size));
      clone.setColor(color);
      return clone;
   }

   public void setPosition(Tuple3DProperty position)
   {
      this.position = position;
   }

   public void setOrientation(Orientation3DProperty orientation)
   {
      this.orientation = orientation;
   }

   public void setSize(Tuple3DProperty size)
   {
      this.size = size;
   }

   public Tuple3DProperty getPosition()
   {
      return position;
   }

   public Orientation3DProperty getOrientation()
   {
      return orientation;
   }

   public Tuple3DProperty getSize()
   {
      return size;
   }

   @Override
   public Node getNode()
   {
      return rampNode;
   }
}
