package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.DrawMode;
import javafx.scene.transform.Affine;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class YoBoxFX3D extends YoGraphicFX3D
{
   private final Box boxNode = new Box();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private Tuple3DProperty size = new Tuple3DProperty(null, 0.0, 0.0, 0.0);

   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();

   public YoBoxFX3D()
   {
      drawModeProperty.addListener((o, oldValue, newValue) ->
                                   {
                                      if (newValue == null)
                                         drawModeProperty.setValue(DrawMode.FILL);
                                      boxNode.setDrawMode(newValue);
                                   });
      boxNode.setMaterial(material);
      boxNode.getTransforms().add(affine);
      boxNode.idProperty().bind(nameProperty());
      boxNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoBoxFX3D(ReferenceFrameWrapper worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      orientation.setReferenceFrame(worldFrame);
      size.setReferenceFrame(worldFrame);
   }

   @Override
   public void render()
   {
      if (position.containsNaN() || orientation.containsNaN() || size.containsNaN())
      {
         affine.setToIdentity();
         affine.appendTranslation(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
         affine.appendScale(0, 0, 0);
         return;
      }

      affine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      boxNode.setWidth(size.getX());
      boxNode.setHeight(size.getY());
      boxNode.setDepth(size.getZ());
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
   public YoGraphicFX clone()
   {
      YoBoxFX3D clone = new YoBoxFX3D();
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
      return boxNode;
   }
}
