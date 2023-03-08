package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class YoEllipsoidFX3D extends YoGraphicFX3D
{
   private final Sphere ellipsoidNode = new Sphere();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private Tuple3DProperty radii = new Tuple3DProperty(null, 0.0, 0.0, 0.0);

   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();

   public YoEllipsoidFX3D()
   {
      ellipsoidNode.setMaterial(material);
      ellipsoidNode.getTransforms().add(affine);
      ellipsoidNode.idProperty().bind(nameProperty());
   }

   public YoEllipsoidFX3D(ReferenceFrame worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      orientation.setReferenceFrame(worldFrame);
      radii.setReferenceFrame(worldFrame);
      ellipsoidNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   @Override
   public void render()
   {
      if (position.containsNaN() || orientation.containsNaN())
      {
         affine.setToIdentity();
         affine.appendScale(0, 0, 0);
         return;
      }

      affine.setToTransform(JavaFXTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      affine.appendScale(radii.getX(), radii.getY(), radii.getZ());

      if (color == null)
         color = new SimpleColorFX();

      material.setDiffuseColor(color.get());
   }

   @Override
   public void clear()
   {
      position = null;
      orientation = null;
      radii = null;
      color = null;
   }

   @Override
   public YoGraphicFX clone()
   {
      YoEllipsoidFX3D clone = new YoEllipsoidFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setOrientation(orientation.clone());
      clone.setRadii(new Tuple3DProperty(radii));
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

   public void setRadii(Tuple3DProperty radii)
   {
      this.radii = radii;
   }

   public Tuple3DProperty getPosition()
   {
      return position;
   }

   public Orientation3DProperty getOrientation()
   {
      return orientation;
   }

   public Tuple3DProperty getRadii()
   {
      return radii;
   }

   @Override
   public Node getNode()
   {
      return ellipsoidNode;
   }
}
