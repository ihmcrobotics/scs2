package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

public class YoArrowFX3D extends YoGraphicFX3D
{
   private Tuple3DProperty origin = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Tuple3DProperty direction = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private boolean scaleLength;
   private boolean scaleRadius;
   private DoubleProperty bodyLength = new SimpleDoubleProperty(0.3);
   private DoubleProperty bodyRadius = new SimpleDoubleProperty(0.005);
   private DoubleProperty headLength = new SimpleDoubleProperty(0.02);
   private DoubleProperty headRadius = new SimpleDoubleProperty(0.01);

   private final PhongMaterial material = new PhongMaterial();
   private final Cylinder body = new Cylinder(1.0, 1.0);
   private final MeshView head = new MeshView(JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(new Cone3DDefinition(1.0, 1.0)));
   private final Group arrow = new Group(body, head);
   private final Affine arrowAffine = new Affine();
   private final Scale headScale = new Scale();

   public YoArrowFX3D()
   {
      body.setMaterial(material);
      head.setMaterial(material);
      body.idProperty().bind(nameProperty().concat(" (body)"));
      head.idProperty().bind(nameProperty().concat(" (head)"));

      Affine bodyAffineToZUp = new Affine();
      bodyAffineToZUp.appendTranslation(0.0, 0.0, 0.5);
      bodyAffineToZUp.append(new Rotate(90.0, Rotate.X_AXIS));
      body.getTransforms().add(bodyAffineToZUp);
      head.getTransforms().addAll(new Translate(0.0, 0.0, 1.0), headScale);

      arrow.getTransforms().addAll(arrowAffine);
      arrow.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoArrowFX3D(ReferenceFrame worldFrame)
   {
      this();
      origin.setReferenceFrame(worldFrame);
      direction.setReferenceFrame(worldFrame);
   }

   @Override
   public void render()
   {
      material.setDiffuseColor(color.get());

      if (origin.containsNaN() || direction.containsNaN() || Double.isNaN(bodyRadius.get()) || Double.isNaN(bodyLength.get()))
      {
         arrowAffine.setToIdentity();
         arrowAffine.appendScale(0, 0, 0);
         return;
      }

      Vector3DReadOnly directionLocal = direction.toVector3DInWorld();
      AxisAngle axisAngle = EuclidGeometryTools.axisAngleFromZUpToVector3D(directionLocal);

      arrowAffine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(axisAngle, origin.toPoint3DInWorld()));

      double directionMagnitude = directionLocal.norm();
      double arrowScaleXY = scaleRadius ? bodyRadius.get() * directionMagnitude : bodyRadius.get();
      double arrowScaleZ = scaleLength ? bodyLength.get() * directionMagnitude : bodyLength.get();

      arrowAffine.appendScale(arrowScaleXY, arrowScaleXY, arrowScaleZ);

      if (Double.isNaN(headRadius.get()) || Double.isNaN(headLength.get()))
      {
         headScale.setX(0.0);
         headScale.setY(0.0);
         headScale.setZ(0.0);
         return;
      }

      double headScaleXY = scaleRadius ? headRadius.get() * directionMagnitude : headRadius.get();
      double headScaleZ = scaleLength ? headLength.get() * directionMagnitude : headLength.get();
      headScaleXY /= arrowScaleXY;
      headScaleZ /= arrowScaleZ;

      headScale.setX(headScaleXY);
      headScale.setY(headScaleXY);
      headScale.setZ(headScaleZ);
   }

   public void setOrigin(Tuple3DProperty origin)
   {
      this.origin = origin;
   }

   public void setDirection(Tuple3DProperty direction)
   {
      this.direction = direction;
   }

   public void setScaleLength(boolean scaleRadius)
   {
      this.scaleLength = scaleRadius;
   }

   public void setScaleRadius(boolean scaleRadius)
   {
      this.scaleRadius = scaleRadius;
   }

   public void setBodyLength(DoubleProperty bodyLength)
   {
      this.bodyLength = bodyLength;
   }

   public void setBodyLength(double bodyLength)
   {
      setBodyLength(new SimpleDoubleProperty(bodyLength));
   }

   public void setBodyRadius(DoubleProperty bodyRadius)
   {
      this.bodyRadius = bodyRadius;
   }

   public void setBodyRadius(double bodyRadius)
   {
      setBodyRadius(new SimpleDoubleProperty(bodyRadius));
   }

   public void setHeadLength(DoubleProperty headLength)
   {
      this.headLength = headLength;
   }

   public void setHeadLength(double headLength)
   {
      setHeadLength(new SimpleDoubleProperty(headLength));
   }

   public void setHeadRadius(DoubleProperty headRadius)
   {
      this.headRadius = headRadius;
   }

   public void setHeadRadius(double headRadius)
   {
      setHeadRadius(new SimpleDoubleProperty(headRadius));
   }

   @Override
   public void clear()
   {
      origin = null;
      direction = null;
      bodyLength = null;
      bodyRadius = null;
      headLength = null;
      headRadius = null;
      color = null;
   }

   @Override
   public YoArrowFX3D clone()
   {
      YoArrowFX3D clone = new YoArrowFX3D();
      clone.setName(getName());
      clone.setOrigin(new Tuple3DProperty(origin));
      clone.setDirection(new Tuple3DProperty(direction));
      clone.setScaleLength(scaleLength);
      clone.setScaleRadius(scaleRadius);
      clone.setBodyLength(bodyLength);
      clone.setBodyRadius(bodyRadius);
      clone.setHeadLength(headLength);
      clone.setHeadRadius(headRadius);
      clone.setColor(color);
      return clone;
   }

   public Tuple3DProperty getOrigin()
   {
      return origin;
   }

   public Tuple3DProperty getDirection()
   {
      return direction;
   }

   public boolean getScaleLength()
   {
      return scaleLength;
   }

   public DoubleProperty getBodyLength()
   {
      return bodyLength;
   }

   public DoubleProperty getHeadLength()
   {
      return headLength;
   }

   public boolean getScaleRadius()
   {
      return scaleRadius;
   }

   public DoubleProperty getBodyRadius()
   {
      return bodyRadius;
   }

   public DoubleProperty getHeadRadius()
   {
      return headRadius;
   }

   @Override
   public Node getNode()
   {
      return arrow;
   }
}
