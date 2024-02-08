package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class YoCapsuleFX3D extends YoGraphicFX3D
{
   private Tuple3DProperty center = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Tuple3DProperty axis = new Tuple3DProperty(null, 0.0, 0.0, 1.0);
   private DoubleProperty length = new SimpleDoubleProperty(0.3);
   private DoubleProperty radius = new SimpleDoubleProperty(0.05);

   private final PhongMaterial material = new PhongMaterial();
   private final MeshView capsuleNode = new MeshView();
   private final Translate translate = new Translate();
   private final Rotate rotate = new Rotate();

   private CapsuleData newData = null;
   private CapsuleData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoCapsuleFX3D()
   {
      capsuleNode.setMaterial(material);
      capsuleNode.getTransforms().addAll(translate, rotate);
      capsuleNode.idProperty().bind(nameProperty());
      capsuleNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoCapsuleFX3D(ReferenceFrameWrapper worldFrame)
   {
      this();
      center.setReferenceFrame(worldFrame);
      axis.setReferenceFrame(worldFrame);
   }

   @Override
   public void render()
   {
      if (center.containsNaN() || axis.containsNaN())
      {
         oldData = null;
         capsuleNode.setMesh(null);
         return;
      }

      newData = newCapsuleData(length, radius);

      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      if (translate != null)
      {
         Point3D centerInWorld = center.toPoint3DInWorld();
         translate.setX(centerInWorld.getX());
         translate.setY(centerInWorld.getY());
         translate.setZ(centerInWorld.getZ());
      }
      if (axis != null)
      {
         AxisAngle axisAngle = EuclidGeometryTools.axisAngleFromZUpToVector3D(axis.toVector3DInWorld());
         axisAngle.setAngle(Math.toDegrees(axisAngle.getAngle()));
         JavaFXMissingTools.convertAxisAngleToRotate(axisAngle, rotate); // FIXME: The conversion has to handle the change of unit: radians -> degrees
      }

      if (clearMesh)
      {
         clearMesh = false;
         capsuleNode.setMesh(null);
      }

      if (newMesh != null)
      {
         capsuleNode.setMesh(newMesh);
         newMesh = null;
      }
   }

   @Override
   public void computeBackground()
   {
      CapsuleData newDataLocal = newData;
      newData = null;

      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.containsNaN() || EuclidCoreTools.isZero(newDataLocal.radius, 1.0e-5))
      {
         clearMesh = true;
         oldData = null;
         return;
      }
      else if (newDataLocal.equals(oldData))
      {
         return;
      }

      newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.Capsule(newDataLocal.length,
                                                                                                              newDataLocal.radius,
                                                                                                              newDataLocal.radius,
                                                                                                              newDataLocal.radius,
                                                                                                              32,
                                                                                                              32));
      oldData = newDataLocal;
   }

   private static CapsuleData newCapsuleData(DoubleProperty length, DoubleProperty radius)
   {
      CapsuleData data = new CapsuleData();

      if (YoGraphicTools.isAnyNull(length, radius))
         return data;
      data.length = length.get();
      data.radius = radius.get();
      return data;
   }

   private static class CapsuleData
   {
      private double length, radius;

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof CapsuleData)
         {
            CapsuleData other = (CapsuleData) object;
            if (length != other.length)
               return false;
            if (radius != other.radius)
               return false;
            return true;
         }
         else
         {
            return false;
         }
      }

      public boolean containsNaN()
      {
         return Double.isNaN(length) || Double.isNaN(radius);
      }
   }

   public void setCenter(Tuple3DProperty center)
   {
      this.center = center;
   }

   public void setAxis(Tuple3DProperty axis)
   {
      this.axis = axis;
   }

   public void setLength(DoubleProperty length)
   {
      this.length = length;
   }

   public void setLength(double length)
   {
      this.length = new SimpleDoubleProperty(length);
   }

   public void setRadius(DoubleProperty radius)
   {
      this.radius = radius;
   }

   public void setRadius(double radius)
   {
      this.radius = new SimpleDoubleProperty(radius);
   }

   @Override
   public void clear()
   {
      center = null;
      axis = null;
      length = null;
      radius = null;
      color = null;
   }

   @Override
   public YoCapsuleFX3D clone()
   {
      YoCapsuleFX3D clone = new YoCapsuleFX3D();
      clone.setName(getName());
      clone.setCenter(new Tuple3DProperty(center));
      clone.setAxis(new Tuple3DProperty(axis));
      clone.setLength(length);
      clone.setRadius(radius);
      clone.setColor(color);
      return clone;
   }

   public Tuple3DProperty getCenter()
   {
      return center;
   }

   public Tuple3DProperty getAxis()
   {
      return axis;
   }

   public DoubleProperty getLength()
   {
      return length;
   }

   public DoubleProperty getRadius()
   {
      return radius;
   }

   @Override
   public Node getNode()
   {
      return capsuleNode;
   }
}
