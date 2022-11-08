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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

public class YoCylinderFX3D extends YoGraphicFX3D
{
   private Tuple3DProperty center = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Tuple3DProperty axis = new Tuple3DProperty(null, 0.0, 0.0, 1.0);
   private DoubleProperty length = new SimpleDoubleProperty(0.3);
   private DoubleProperty radius = new SimpleDoubleProperty(0.05);

   private final PhongMaterial material = new PhongMaterial();
   private final MeshView cylinderNode = new MeshView();
   private final Translate translate = new Translate();
   private final Rotate rotate = new Rotate();

   private CylinderData newData = null;
   private CylinderData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoCylinderFX3D()
   {
      cylinderNode.setMaterial(material);
      cylinderNode.getTransforms().addAll(translate, rotate);
      cylinderNode.idProperty().bind(nameProperty());
      cylinderNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoCylinderFX3D(ReferenceFrame worldFrame)
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
         cylinderNode.setMesh(null);
         return;
      }

      newData = newCylinderData(length, radius);

      if (color == null)
         color = () -> null;
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
         JavaFXTools.convertAxisAngleToRotate(axisAngle, rotate); // FIXME: The conversion has to handle the change of unit: radians -> degrees
      }

      if (clearMesh)
      {
         clearMesh = false;
         cylinderNode.setMesh(null);
      }

      if (newMesh != null)
      {
         cylinderNode.setMesh(newMesh);
         newMesh = null;
      }
   }

   @Override
   public void computeBackground()
   {
      CylinderData newDataLocal = newData;
      newData = null;

      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.containsNaN() || EuclidCoreTools.isZero(newDataLocal.length, 1.0e-5) || EuclidCoreTools.isZero(newDataLocal.radius, 1.0e-5))
      {
         clearMesh = true;
         oldData = null;
         return;
      }
      else if (newDataLocal.equals(oldData))
      {
         return;
      }

      newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.Cylinder(newDataLocal.radius,
                                                                                                               newDataLocal.length,
                                                                                                               32,
                                                                                                               true));
      oldData = newDataLocal;
   }

   private static CylinderData newCylinderData(DoubleProperty length, DoubleProperty radius)
   {
      CylinderData data = new CylinderData();

      if (YoGraphicTools.isAnyNull(length, radius))
         return data;
      data.length = length.get();
      data.radius = radius.get();
      return data;
   }

   private static class CylinderData
   {
      private double length, radius;

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof CylinderData)
         {
            CylinderData other = (CylinderData) object;
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
   public YoCylinderFX3D clone()
   {
      YoCylinderFX3D clone = new YoCylinderFX3D();
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
      return cylinderNode;
   }
}
