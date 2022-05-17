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

public class YoConeFX3D extends YoGraphicFX3D
{
   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Tuple3DProperty axis = new Tuple3DProperty(null, 0.0, 0.0, 1.0);
   private DoubleProperty height = new SimpleDoubleProperty(0.3);
   private DoubleProperty radius = new SimpleDoubleProperty(0.05);

   private final PhongMaterial material = new PhongMaterial();
   private final MeshView coneNode = new MeshView();
   private final Translate translate = new Translate();
   private final Rotate rotate = new Rotate();

   private ConeData newData = null;
   private ConeData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoConeFX3D()
   {
      coneNode.setMaterial(material);
      coneNode.getTransforms().addAll(translate, rotate);
      coneNode.idProperty().bind(nameProperty());
   }

   public YoConeFX3D(ReferenceFrame worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      axis.setReferenceFrame(worldFrame);
   }

   @Override
   public void render()
   {
      newData = newConeData(height, radius);

      if (color == null)
         color = () -> null;
      material.setDiffuseColor(color.get());

      if (translate != null)
      {
         Point3D positionInWorld = position.toPoint3DInWorld();
         translate.setX(positionInWorld.getX());
         translate.setY(positionInWorld.getY());
         translate.setZ(positionInWorld.getZ());
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
         coneNode.setMesh(null);
      }

      if (newMesh != null)
      {
         coneNode.setMesh(newMesh);
         newMesh = null;
      }
   }

   @Override
   public void computeBackground()
   {
      ConeData newDataLocal = newData;
      newData = null;

      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.containsNaN() || EuclidCoreTools.isZero(newDataLocal.radius, 1.0e-5))
      {
         clearMesh = true;
         return;
      }
      else if (newDataLocal.equals(oldData))
      {
         return;
      }

      newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.Cone(newDataLocal.height, newDataLocal.radius, 32));
      oldData = newDataLocal;
   }

   private static ConeData newConeData(DoubleProperty height, DoubleProperty radius)
   {
      ConeData data = new ConeData();

      if (YoGraphicTools.isAnyNull(height, radius))
         return data;
      data.height = height.get();
      data.radius = radius.get();
      return data;
   }

   private static class ConeData
   {
      private double height, radius;

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof ConeData)
         {
            ConeData other = (ConeData) object;
            if (height != other.height)
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
         return Double.isNaN(height) || Double.isNaN(radius);
      }
   }

   public void setPosition(Tuple3DProperty position)
   {
      this.position = position;
   }

   public void setAxis(Tuple3DProperty axis)
   {
      this.axis = axis;
   }

   public void setHeight(DoubleProperty height)
   {
      this.height = height;
   }

   public void setHeight(double height)
   {
      this.height = new SimpleDoubleProperty(height);
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
      position = null;
      axis = null;
      height = null;
      radius = null;
      color = null;
   }

   @Override
   public YoConeFX3D clone()
   {
      YoConeFX3D clone = new YoConeFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setAxis(new Tuple3DProperty(axis));
      clone.setHeight(height);
      clone.setRadius(radius);
      clone.setColor(color);
      return clone;
   }

   public Tuple3DProperty getPosition()
   {
      return position;
   }

   public Tuple3DProperty getAxis()
   {
      return axis;
   }

   public DoubleProperty getHeight()
   {
      return height;
   }

   public DoubleProperty getRadius()
   {
      return radius;
   }

   @Override
   public Node getNode()
   {
      return coneNode;
   }
}
