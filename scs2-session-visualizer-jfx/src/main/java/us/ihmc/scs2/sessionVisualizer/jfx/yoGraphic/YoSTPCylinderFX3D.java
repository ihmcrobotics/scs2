package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;
import us.ihmc.scs2.simulation.shapes.STPCylinder3D;
import us.ihmc.scs2.simulation.shapes.STPShape3DTools;

public class YoSTPCylinderFX3D extends YoGraphicFX3D
{
   private final MeshView cylinderNode = new MeshView();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Tuple3DProperty axis = new Tuple3DProperty(null, 0.0,0.0,1.0);
   private DoubleProperty length = new SimpleDoubleProperty(0.3);
   private DoubleProperty radius = new SimpleDoubleProperty(0.05);
   private DoubleProperty minimumMargin = new SimpleDoubleProperty(0.0);
   private DoubleProperty maximumMargin = new SimpleDoubleProperty(0.0);

//   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();
   private final Translate translate = new Translate();
   private final Rotate rotate = new Rotate();


   private STPCylinderData newData = null;
   private STPCylinderData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;
   
   public YoSTPCylinderFX3D()
   {
      cylinderNode.setMaterial(material);
//      cylinderNode.getTransforms().add(affine);
      cylinderNode.getTransforms().addAll(translate,rotate);
      cylinderNode.idProperty().bind(nameProperty());
      cylinderNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }


   public YoSTPCylinderFX3D(ReferenceFrame worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      axis.setReferenceFrame(worldFrame);
      
   }

   @Override
   public void render()
   {
      if (position.containsNaN() || axis.containsNaN() )
      {
         cylinderNode.setMesh(null);
         oldData = null;
         return;
      }

      newData = newSTPCylinderData(length,radius,minimumMargin,maximumMargin);

//      affine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(axis.toVector3DInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      if (translate != null)
      {
         Point3D centerInWorld = position.toPoint3DInWorld();
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
      STPCylinderData newDataLocal = newData;
      newData = null;
      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.containsNaN() || EuclidCoreTools.isZero(newDataLocal.length, 1.0e-5) || EuclidCoreTools.isZero(newDataLocal.radius, 1.0e-5))      {
         clearMesh = true;
         oldData = null;
         return;
      }
      else if (newDataLocal.equals(oldData))
      {
         return;
      }
      try
      {
         newDataLocal.computeRadii();
         newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPCylinder3DMesh(null,
                                                                                                                        newDataLocal.radius,
                                                                                                                        newDataLocal.length,
                                                                                                                        newDataLocal.smallRadius,
                                                                                                                        newDataLocal.largeRadius,
                                                                                                                        false));
      }
      catch (IllegalArgumentException e)
      {
         return;
      }

      oldData = newDataLocal;

   }
   
   static STPCylinderData newSTPCylinderData(DoubleProperty length, DoubleProperty radius, DoubleProperty minimumMargin, DoubleProperty maximumMargin)
   {
      STPCylinderData data = new STPCylinderData();
      if(YoGraphicTools.isAnyNull(length,radius,minimumMargin,maximumMargin))
         return data;
      data.length = length.get();
      data.radius = radius.get();
      data.minimumMargin = minimumMargin.get();
      data.maximumMargin = maximumMargin.get();
      return data;
   }

   private static class STPCylinderData
   {
      private double minimumMargin, maximumMargin;
      private double length, radius;
      private boolean radiiDirty = true;
      private double smallRadius, largeRadius;

      public void computeRadii()
      {
         if (radiiDirty)
         {
            radiiDirty = false;
            smallRadius = minimumMargin;
            largeRadius = STPShape3DTools.computeLargeRadiusFromMargins(minimumMargin,
                                                                        maximumMargin,
                                                                        STPShape3DTools.computeCylinder3DMaximumEdgeLengthSquared(length,radius));

         }
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof STPCylinderData)
         {
            STPCylinderData other = (STPCylinderData) object;

            if(length != other.length)
               return false;
            if(radius != other.radius)
               return false;
            if (minimumMargin != other.minimumMargin)
               return false;
            if (maximumMargin != other.maximumMargin)
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

   @Override
   public void clear()
   {
      position  = null;
      axis = null;
      length = null;
      radius = null;
      color = null;
   }

   @Override
   public YoSTPCylinderFX3D clone()
   {
      YoSTPCylinderFX3D clone = new YoSTPCylinderFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setAxis(new Tuple3DProperty(axis));
      clone.setLength(length);
      clone.setRadius(radius);
      clone.setMinimumMargin(minimumMargin);
      clone.setMaximumMargin(maximumMargin);
      clone.setColor(color);
      return clone;
   }

   public void setPosition(Tuple3DProperty position) {this.position = position;}

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

   public void setMinimumMargin(DoubleProperty minimumMargin)
   {
      this.minimumMargin = minimumMargin;
   }

   public void setMinimumMargin(double minimumMargin)
   {
      setMinimumMargin(new SimpleDoubleProperty(minimumMargin));
   }
   public void setMaximumMargin(DoubleProperty maximumMargin)
   {
      this.maximumMargin = maximumMargin;
   }

   public void setMaximumMargin(double maximumMargin)
   {
      setMaximumMargin(new SimpleDoubleProperty(maximumMargin));
   }

   public Tuple3DProperty getPosition() {return position;}

   public Tuple3DProperty getAxis() {return axis;}

   public DoubleProperty getLength() {return length;}

   public DoubleProperty getRadius() {return radius;}

   public DoubleProperty getMinimumMargin() {return minimumMargin;}

   public DoubleProperty getMaximumMargin() {return maximumMargin;}


   @Override
   public Node getNode() { return cylinderNode;}
}
