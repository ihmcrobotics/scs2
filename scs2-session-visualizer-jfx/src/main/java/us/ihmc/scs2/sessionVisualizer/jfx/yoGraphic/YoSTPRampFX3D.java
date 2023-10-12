package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Scale;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;
import us.ihmc.scs2.simulation.shapes.STPShape3DTools;

public class YoSTPRampFX3D extends YoGraphicFX3D
{
   private final MeshView rampNode = new MeshView(JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(new Ramp3DDefinition(1.0, 1.0, 1.0)));

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private Tuple3DProperty size = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private DoubleProperty minimumMargin = new SimpleDoubleProperty(0.0);
   private DoubleProperty maximumMargin = new SimpleDoubleProperty(0.0);


   private final Affine affine = new Affine();
   private final Scale scale = new Scale();
   private final PhongMaterial material = new PhongMaterial();

   private STPRampData newData = null;
   private STPRampData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoSTPRampFX3D()
   {
      rampNode.setMaterial(material);
      rampNode.getTransforms().addAll(affine, scale);
      rampNode.idProperty().bind(nameProperty());
      rampNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoSTPRampFX3D(ReferenceFrame worldFrame)
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

      newData = newSTPRampData(size,minimumMargin,maximumMargin);

      affine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      scale.setX(size.getX());
      scale.setY(size.getY());
      scale.setZ(size.getZ());

      if (clearMesh)
      {
         clearMesh = false;
         rampNode.setMesh(null);
      }

      if (newMesh != null)
      {
         rampNode.setMesh(newMesh);
         newMesh = null;
      }
   }

   @Override
   public void computeBackground()
   {
      STPRampData newDataLocal = newData;
      newData = null;

      if(newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.size == null)
      {
         clearMesh = true;
         return;
      }
      else if (newDataLocal.equals(oldData) && rampNode.getMesh() != null)
      {
         return;
      }

      try
      {
         newDataLocal.computeRadii();
         newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPRamp3DMesh(null,
                                                                                                                        newDataLocal.size.getX(),
                                                                                                                        newDataLocal.size.getY(),
                                                                                                                        newDataLocal.size.getZ(),
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


   static STPRampData newSTPRampData(Tuple3DProperty size, DoubleProperty minimumMargin, DoubleProperty maximumMargin)
   {
      STPRampData data = new STPRampData();

      if (YoGraphicTools.isAnyNull(size, minimumMargin, maximumMargin))
         return data;
      data.size = new Vector3D(size);
      data.minimumMargin = minimumMargin.get();
      data.maximumMargin = maximumMargin.get();

      return data;

   }

   private static class STPRampData
   {
      private Vector3DReadOnly size;
      private double minimumMargin, maximumMargin;
      private boolean radiiDirty = true;
      private double smallRadius, largeRadius;

      public void computeRadii()
      {
         if(radiiDirty)
         {
            radiiDirty = false;
            smallRadius = minimumMargin;
            largeRadius = STPShape3DTools.computeLargeRadiusFromMargins(minimumMargin,
                                                                        maximumMargin,
                                                                        STPShape3DTools.computeRamp3DMaximumEdgeLengthSquared(size));

         }
      }

      @Override
      public boolean equals(Object object)
      {
         if(object== this)
         {
            return true;
         }
         else if (object instanceof STPRampData)
         {
            STPRampData other = (STPRampData) object;
            if (size == null ? other.size != null : !size.equals(other.size))
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
   public YoSTPRampFX3D clone()
   {
      YoSTPRampFX3D clone = new YoSTPRampFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setOrientation(orientation.clone());
      clone.setSize(new Tuple3DProperty(size));
      clone.setMinimumMargin(minimumMargin);
      clone.setMaximumMargin(maximumMargin);
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

   public DoubleProperty getMinimumMargin()
   {
      return minimumMargin;
   }

   public DoubleProperty getMaximumMargin()
   {
      return maximumMargin;
   }

   @Override
   public Node getNode()
   {
      return rampNode;
   }
}
