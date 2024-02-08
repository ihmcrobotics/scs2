package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;
import us.ihmc.scs2.simulation.shapes.STPShape3DTools;

public class YoSTPBoxFX3D extends YoGraphicFX3D
{
   private final MeshView boxNode = new MeshView();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private Tuple3DProperty size = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private DoubleProperty minimumMargin = new SimpleDoubleProperty(0.0);
   private DoubleProperty maximumMargin = new SimpleDoubleProperty(0.0);

   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();

   private STPBoxData newData = null;
   private STPBoxData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoSTPBoxFX3D()
   {
      boxNode.setMaterial(material);
      boxNode.getTransforms().add(affine);
      boxNode.idProperty().bind(nameProperty());
      boxNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoSTPBoxFX3D(ReferenceFrameWrapper worldFrame)
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
         boxNode.setMesh(null);
         oldData = null;
         return;
      }

      newData = newSTPBoxData(size, minimumMargin, maximumMargin);

      affine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      if (clearMesh)
      {
         clearMesh = false;
         boxNode.setMesh(null);
      }

      if (newMesh != null)
      {
         boxNode.setMesh(newMesh);
         newMesh = null;
      }
   }

   @Override
   public void computeBackground()
   {
      STPBoxData newDataLocal = newData;
      newData = null;

      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.size == null)
      {
         clearMesh = true;
         return;
      }
      else if (newDataLocal.equals(oldData) && boxNode.getMesh() != null)
      {
         return;
      }

      try
      {
         newDataLocal.computeRadii();
         newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPBox3DMesh(null,
                                                                                                                        newDataLocal.size,
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

   static STPBoxData newSTPBoxData(Tuple3DProperty size, DoubleProperty minimumMargin, DoubleProperty maximumMargin)
   {
      STPBoxData data = new STPBoxData();
      if (YoGraphicTools.isAnyNull(size, minimumMargin, maximumMargin))
         return data;
      data.size = new Vector3D(size);
      data.minimumMargin = minimumMargin.get();
      data.maximumMargin = maximumMargin.get();
      return data;
   }

   private static class STPBoxData
   {
      private Vector3DReadOnly size;
      private double minimumMargin, maximumMargin;
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
                                                                        STPShape3DTools.computeBox3DMaximumEdgeLengthSquared(size));
         }
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof STPBoxData)
         {
            STPBoxData other = (STPBoxData) object;
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
   public YoSTPBoxFX3D clone()
   {
      YoSTPBoxFX3D clone = new YoSTPBoxFX3D();
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
      return boxNode;
   }
}
