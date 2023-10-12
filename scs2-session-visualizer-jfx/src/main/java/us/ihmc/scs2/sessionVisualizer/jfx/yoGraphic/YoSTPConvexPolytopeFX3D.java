package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;
import us.ihmc.scs2.simulation.shapes.STPConvexPolytope3D;
import us.ihmc.scs2.simulation.shapes.STPShape3DTools;

public class YoSTPConvexPolytopeFX3D extends YoGraphicFX3D
{
   private final MeshView polytopeNode = new MeshView();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private List<Tuple3DProperty> vertices = new ArrayList<>();
   private IntegerProperty numberOfVertices = null;
   private final Affine affine = new Affine();
   private DoubleProperty radius = new SimpleDoubleProperty(0.05);
   private DoubleProperty minimumMargin = new SimpleDoubleProperty(0.015);
   private DoubleProperty maximumMargin = new SimpleDoubleProperty(0.035);

   private ConvexPolytope3DReadOnly polytope3D;

   private final PhongMaterial material = new PhongMaterial();

   private STPPolytopeData newData = null;
   private STPPolytopeData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;


   public YoSTPConvexPolytopeFX3D()
   {
      polytopeNode.setMaterial(material);
      polytopeNode.getTransforms().add(affine);
      polytopeNode.idProperty().bind(nameProperty());
      polytopeNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY,this);
   }

   public YoSTPConvexPolytopeFX3D(ReferenceFrame worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      orientation.setReferenceFrame(worldFrame);
   }


   @Override
   public void render()
   {
      if (position.containsNaN() || orientation.containsNaN())
      {
         oldData = null;
         polytopeNode.setMesh(null);
         return;
      }

      newData = newSTPPolytopeData(polytope3D,minimumMargin, maximumMargin,vertices, numberOfVertices);

      affine.setToTransform(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      if (clearMesh)
      {
         clearMesh = false;
         polytopeNode.setMesh(null);
      }

      if (newMesh != null)
      {
         polytopeNode.setMesh(newMesh);
         newMesh = null;
      }
   }


   static STPPolytopeData newSTPPolytopeData(ConvexPolytope3DReadOnly polytope3D,DoubleProperty minimumMargin, DoubleProperty maximumMargin, List<Tuple3DProperty> vertices, IntegerProperty numberOfVertices)
   {
      STPPolytopeData data = new STPPolytopeData();
      if (YoGraphicTools.isAnyNull(vertices, numberOfVertices))
         return data;
      if (vertices.size() <= 2)
         return data;

      if (numberOfVertices != null && numberOfVertices.get() >= 0 && numberOfVertices.get() < vertices.size())
         vertices = vertices.subList(0, numberOfVertices.get());
      data.vertices = new ArrayList<>();

      for (Tuple3DProperty vertex : vertices)
      {
         if (vertex.containsNaN())
            break;
         data.vertices.add(new Point3D(vertex));
      }

      data.minimumMargin = minimumMargin.get();
      data.maximumMargin = maximumMargin.get();
      data.polytope3D = polytope3D;

      return data;
   }

   private static class STPPolytopeData
   {
      private List<Point3DReadOnly> vertices = new ArrayList<>();
      private double minimumMargin, maximumMargin;
      private boolean radiiDirty = true;
      private double smallRadius, largeRadius;
      private ConvexPolytope3DReadOnly polytope3D;

      public void computeRadii()
      {
         if(radiiDirty)
         {
            radiiDirty = false;
            smallRadius = minimumMargin;
            largeRadius = STPShape3DTools.computeLargeRadiusFromMargins(minimumMargin,
                                                                        maximumMargin,
                                                                        STPShape3DTools.computeConvexPolytope3DMaximumEdgeLengthSquared(polytope3D));
         }
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof STPPolytopeData)
         {
            STPPolytopeData other = (STPPolytopeData) object;
            if (vertices == null ? other.vertices != null : !vertices.equals(other.vertices))
               return false;
            if(minimumMargin != other.minimumMargin)
               return false;
            if(maximumMargin != other.maximumMargin)
               return false;
            return true;
         }
         else
         {
            return false;
         }
      }
   }

   public void computeBackground()
   {
      STPPolytopeData newDataLocal = newData;
      newData = null;

      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.vertices == null)
      {
         clearMesh = true;
         return;
      }
      else if (newDataLocal.equals(oldData) && polytopeNode.getMesh() != null)
      {
         return;
      }
      try
      {
         newDataLocal.computeRadii();
         newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPConvexPolytope3DMesh(newDataLocal.polytope3D,
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

   public void setPosition(Tuple3DProperty position)
   {
      this.position = position;
   }

   public void setOrientation(Orientation3DProperty orientation)
   {
      this.orientation = orientation;
   }

   public void setVertices(List<Tuple3DProperty> vertices)
   {
      this.vertices = vertices;
   }

   public void addVertex(DoubleProperty x, DoubleProperty y, DoubleProperty z)
   {
      addVertex(new Tuple3DProperty(null, x, y, z));
   }

   public void addVertex(Tuple3DProperty vertex)
   {
      vertices.add(vertex);
   }

   public void setNumberOfVertices(IntegerProperty numberOfVertices)
   {
      this.numberOfVertices = numberOfVertices;
   }

   public void setNumberOfVertices(int numberOfVertices)
   {
      setNumberOfVertices(new SimpleIntegerProperty(numberOfVertices));
   }

   public void setRadius(DoubleProperty radius) {this.radius = radius;}

   public void setRadius(double radius) {this.radius = new SimpleDoubleProperty(radius);}

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

   public void setPolytope3D(ConvexPolytope3DReadOnly polytope3D) {this.polytope3D = polytope3D;}

   @Override
   public void clear()
   {
      position = null;
      orientation = null;
      vertices = null;
      numberOfVertices = null;
      radius = null;
      color = null;
   }

   @Override
   public YoSTPConvexPolytopeFX3D clone()
   {
      YoSTPConvexPolytopeFX3D clone = new YoSTPConvexPolytopeFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setOrientation(orientation.clone());
      clone.setVertices(new ArrayList<>(vertices));
      clone.setNumberOfVertices(numberOfVertices);
      clone.setColor(color);
      return clone;
   }

   public Tuple3DProperty getPosition()
   {
      return position;
   }

   public Orientation3DProperty getOrientation()
   {
      return orientation;
   }

   public List<Tuple3DProperty> getVertices()
   {
      return vertices;
   }

   public IntegerProperty getNumberOfVertices()
   {
      return numberOfVertices;
   }

   public DoubleProperty getRadius() {return radius;}

   public ConvexPolytope3DReadOnly getPolytope() {return polytope3D;}

   @Override
   public Node getNode()
   {
      return polytopeNode;
   }

}
