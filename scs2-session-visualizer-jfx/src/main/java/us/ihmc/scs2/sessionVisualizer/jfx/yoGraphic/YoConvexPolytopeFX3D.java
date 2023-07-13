package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class YoConvexPolytopeFX3D extends YoGraphicFX3D
{
   private final MeshView polytopeNode = new MeshView();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private List<Tuple3DProperty> vertices = new ArrayList<>();
   private IntegerProperty numberOfVertices = null;
   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();

   private PolytopeData newData = null;
   private PolytopeData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoConvexPolytopeFX3D()
   {
      polytopeNode.setMaterial(material);
      polytopeNode.getTransforms().add(affine);
      polytopeNode.idProperty().bind(nameProperty());
      polytopeNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoConvexPolytopeFX3D(ReferenceFrame worldFrame)
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

      newData = newPolytopeData(vertices, numberOfVertices);

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

   static PolytopeData newPolytopeData(List<Tuple3DProperty> vertices, IntegerProperty numberOfVertices)
   {
      PolytopeData data = new PolytopeData();
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
      return data;
   }

   private static class PolytopeData
   {
      private List<Point3DReadOnly> vertices = new ArrayList<>();

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof PolytopeData)
         {
            PolytopeData other = (PolytopeData) object;
            if (vertices == null ? other.vertices != null : !vertices.equals(other.vertices))
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
   public void computeBackground()
   {
      PolytopeData newDataLocal = newData;
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

      List<Point3DReadOnly> vertices = newDataLocal.vertices;

      newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.ConvexPolytope(new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices))));
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

   @Override
   public void clear()
   {
      position = null;
      orientation = null;
      vertices = null;
      numberOfVertices = null;
      color = null;
   }

   @Override
   public YoConvexPolytopeFX3D clone()
   {
      YoConvexPolytopeFX3D clone = new YoConvexPolytopeFX3D();
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

   @Override
   public Node getNode()
   {
      return polytopeNode;
   }
}
