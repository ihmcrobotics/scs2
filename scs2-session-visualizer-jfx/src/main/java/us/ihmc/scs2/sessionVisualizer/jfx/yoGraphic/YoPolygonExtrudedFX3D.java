package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

public class YoPolygonExtrudedFX3D extends YoGraphicFX3D
{
   private final MeshView polygonNode = new MeshView();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private List<Tuple2DProperty> vertices = new ArrayList<>();
   private IntegerProperty numberOfVertices = null;
   private DoubleProperty thickness = new SimpleDoubleProperty(0.02);
   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();

   private PolygonData newData = null;
   private PolygonData oldData = null;
   private Mesh newMesh = null;
   private boolean clearMesh = false;

   public YoPolygonExtrudedFX3D()
   {
      polygonNode.setMaterial(material);
      polygonNode.getTransforms().add(affine);
      polygonNode.idProperty().bind(nameProperty());
   }

   public YoPolygonExtrudedFX3D(ReferenceFrame worldFrame)
   {
      this();
      position.setReferenceFrame(worldFrame);
      orientation.setReferenceFrame(worldFrame);
   }

   @Override
   public void render()
   {
      newData = newPolygonData(vertices, numberOfVertices, thickness);

      affine.setToTransform(JavaFXTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = () -> null;
      material.setDiffuseColor(color.get());

      if (clearMesh)
      {
         clearMesh = false;
         polygonNode.setMesh(null);
      }

      if (newMesh != null)
      {
         polygonNode.setMesh(newMesh);
         newMesh = null;
      }
   }

   static PolygonData newPolygonData(List<Tuple2DProperty> vertices, IntegerProperty numberOfVertices, DoubleProperty thickness)
   {
      PolygonData data = new PolygonData();
      if (YoGraphicTools.isAnyNull(vertices, numberOfVertices, thickness))
         return data;
      if (vertices.size() <= 2 || thickness.get() <= 0.0)
         return data;

      if (numberOfVertices != null && numberOfVertices.get() >= 0 && numberOfVertices.get() < vertices.size())
         vertices = vertices.subList(0, numberOfVertices.get());
      data.vertices = vertices.stream().map(Point2D::new).collect(Collectors.toList());
      data.thickness = thickness.get();
      return data;
   }

   private static class PolygonData
   {
      private List<Point2DReadOnly> vertices;
      private double thickness;

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof PolygonData)
         {
            PolygonData other = (PolygonData) object;
            if (vertices == null ? other.vertices != null : !vertices.equals(other.vertices))
               return false;
            if (thickness != other.thickness)
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
      PolygonData newDataLocal = newData;
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
      else if (newDataLocal.equals(oldData) && polygonNode.getMesh() != null)
      {
         return;
      }

      List<Point2DReadOnly> vertices = newDataLocal.vertices;

      if (vertices.size() > 2)
      {
         Vector2D firstEdge = new Vector2D();
         firstEdge.sub(vertices.get(1), vertices.get(0));
         Vector2D secondEdge = new Vector2D();
         secondEdge.sub(vertices.get(2), vertices.get(1));

         if (firstEdge.cross(secondEdge) < 0.0)
         {
            vertices = new ArrayList<>(vertices);
            Collections.reverse(vertices);
         }
      }

      newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.ExtrudedPolygon(vertices, newDataLocal.thickness));
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

   public void setVertices(List<Tuple2DProperty> vertices)
   {
      this.vertices = vertices;
   }

   public void addVertex(DoubleProperty x, DoubleProperty y)
   {
      addVertex(new Tuple2DProperty(null, x, y));
   }

   public void addVertex(Tuple2DProperty vertex)
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

   public void setThickness(DoubleProperty thickness)
   {
      this.thickness = thickness;
   }

   public void setThickness(double thickness)
   {
      setThickness(new SimpleDoubleProperty(thickness));
   }

   @Override
   public void clear()
   {
      position = null;
      orientation = null;
      vertices = null;
      numberOfVertices = null;
      thickness = null;
      color = null;
   }

   @Override
   public YoPolygonExtrudedFX3D clone()
   {
      YoPolygonExtrudedFX3D clone = new YoPolygonExtrudedFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setOrientation(orientation.clone());
      clone.setVertices(new ArrayList<>(vertices));
      clone.setNumberOfVertices(numberOfVertices);
      clone.setThickness(thickness);
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

   public List<Tuple2DProperty> getVertices()
   {
      return vertices;
   }

   public IntegerProperty getNumberOfVertices()
   {
      return numberOfVertices;
   }

   public DoubleProperty getThickness()
   {
      return thickness;
   }

   @Override
   public Node getNode()
   {
      return polygonNode;
   }
}
