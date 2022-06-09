package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.Node;
import javafx.scene.shape.Polygon;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;

public class YoPolygonFX2D extends YoGraphicFX2D
{
   private List<Tuple2DProperty> vertices = new ArrayList<>();
   private IntegerProperty numberOfVertices = null;

   private Polygon polygonNode = new Polygon();

   public YoPolygonFX2D()
   {
      polygonNode.idProperty().bind(nameProperty());
   }

   public YoPolygonFX2D(ReferenceFrame worldFrame)
   {
      this();
   }

   @Override
   public void render()
   {
      polygonNode.getPoints().clear();

      if (vertices == null)
         return;

      if (numberOfVertices == null)
      {
         for (Tuple2DProperty vertex : vertices)
         {
            if (vertex.containsNaN())
               break;
            polygonNode.getPoints().addAll(vertex.getX(), vertex.getY());
         }
      }
      else
      {
         int n = Math.min(numberOfVertices.get(), vertices.size());
         for (int i = 0; i < n; i++)
         {
            Point2D vertex = vertices.get(i).toPoint2DInWorld();

            if (vertex.containsNaN())
               break;
            polygonNode.getPoints().addAll(vertex.getX(), vertex.getY());
         }
      }
      polygonNode.setFill(fillColor == null ? null : fillColor.get());
      polygonNode.setStroke(strokeColor == null ? null : strokeColor.get());
      double scale = polygonNode.getLocalToSceneTransform().deltaTransform(1.0, 0.0).getX();
      polygonNode.setStrokeWidth(strokeWidth.get() / scale);
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

   @Override
   public void clear()
   {
      vertices = null;
      numberOfVertices = null;
      fillColor = null;
      strokeColor = null;
      strokeWidth = null;
   }

   @Override
   public YoPolygonFX2D clone()
   {
      YoPolygonFX2D clone = new YoPolygonFX2D();
      clone.setName(getName());
      clone.setVertices(new ArrayList<>(vertices));
      clone.setNumberOfVertices(numberOfVertices);
      clone.setFillColor(fillColor);
      clone.setStrokeColor(strokeColor);
      clone.setStrokeWidth(strokeWidth);
      return clone;
   }

   public List<Tuple2DProperty> getVertices()
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
      return polygonNode;
   }
}
