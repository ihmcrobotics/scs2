package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ReadOnlyStringProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Orientation3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.SimpleColorFX;

public class YoCoordinateSystemFX3D extends YoGraphicFX3D
{
   private static final Color[] axisColors = new Color[] {Color.RED, Color.GREEN, Color.BLUE};
   private static final Rotate[] axisBodyRotates = {new Rotate(-90.0, Rotate.Z_AXIS), null, new Rotate(90.0, Rotate.X_AXIS)};
   private static final AxisAngle[] axisHeadOrientations = {new AxisAngle(Axis3D.Y, Math.PI / 2.0), new AxisAngle(Axis3D.X, -Math.PI / 2.0), new AxisAngle()};

   private final Group coordinateSystemNode = new Group();

   private Tuple3DProperty position = new Tuple3DProperty(null, 0.0, 0.0, 0.0);
   private Orientation3DProperty orientation = new QuaternionProperty(null, 0.0, 0.0, 0.0, 1.0);
   private DoubleProperty bodyLength = new SimpleDoubleProperty(0.3);
   private DoubleProperty bodyRadius = new SimpleDoubleProperty(0.01);
   private DoubleProperty headLength = new SimpleDoubleProperty(0.05);
   private DoubleProperty headRadius = new SimpleDoubleProperty(0.02);
   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();

   private CoordinateSystemData newData = null;
   private CoordinateSystemData oldData = null;
   private Node[] newNodes = null;

   public YoCoordinateSystemFX3D()
   {
      coordinateSystemNode.getTransforms().add(affine);
      coordinateSystemNode.idProperty().bind(nameProperty());
      coordinateSystemNode.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
   }

   public YoCoordinateSystemFX3D(ReferenceFrame worldFrame)
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
         newNodes = null;
         oldData = null;
         coordinateSystemNode.getChildren().clear();
         return;
      }

      newData = newCoordinateSystemData(bodyLength, bodyRadius, headLength, headRadius);

      affine.setToTransform(JavaFXTools.createAffineFromOrientation3DAndTuple(orientation.toQuaternionInWorld(), position.toPoint3DInWorld()));
      if (color == null)
         color = new SimpleColorFX();
      material.setDiffuseColor(color.get());

      if (newNodes != null)
      {
         coordinateSystemNode.getChildren().clear();
         coordinateSystemNode.getChildren().addAll(newNodes);
         newNodes = null;
      }
   }

   static CoordinateSystemData newCoordinateSystemData(DoubleProperty bodyLength,
                                                       DoubleProperty bodyRadius,
                                                       DoubleProperty headLength,
                                                       DoubleProperty headRadius)
   {
      CoordinateSystemData data = new CoordinateSystemData();
      if (YoGraphicTools.isAnyNull(bodyLength, bodyRadius, headLength, headRadius))
         return data;

      data.bodyLength = bodyLength.get();
      data.bodyRadius = bodyRadius.get();
      data.headLength = headLength.get();
      data.headRadius = headRadius.get();
      return data;
   }

   static class CoordinateSystemData
   {
      private double bodyLength = Double.NaN, bodyRadius = Double.NaN;
      private double headLength = Double.NaN, headRadius = Double.NaN;

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof CoordinateSystemData)
         {
            CoordinateSystemData other = (CoordinateSystemData) object;
            if (bodyLength != other.bodyLength)
               return false;
            if (bodyRadius != other.bodyRadius)
               return false;
            if (headLength != other.headLength)
               return false;
            if (headRadius != other.headRadius)
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
         return Double.isNaN(bodyLength) || Double.isNaN(bodyRadius) || Double.isNaN(headLength) || Double.isNaN(headRadius);
      }
   }

   @Override
   public void computeBackground()
   {
      CoordinateSystemData newDataLocal = newData;
      newData = null;

      if (newDataLocal == null)
      {
         return;
      }
      else if (newDataLocal.containsNaN())
      {
         newNodes = new Node[0];
         return;
      }
      else if (newDataLocal.equals(oldData) && !coordinateSystemNode.getChildren().isEmpty())
      {
         return;
      }

      oldData = newDataLocal;
      newNodes = createCoordinateSystem(newDataLocal, material, nameProperty());
   }

   static Node[] createCoordinateSystem(CoordinateSystemData data, Material material, ReadOnlyStringProperty nameProperty)
   {
      Node[] nodes = new Node[6];

      Translate axisBodyTranslate = new Translate(0.0, 0.5 * data.bodyLength, 0.0);

      for (int axis = 0; axis < 3; axis++)
      {
         Cylinder body = new Cylinder(data.bodyRadius, data.bodyLength);
         body.setMaterial(material);
         body.idProperty().bind(nameProperty.concat(" (").concat(Axis3D.values[axis].name()).concat("-body)"));

         if (axisBodyRotates[axis] != null)
            body.getTransforms().add(axisBodyRotates[axis]);
         body.getTransforms().add(axisBodyTranslate);

         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
         Point3D headPosition = new Point3D();
         headPosition.setElement(axis, data.bodyLength);
         meshBuilder.addCone(data.headLength, data.headRadius, headPosition, axisHeadOrientations[axis]);
         MeshView head = new MeshView(meshBuilder.generateMesh());
         head.setMaterial(new PhongMaterial(axisColors[axis]));
         head.idProperty().bind(nameProperty.concat(" (").concat(Axis3D.values[axis].name()).concat("-head)"));

         nodes[2 * axis] = body;
         nodes[2 * axis + 1] = head;
      }

      return nodes;
   }

   public void setPosition(Tuple3DProperty position)
   {
      this.position = position;
   }

   public void setOrientation(Orientation3DProperty orientation)
   {
      this.orientation = orientation;
   }

   public void setBodyLength(DoubleProperty bodyLength)
   {
      this.bodyLength = bodyLength;
   }

   public void setBodyLength(double bodyLength)
   {
      setBodyLength(new SimpleDoubleProperty(bodyLength));
   }

   public void setBodyRadius(DoubleProperty bodyRadius)
   {
      this.bodyRadius = bodyRadius;
   }

   public void setBodyRadius(double bodyRadius)
   {
      setBodyRadius(new SimpleDoubleProperty(bodyRadius));
   }

   public void setHeadLength(DoubleProperty headLength)
   {
      this.headLength = headLength;
   }

   public void setHeadLength(double headLength)
   {
      setHeadLength(new SimpleDoubleProperty(headLength));
   }

   public void setHeadRadius(DoubleProperty headRadius)
   {
      this.headRadius = headRadius;
   }

   public void setHeadRadius(double headRadius)
   {
      setHeadRadius(new SimpleDoubleProperty(headRadius));
   }

   @Override
   public void clear()
   {
      position = null;
      orientation = null;
      bodyLength = null;
      bodyRadius = null;
      headLength = null;
      headRadius = null;
      color = null;
   }

   @Override
   public YoCoordinateSystemFX3D clone()
   {
      YoCoordinateSystemFX3D clone = new YoCoordinateSystemFX3D();
      clone.setName(getName());
      clone.setPosition(new Tuple3DProperty(position));
      clone.setOrientation(orientation.clone());
      clone.setBodyLength(bodyLength);
      clone.setBodyRadius(bodyRadius);
      clone.setHeadLength(headLength);
      clone.setHeadRadius(headRadius);
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

   public DoubleProperty getBodyLength()
   {
      return bodyLength;
   }

   public DoubleProperty getHeadLength()
   {
      return headLength;
   }

   public DoubleProperty getBodyRadius()
   {
      return bodyRadius;
   }

   public DoubleProperty getHeadRadius()
   {
      return headRadius;
   }

   @Override
   public Node getNode()
   {
      return coordinateSystemNode;
   }
}
