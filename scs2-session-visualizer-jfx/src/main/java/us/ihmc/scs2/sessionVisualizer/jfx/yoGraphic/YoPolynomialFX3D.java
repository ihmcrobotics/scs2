package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.stream.Stream;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.visual.SegmentedLine3DTriangleMeshFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class YoPolynomialFX3D extends YoGraphicFX3D
{
   private List<DoubleProperty> coefficientsX, coefficientsY, coefficientsZ;
   private IntegerProperty numberOfCoefficientsX, numberOfCoefficientsY, numberOfCoefficientsZ;
   private Property<ReferenceFrame> referenceFrame;
   private DoubleProperty startTime = new SimpleDoubleProperty(0.0);
   private DoubleProperty endTime;
   private DoubleProperty size = new SimpleDoubleProperty(0.01);
   private IntegerProperty timeResolution = new SimpleIntegerProperty(50);
   private IntegerProperty numberOfDivisions = new SimpleIntegerProperty(20);

   private final Affine affine = new Affine();
   private final PhongMaterial material = new PhongMaterial();
   private final Group polynomialNode = new Group();

   private Polynomial3DData newPolynomial = null;
   private Polynomial3DData oldPolynomial = null;
   private MeshView[] newMeshViews = null;

   public YoPolynomialFX3D()
   {
      polynomialNode.getTransforms().add(affine);
   }

   public YoPolynomialFX3D(ReferenceFrame worldFrame)
   {
      this();
   }

   @Override
   public void render()
   {
      if (referenceFrame == null || referenceFrame.getValue() == null || referenceFrame.getValue().isRootFrame())
      {
         affine.setToIdentity();
      }
      else
      {
         JavaFXMissingTools.toJavaFX(referenceFrame.getValue().getTransformToRoot(), affine);
      }

      newPolynomial = newPolynomial3D();

      if (color != null)
         material.setDiffuseColor(color.get());

      if (newMeshViews != null)
      {
         polynomialNode.getChildren().clear();
         polynomialNode.getChildren().addAll(newMeshViews);
         newMeshViews = null;
      }
   }

   private Polynomial3DData newPolynomial3D()
   {
      Polynomial3DData polynomial3D = new Polynomial3DData();

      if (YoGraphicTools.isAnyNull(coefficientsX, coefficientsY, coefficientsZ, startTime, endTime, size))
         return polynomial3D;

      if (EuclidCoreTools.epsilonEquals(startTime.get(), endTime.get(), 1.0e-5))
         return polynomial3D;

      polynomial3D.coefficientsX = toDoubleArray(coefficientsX, numberOfCoefficientsX);
      polynomial3D.coefficientsY = toDoubleArray(coefficientsY, numberOfCoefficientsY);
      polynomial3D.coefficientsZ = toDoubleArray(coefficientsZ, numberOfCoefficientsZ);
      polynomial3D.startTime = startTime.get();
      polynomial3D.endTime = endTime.get();
      polynomial3D.size = size.get();

      return polynomial3D;
   }

   @Override
   public void computeBackground()
   {
      Polynomial3DData newPolynomialLocal = newPolynomial;
      newPolynomial = null;

      if (newPolynomialLocal == null)
      {
         return;
      }
      else if (Double.isNaN(newPolynomialLocal.startTime) || Double.isNaN(newPolynomialLocal.endTime) || newPolynomialLocal.coefficientsX == null
            || newPolynomialLocal.coefficientsX.length == 0 || newPolynomialLocal.coefficientsY.length == 0 || newPolynomialLocal.coefficientsZ.length == 0)
      {
         newMeshViews = new MeshView[0];
         return;
      }

      if (newPolynomialLocal.equals(oldPolynomial) && !polynomialNode.getChildren().isEmpty())
         return;

      int timeResolution = this.timeResolution.get();
      int numberOfDivisions = this.numberOfDivisions.get();

      Point3D[] positions = new Point3D[timeResolution];
      Vector3D[] velocities = new Vector3D[timeResolution];

      for (int i = 0; i < timeResolution; i++)
      {
         double alpha = (double) i / (timeResolution - 1.0);
         double time = EuclidCoreTools.interpolate(newPolynomialLocal.startTime, newPolynomialLocal.endTime, alpha);
         positions[i] = new Point3D();
         velocities[i] = new Vector3D();

         computeAt(time, positions[i]::setX, velocities[i]::setX, newPolynomialLocal.coefficientsX);
         computeAt(time, positions[i]::setY, velocities[i]::setY, newPolynomialLocal.coefficientsY);
         computeAt(time, positions[i]::setZ, velocities[i]::setZ, newPolynomialLocal.coefficientsZ);
      }

      SegmentedLine3DTriangleMeshFactory meshGenerator = new SegmentedLine3DTriangleMeshFactory(timeResolution, numberOfDivisions, newPolynomialLocal.size);
      meshGenerator.compute(positions, velocities);
      Mesh[] meshes = Stream.of(meshGenerator.getTriangleMesh3DDefinitions())
                            .map(JavaFXTriangleMesh3DDefinitionInterpreter::interpretDefinition)
                            .toArray(Mesh[]::new);

      MeshView[] meshViews = new MeshView[meshes.length];

      for (int i = 0; i < meshes.length; i++)
      {
         meshViews[i] = new MeshView(meshes[i]);
         meshViews[i].setMaterial(material);
         meshViews[i].idProperty().bind(nameProperty().concat(" (").concat(Integer.toString(i)).concat(")"));
      }

      oldPolynomial = newPolynomialLocal;
      newMeshViews = meshViews;
   }

   private static class Polynomial3DData
   {
      private double[] coefficientsX, coefficientsY, coefficientsZ;
      private double startTime = Double.NaN, endTime = Double.NaN;
      private double size = Double.NaN;

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof Polynomial3DData)
         {
            Polynomial3DData other = (Polynomial3DData) object;

            if (!Arrays.equals(coefficientsX, other.coefficientsX))
               return false;
            if (!Arrays.equals(coefficientsY, other.coefficientsY))
               return false;
            if (!Arrays.equals(coefficientsZ, other.coefficientsZ))
               return false;
            if (startTime != other.startTime)
               return false;
            if (endTime != other.endTime)
               return false;
            if (size != other.size)
               return false;

            return true;
         }
         else
         {
            return false;
         }
      }
   }

   private static double[] toDoubleArray(List<DoubleProperty> list, IntegerProperty size)
   {
      if (size != null && size.get() < list.size())
         list = list.subList(0, size.get());

      return list.stream().mapToDouble(DoubleProperty::get).toArray();
   }

   private static void computeAt(double time, DoubleConsumer positionConsumer, DoubleConsumer velocityConsumer, double[] coefficients)
   {
      double[] timePowers = computeTimePowers(time, coefficients.length);
      double position = 0.0;
      double velocity = 0.0;

      for (int i = 0; i < coefficients.length; i++)
         position += timePowers[i] * coefficients[i];
      for (int i = 1; i < coefficients.length; i++)
         velocity += i * timePowers[i - 1] * coefficients[i];

      positionConsumer.accept(position);
      velocityConsumer.accept(velocity);
   }

   private static double[] computeTimePowers(double t, int n)
   {
      if (n == 0)
         return new double[0];

      double[] timePowers = new double[n];

      timePowers[0] = 1.0;

      for (int i = 1; i < n; i++)
         timePowers[i] = timePowers[i - 1] * t;

      return timePowers;
   }

   public void setCoefficientsX(List<DoubleProperty> coefficientsX)
   {
      this.coefficientsX = coefficientsX;
   }

   public void setCoefficientsY(List<DoubleProperty> coefficientsY)
   {
      this.coefficientsY = coefficientsY;
   }

   public void setCoefficientsZ(List<DoubleProperty> coefficientsZ)
   {
      this.coefficientsZ = coefficientsZ;
   }

   public void setNumberOfCoefficientsX(IntegerProperty numberOfCoefficientsX)
   {
      this.numberOfCoefficientsX = numberOfCoefficientsX;
   }

   public void setNumberOfCoefficientsX(int numberOfCoefficientsX)
   {
      setNumberOfCoefficientsX(new SimpleIntegerProperty(numberOfCoefficientsX));
   }

   public void setNumberOfCoefficientsY(IntegerProperty numberOfCoefficientsY)
   {
      this.numberOfCoefficientsY = numberOfCoefficientsY;
   }

   public void setNumberOfCoefficientsY(int numberOfCoefficientsY)
   {
      setNumberOfCoefficientsY(new SimpleIntegerProperty(numberOfCoefficientsY));
   }

   public void setNumberOfCoefficientsZ(IntegerProperty numberOfCoefficientsZ)
   {
      this.numberOfCoefficientsZ = numberOfCoefficientsZ;
   }

   public void setNumberOfCoefficientsZ(int numberOfCoefficientsZ)
   {
      setNumberOfCoefficientsZ(new SimpleIntegerProperty(numberOfCoefficientsZ));
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(new SimpleObjectProperty<>(referenceFrame));
   }

   public void setReferenceFrame(Property<ReferenceFrame> referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void setStartTime(DoubleProperty startTime)
   {
      this.startTime = startTime;
   }

   public void setStartTime(double startTime)
   {
      setStartTime(new SimpleDoubleProperty(startTime));
   }

   public void setEndTime(DoubleProperty endTime)
   {
      this.endTime = endTime;
   }

   public void setEndTime(double endTime)
   {
      setEndTime(new SimpleDoubleProperty(endTime));
   }

   public void setSize(DoubleProperty size)
   {
      this.size = size;
   }

   public void setSize(double size)
   {
      setSize(new SimpleDoubleProperty(size));
   }

   public void setTimeResolution(int timeResolution)
   {
      setTimeResolution(new SimpleIntegerProperty(timeResolution));
   }

   public void setTimeResolution(IntegerProperty timeResolution)
   {
      this.timeResolution = timeResolution;
   }

   public void setNumberOfDivisions(int numberOfDivisions)
   {
      setNumberOfDivisions(new SimpleIntegerProperty(numberOfDivisions));
   }

   public void setNumberOfDivisions(IntegerProperty numberOfDivisions)
   {
      this.numberOfDivisions = numberOfDivisions;
   }

   @Override
   public void clear()
   {
      coefficientsX = null;
      coefficientsY = null;
      coefficientsZ = null;
      numberOfCoefficientsX = null;
      numberOfCoefficientsY = null;
      numberOfCoefficientsZ = null;
      referenceFrame = null;
      startTime = null;
      endTime = null;
      size = null;
      color = null;
   }

   @Override
   public YoGraphicFX clone()
   {
      YoPolynomialFX3D clone = new YoPolynomialFX3D();
      clone.setName(getName());
      clone.setCoefficientsX(new ArrayList<>(coefficientsX));
      clone.setCoefficientsY(new ArrayList<>(coefficientsY));
      clone.setCoefficientsZ(new ArrayList<>(coefficientsZ));
      clone.setNumberOfCoefficientsX(numberOfCoefficientsX);
      clone.setNumberOfCoefficientsY(numberOfCoefficientsY);
      clone.setNumberOfCoefficientsZ(numberOfCoefficientsZ);
      clone.setReferenceFrame(referenceFrame);
      clone.setStartTime(startTime);
      clone.setEndTime(endTime);
      clone.setSize(size);
      clone.setTimeResolution(timeResolution);
      clone.setNumberOfDivisions(numberOfDivisions);
      clone.setColor(color);
      return clone;
   }

   public List<DoubleProperty> getCoefficientsX()
   {
      return coefficientsX;
   }

   public List<DoubleProperty> getCoefficientsY()
   {
      return coefficientsY;
   }

   public List<DoubleProperty> getCoefficientsZ()
   {
      return coefficientsZ;
   }

   public IntegerProperty getNumberOfCoefficientsX()
   {
      return numberOfCoefficientsX;
   }

   public IntegerProperty getNumberOfCoefficientsY()
   {
      return numberOfCoefficientsY;
   }

   public IntegerProperty getNumberOfCoefficientsZ()
   {
      return numberOfCoefficientsZ;
   }

   public Property<ReferenceFrame> getReferenceFrame()
   {
      return referenceFrame;
   }

   public DoubleProperty getStartTime()
   {
      return startTime;
   }

   public DoubleProperty getEndTime()
   {
      return endTime;
   }

   public DoubleProperty getSize()
   {
      return size;
   }

   public IntegerProperty getTimeResolution()
   {
      return timeResolution;
   }

   public IntegerProperty getNumberOfDivisions()
   {
      return numberOfDivisions;
   }

   @Override
   public Node getNode()
   {
      return polynomialNode;
   }
}
