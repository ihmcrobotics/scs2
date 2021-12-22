package us.ihmc.scs2.definition.visual;

import java.awt.image.BufferedImage;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.ArcTorus3DDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrusionDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition.SubMeshDefinition;
import us.ihmc.scs2.definition.geometry.Polygon2DDefinition;
import us.ihmc.scs2.definition.geometry.Polygon3DDefinition;
import us.ihmc.scs2.definition.geometry.PyramidBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.TruncatedCone3DDefinition;

// TODO Needs major cleanup
public class VisualDefinitionFactory
{
   private static final MaterialDefinition DEFAULT_MATERIAL = new MaterialDefinition(ColorDefinitions.Black());

   private static final int RESOLUTION = 32;

   private final AffineTransform currentTransform = new AffineTransform();
   private final List<VisualDefinition> visualDefinitions = new ArrayList<>();

   public VisualDefinitionFactory()
   {
   }

   public List<VisualDefinition> getVisualDefinitions()
   {
      return visualDefinitions;
   }

   public void combine(VisualDefinitionFactory other)
   {
      identity();
      visualDefinitions.addAll(other.getVisualDefinitions());
   }

   public void identity()
   {
      currentTransform.setIdentity();
   }

   public void appendTransform(RigidBodyTransformReadOnly transform)
   {
      currentTransform.multiply(transform);
   }

   public void appendTranslation(double x, double y, double z)
   {
      currentTransform.appendTranslation(x, y, z);
   }

   public void appendTranslation(Tuple3DReadOnly translation)
   {
      currentTransform.appendTranslation(translation);
   }

   public void appendRotation(double rotationAngle, Vector3DReadOnly rotationAxis)
   {
      appendRotation(new AxisAngle(rotationAxis, rotationAngle));
   }

   public void appendRotation(Orientation3DReadOnly orientation)
   {
      currentTransform.appendOrientation(orientation);
   }

   public void appendScale(double scaleFactor)
   {
      currentTransform.appendScale(scaleFactor);
   }

   public void appendScale(Vector3DReadOnly scaleFactors)
   {
      currentTransform.appendScale(scaleFactors);
   }

   public void prependTransform(RigidBodyTransformReadOnly transform)
   {
      for (int i = 0; i < visualDefinitions.size(); i++)
         visualDefinitions.get(i).getOriginPose().preMultiply(transform);
      currentTransform.preMultiply(transform);
   }

   public void prependTranslation(double x, double y, double z)
   {
      for (int i = 0; i < visualDefinitions.size(); i++)
      {
         visualDefinitions.get(i).getOriginPose().prependTranslation(x, y, z);
      }
      currentTransform.prependTranslation(x, y, z);
   }

   public void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   public void prependRotation(double rotationAngle, Vector3DReadOnly rotationAxis)
   {
      prependRotation(new AxisAngle(rotationAxis, rotationAngle));
   }

   public void prependRotation(Orientation3DReadOnly orientation)
   {
      for (int i = 0; i < visualDefinitions.size(); i++)
         visualDefinitions.get(i).getOriginPose().prependOrientation(orientation);
      currentTransform.prependOrientation(orientation);
   }

   public void prependScale(double scaleFactor)
   {
      prependScale(new Vector3D(scaleFactor, scaleFactor, scaleFactor));
   }

   public void prependScale(Tuple3DReadOnly scaleFactors)
   {
      for (int i = 0; i < visualDefinitions.size(); i++)
         visualDefinitions.get(i).getOriginPose().prependScale(scaleFactors);
      currentTransform.prependScale(scaleFactors);
   }

   public VisualDefinition addVisualDefinition(VisualDefinition instruction)
   {
      visualDefinitions.add(instruction);
      return instruction;
   }

   public VisualDefinition addGeometryDefinition(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      return addVisualDefinition(new VisualDefinition(new AffineTransform(currentTransform), geometryDefinition, materialDefinition));
   }

   public VisualDefinition addModelFile(URL fileURL)
   {
      return addModelFile(fileURL, null);
   }

   public VisualDefinition addModelFile(URL fileURL, MaterialDefinition materialDefinition)
   {
      if (fileURL == null)
      {
         LogTools.error("fileURL == null in addModelFile");
         return null;
      }

      String fileName = fileURL.getFile();

      if (fileName == null || fileName.equals(""))
      {
         LogTools.error("Null File Name in add3DSFile");
         return null;
      }

      return addModelFile(fileName, materialDefinition);
   }

   public VisualDefinition addModelFile(String fileName)
   {
      return addModelFile(fileName, null);
   }

   public VisualDefinition addModelFile(String fileName, MaterialDefinition materialDefinition)
   {
      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition();
      modelFileGeometryDefinition.setFileName(fileName);
      return addGeometryDefinition(modelFileGeometryDefinition, materialDefinition);
   }

   public VisualDefinition addModelFile(String fileName,
                                        String submesh,
                                        boolean centerSubmesh,
                                        List<String> resourceDirectories,
                                        ClassLoader resourceClassLoader,
                                        MaterialDefinition materialDefinition)
   {
      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition(fileName);
      modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
      modelFileGeometryDefinition.setSubmeshes(Collections.singletonList(new SubMeshDefinition(submesh, centerSubmesh)));
      modelFileGeometryDefinition.setResourceClassLoader(resourceClassLoader);
      return addGeometryDefinition(modelFileGeometryDefinition, materialDefinition);
   }

   public VisualDefinition addModelFile(String fileName,
                                        List<String> resourceDirectories,
                                        ClassLoader resourceClassLoader,
                                        MaterialDefinition materialDefinition)
   {
      return addModelFile(fileName, null, false, resourceDirectories, resourceClassLoader, materialDefinition);
   }

   public void addCoordinateSystem(double length)
   {
      addCoordinateSystem(length, new MaterialDefinition(ColorDefinitions.Gray()));
   }

   public void addCoordinateSystem(double length, MaterialDefinition materialDefinition)
   {
      addCoordinateSystem(length,
                          new MaterialDefinition(ColorDefinitions.Red()),
                          new MaterialDefinition(ColorDefinitions.White()),
                          new MaterialDefinition(ColorDefinitions.Blue()),
                          materialDefinition);
   }

   public void addCoordinateSystem(double length,
                                   MaterialDefinition xAxisMaterial,
                                   MaterialDefinition yAxisMaterial,
                                   MaterialDefinition zAxisMaterial,
                                   MaterialDefinition arrowMaterial)
   {
      appendRotation(Math.PI / 2.0, Axis3D.Y);
      addArrow(length, xAxisMaterial, arrowMaterial);
      appendRotation(-Math.PI / 2.0, Axis3D.Y);
      appendRotation(-Math.PI / 2.0, Axis3D.X);
      addArrow(length, yAxisMaterial, arrowMaterial);
      appendRotation(Math.PI / 2.0, Axis3D.X);
      addArrow(length, zAxisMaterial, arrowMaterial);
   }

   public void add(Shape3DReadOnly shape, MaterialDefinition materialDefinition)
   {
      if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         appendTransform(box.getPose());
         addBox(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, materialDefinition);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         appendTranslation(capsule.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(capsule.getAxis()));
         addCapsule(capsule.getRadius(),
                    capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                    materialDefinition);
      }
      else if (shape instanceof Cylinder3DReadOnly)
      {
         Cylinder3DReadOnly cylinder = (Cylinder3DReadOnly) shape;
         appendTranslation(cylinder.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinder.getAxis()));
         appendTranslation(0.0, 0.0, -cylinder.getHalfLength());
         addCylinder(cylinder.getLength(), cylinder.getRadius(), materialDefinition);
      }
      else if (shape instanceof Ellipsoid3DReadOnly)
      {
         Ellipsoid3DReadOnly ellipsoid = (Ellipsoid3DReadOnly) shape;
         appendTransform(ellipsoid.getPose());
         addEllipsoid(ellipsoid.getRadiusX(), ellipsoid.getRadiusY(), ellipsoid.getRadiusZ(), materialDefinition);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         appendTranslation(pointShape);
         addSphere(0.005, materialDefinition); // Arbitrary radius
      }
      else if (shape instanceof Ramp3DReadOnly)
      {
         Ramp3DReadOnly ramp = (Ramp3DReadOnly) shape;
         appendTransform(ramp.getPose());
         appendTranslation(-0.5 * ramp.getSizeX(), 0.0, 0.0);
         addRamp(ramp.getSizeX(), ramp.getSizeY(), ramp.getSizeZ(), materialDefinition);
      }
      else if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         appendTranslation(sphere.getPosition());
         addSphere(sphere.getRadius(), materialDefinition);
      }
      else if (shape instanceof Torus3DReadOnly)
      {
         Torus3DReadOnly torus = (Torus3DReadOnly) shape;
         appendTranslation(torus.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(torus.getAxis()));
         addArcTorus(0.0, 2.0 * Math.PI, torus.getRadius(), torus.getTubeRadius(), materialDefinition);
      }
      else
      {
         // TODO Implement for ConvexPolytope3D
         throw new UnsupportedOperationException("Unsupported shape: " + shape);
      }
   }

   public void addArrow(double length, MaterialDefinition baseMaterial, MaterialDefinition headMaterial)
   {
      double coneHeight = 0.1 * length;
      double cylinderHeight = length - coneHeight;
      double radius = 0.02 * length;
      double coneRadius = 2.0 * radius;

      addCylinder(cylinderHeight, radius, baseMaterial);
      appendTranslation(0.0, 0.0, cylinderHeight);
      addCone(coneHeight, coneRadius, headMaterial);
      appendTranslation(0.0, 0.0, -cylinderHeight);
   }

   public VisualDefinition addBox(double lengthX, double widthY, double heightZ)
   {
      return addBox(lengthX, widthY, heightZ, DEFAULT_MATERIAL);
   }

   public VisualDefinition addBox(double lengthX, double widthY, double heightZ, MaterialDefinition materialDefinition)
   {
      return addBox(lengthX, widthY, heightZ, true, materialDefinition);
   }

   public VisualDefinition addBox(double lengthX, double widthY, double heightZ, boolean centeredInTheCenter, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Box3DDefinition(lengthX, widthY, heightZ, centeredInTheCenter), materialDefinition);
   }

   public VisualDefinition addRamp(double lengthX, double widthY, double heightZ)
   {
      return addRamp(lengthX, widthY, heightZ, DEFAULT_MATERIAL);
   }

   public VisualDefinition addRamp(double lengthX, double widthY, double heightZ, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Ramp3DDefinition(lengthX, widthY, heightZ), materialDefinition);
   }

   public VisualDefinition addSphere(double radius)
   {
      return addSphere(radius, DEFAULT_MATERIAL);
   }

   public VisualDefinition addSphere(double radius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Sphere3DDefinition(radius, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addCapsule(double radius, double height)
   {
      return addCapsule(radius, height, DEFAULT_MATERIAL);
   }

   public VisualDefinition addCapsule(double radius, double height, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Capsule3DDefinition(height, radius, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addEllipsoid(double xRadius, double yRadius, double zRadius)
   {
      return addEllipsoid(xRadius, yRadius, zRadius, DEFAULT_MATERIAL);
   }

   public VisualDefinition addEllipsoid(double xRadius, double yRadius, double zRadius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Ellipsoid3DDefinition(xRadius, yRadius, zRadius, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addCylinder(double height, double radius)
   {
      return addCylinder(height, radius, DEFAULT_MATERIAL);
   }

   public VisualDefinition addCylinder(double height, double radius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Cylinder3DDefinition(height, radius, false, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addCone(double height, double radius)
   {
      return addCone(height, radius, DEFAULT_MATERIAL);
   }

   public VisualDefinition addCone(double height, double radius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Cone3DDefinition(height, radius, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addGenTruncatedCone(double height, double bx, double by, double tx, double ty)
   {
      return addGenTruncatedCone(height, bx, by, tx, ty, DEFAULT_MATERIAL);
   }

   public VisualDefinition addGenTruncatedCone(double height, double bx, double by, double tx, double ty, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new TruncatedCone3DDefinition(height, tx, ty, bx, by, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addHemiEllipsoid(double xRad, double yRad, double zRad)
   {
      return addHemiEllipsoid(xRad, yRad, zRad, DEFAULT_MATERIAL);
   }

   public VisualDefinition addHemiEllipsoid(double xRad, double yRad, double zRad, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new HemiEllipsoid3DDefinition(xRad, yRad, zRad, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius)
   {
      return addArcTorus(startAngle, endAngle, majorRadius, minorRadius, DEFAULT_MATERIAL);
   }

   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ArcTorus3DDefinition(startAngle, endAngle, majorRadius, minorRadius, RESOLUTION), materialDefinition);
   }

   public VisualDefinition addPyramidCube(double lx, double ly, double lz, double lh)
   {
      return addPyramidCube(lx, ly, lz, lh, DEFAULT_MATERIAL);
   }

   public VisualDefinition addPyramidCube(double lx, double ly, double lz, double lh, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new PyramidBox3DDefinition(lx, ly, lz, lh), materialDefinition);
   }

   public VisualDefinition addPolygon(List<? extends Point3DReadOnly> polygonPoints)
   {
      return addPolygon(polygonPoints, DEFAULT_MATERIAL);
   }

   public VisualDefinition addPolygon(List<? extends Point3DReadOnly> polygonPoints, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Polygon3DDefinition(Polygon3DDefinition.toPoint3DDefinitionList(polygonPoints), true), materialDefinition);
   }

   public VisualDefinition addPolygon(ConvexPolygon2DReadOnly convexPolygon2d, MaterialDefinition materialDefinition)
   {
      List<Point3D> polygonPoints = new ArrayList<>();
      int numPoints = convexPolygon2d.getNumberOfVertices();

      for (int i = 0; i < numPoints; i++)
      {
         Point2DReadOnly planarPoint = convexPolygon2d.getVertex(i);
         polygonPoints.add(new Point3D(planarPoint.getX(), planarPoint.getY(), 0.0));
      }

      return addPolygon(polygonPoints, materialDefinition);
   }

   public VisualDefinition addPolygon(ConvexPolygon2DReadOnly convexPolygon2d)
   {
      return addPolygon(convexPolygon2d, DEFAULT_MATERIAL);
   }

   public void addPolygons(RigidBodyTransformReadOnly transform, List<? extends ConvexPolygon2DReadOnly> convexPolygon2D)
   {
      addPolygons(transform, convexPolygon2D, DEFAULT_MATERIAL);
   }

   public void addPolygons(RigidBodyTransformReadOnly transform, List<? extends ConvexPolygon2DReadOnly> convexPolygon2D, MaterialDefinition materialDefinition)
   {
      appendTransform(transform);

      for (int i = 0; i < convexPolygon2D.size(); i++)
      {
         ConvexPolygon2DReadOnly convexPolygon = convexPolygon2D.get(i);
         addGeometryDefinition(new Polygon2DDefinition(Polygon2DDefinition.toPoint2DDefinitionList(convexPolygon.getPolygonVerticesView()),
                                                       !convexPolygon.isClockwiseOrdered()),
                               materialDefinition);
      }

      RigidBodyTransform transformLocal = new RigidBodyTransform(transform);
      transformLocal.invert();
      appendTransform(transformLocal);
   }

   public VisualDefinition addPolygon(Point3DReadOnly[] polygonPoint)
   {
      return addPolygon(polygonPoint, DEFAULT_MATERIAL);
   }

   public VisualDefinition addPolygon(Point3DReadOnly[] polygonPoints, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Polygon3DDefinition(Polygon3DDefinition.toPoint3DDefinitionList(polygonPoints), true), materialDefinition);
   }

   public VisualDefinition addPolygon(MaterialDefinition materialDefinition, Point3DReadOnly... polygonPoints)
   {
      return addPolygon(polygonPoints, materialDefinition);
   }

   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2d, double height)
   {
      return addExtrudedPolygon(convexPolygon2d, height, DEFAULT_MATERIAL);
   }

   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2d, double height, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrudedPolygon2DDefinition(ExtrudedPolygon2DDefinition.toPoint2DDefinitionList(convexPolygon2d.getPolygonVerticesView()),
                                                                   true,
                                                                   height),
                                   materialDefinition);
   }

   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints, double height)
   {
      return addExtrudedPolygon(polygonPoints, height, DEFAULT_MATERIAL);
   }

   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints, double height, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrudedPolygon2DDefinition(ExtrudedPolygon2DDefinition.toPoint2DDefinitionList(polygonPoints), true, height),
                                   materialDefinition);
   }

   public VisualDefinition addExtrusion(BufferedImage bufferedImageToExtrude, double thickness, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrusionDefinition(bufferedImageToExtrude, thickness), materialDefinition);
   }

   public VisualDefinition addText(String text, double thickness, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrusionDefinition(text, thickness), materialDefinition);
   }
}
