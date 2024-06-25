package us.ihmc.scs2.simulation.collision;

import org.apache.commons.io.FilenameUtils;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePointShape3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.Torus3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.shapes.FrameSTPBox3D;
import us.ihmc.scs2.simulation.shapes.FrameSTPCapsule3D;
import us.ihmc.scs2.simulation.shapes.FrameSTPCylinder3D;
import us.ihmc.scs2.simulation.shapes.FrameSTPRamp3D;
import us.ihmc.scs2.simulation.shapes.STPBox3D;
import us.ihmc.scs2.simulation.shapes.STPCapsule3D;
import us.ihmc.scs2.simulation.shapes.STPCylinder3D;
import us.ihmc.scs2.simulation.shapes.STPRamp3D;

import java.net.URL;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class CollisionTools
{
   private static final int CONE_NUMBER_OF_DIVISIONS = 64;

   public static List<Collidable> extractCollidableRigidBodies(RobotDefinition robotDefinition, SimRigidBodyBasics rootBody)
   {
      return rootBody.subtreeStream()
                     .flatMap(rigidBody -> toCollidableRigidBody(robotDefinition.getRigidBodyDefinition(rigidBody.getName()), rigidBody).stream())
                     .collect(Collectors.toList());
   }

   public static List<Collidable> toCollidableRigidBody(RigidBodyDefinition definition, SimRigidBodyBasics rigidBodyInstance)
   {
      return definition.getCollisionShapeDefinitions()
                       .stream()
                       .map(collisionShapeDefinition -> toCollidable(collisionShapeDefinition, rigidBodyInstance))
                       .filter(Objects::nonNull)
                       .collect(Collectors.toList());
   }

   private static Collidable toCollidable(CollisionShapeDefinition definition, SimRigidBodyBasics rigidBody)
   {
      ReferenceFrame shapeFrame = rigidBody.isRootBody() ? rigidBody.getBodyFixedFrame() : rigidBody.getParentJoint().getFrameAfterJoint();

      FrameShape3DReadOnly shape = toFrameShape3D(definition.getOriginPose(), shapeFrame, definition.getGeometryDefinition());

      if (shape == null)
         return null;

      long collisionMask = definition.getCollisionMask();
      long collisionGroup = definition.getCollisionGroup();

      return new Collidable(rigidBody, collisionMask, collisionGroup, shape);
   }

   public static List<Collidable> toCollisionShape(TerrainObjectDefinition definition, ReferenceFrame worldFrame)
   {
      return definition.getCollisionShapeDefinitions()
                       .stream()
                       .map(collisionShapeDefinition -> toStaticCollidable(collisionShapeDefinition, worldFrame))
                       .collect(Collectors.toList());
   }

   private static Collidable toStaticCollidable(CollisionShapeDefinition definition, ReferenceFrame worldFrame)
   {
      FrameShape3DReadOnly shape = toFrameShape3D(definition.getOriginPose(), worldFrame, definition.getGeometryDefinition());

      long collisionMask = definition.getCollisionMask();
      long collisionGroup = definition.getCollisionGroup();

      return new Collidable(null, collisionMask, collisionGroup, shape);
   }

   public static Shape3DReadOnly toShape3D(RigidBodyTransformReadOnly originPose, GeometryDefinition definition)
   {
      if (definition instanceof STPBox3DDefinition)
         return toSTPBox3D(originPose, (STPBox3DDefinition) definition);
      else if (definition instanceof Box3DDefinition)
         return toBox3D(originPose, (Box3DDefinition) definition);
      else if (definition instanceof STPCapsule3DDefinition)
         return toSTPCapsule3D(originPose, (STPCapsule3DDefinition) definition);
      else if (definition instanceof Capsule3DDefinition)
         return toCapsule3D(originPose, (Capsule3DDefinition) definition);
      else if (definition instanceof ConvexPolytope3DDefinition)
         return toConvexPolytope3D(originPose, (ConvexPolytope3DDefinition) definition);
      else if (definition instanceof Cone3DDefinition)
         return toConvexPolytope3D(originPose, (Cone3DDefinition) definition);
      else if (definition instanceof STPCylinder3DDefinition)
         return toSTPCylinder3D(originPose, (STPCylinder3DDefinition) definition);
      else if (definition instanceof Cylinder3DDefinition)
         return toCylinder3D(originPose, (Cylinder3DDefinition) definition);
      else if (definition instanceof Ellipsoid3DDefinition)
         return toEllipsoid3D(originPose, (Ellipsoid3DDefinition) definition);
      else if (definition instanceof Point3DDefinition)
         return toPointShape3D(originPose, (Point3DDefinition) definition);
      else if (definition instanceof Sphere3DDefinition)
         return toSphere3D(originPose, (Sphere3DDefinition) definition);
      else if (definition instanceof Torus3DDefinition)
         return toTorus3D(originPose, (Torus3DDefinition) definition);
      else if (definition instanceof STPRamp3DDefinition)
         return toSTPRamp3D(originPose, (STPRamp3DDefinition) definition);
      else if (definition instanceof Ramp3DDefinition)
         return toRamp3D(originPose, (Ramp3DDefinition) definition);

      LogTools.warn("Unhandled geometry type: " + definition.getClass().getSimpleName());
      return null;
   }

   public static FrameShape3DReadOnly toFrameShape3D(RigidBodyTransformReadOnly originPose, ReferenceFrame referenceFrame, GeometryDefinition definition)
   {
      if (definition instanceof STPBox3DDefinition stpBox3DDefinition)
         return new FrameSTPBox3D(referenceFrame, toSTPBox3D(originPose, stpBox3DDefinition));
      if (definition instanceof Box3DDefinition box3DDefinition)
         return new FrameBox3D(referenceFrame, toBox3D(originPose, box3DDefinition));
      else if (definition instanceof STPCapsule3DDefinition stpCapsule3DDefinition)
         return new FrameSTPCapsule3D(referenceFrame, toSTPCapsule3D(originPose, stpCapsule3DDefinition));
      else if (definition instanceof Capsule3DDefinition capsule3DDefinition)
         return new FrameCapsule3D(referenceFrame, toCapsule3D(originPose, capsule3DDefinition));
      else if (definition instanceof ConvexPolytope3DDefinition convexPolytope3DDefinition)
         return new FrameConvexPolytope3D(referenceFrame, toConvexPolytope3D(originPose, convexPolytope3DDefinition));
      else if (definition instanceof Cone3DDefinition cone3DDefinition)
         return new FrameConvexPolytope3D(referenceFrame, toConvexPolytope3D(originPose, cone3DDefinition));
      else if (definition instanceof STPCylinder3DDefinition stpCylinder3DDefinition)
         return new FrameSTPCylinder3D(referenceFrame, toSTPCylinder3D(originPose, stpCylinder3DDefinition));
      else if (definition instanceof Cylinder3DDefinition cylinder3DDefinition)
         return new FrameCylinder3D(referenceFrame, toCylinder3D(originPose, cylinder3DDefinition));
      else if (definition instanceof Ellipsoid3DDefinition ellipsoid3DDefinition)
         return new FrameEllipsoid3D(referenceFrame, toEllipsoid3D(originPose, ellipsoid3DDefinition));
      else if (definition instanceof Point3DDefinition point3DDefinition)
         return new FramePointShape3D(referenceFrame, toPointShape3D(originPose, point3DDefinition));
      else if (definition instanceof Sphere3DDefinition sphere3DDefinition)
         return new FrameSphere3D(referenceFrame, toSphere3D(originPose, sphere3DDefinition));
      else if (definition instanceof STPRamp3DDefinition stpRamp3DDefinition)
         return new FrameSTPRamp3D(referenceFrame, toSTPRamp3D(originPose, stpRamp3DDefinition));
      else if (definition instanceof Ramp3DDefinition ramp3DDefinition)
         return new FrameRamp3D(referenceFrame, toRamp3D(originPose, ramp3DDefinition));
      else if (definition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
         return new FrameConvexPolytope3D(referenceFrame, toConvexPolytope3D(originPose, modelFileGeometryDefinition));

      LogTools.warn("Unhandled geometry type: " + definition.getClass().getSimpleName());
      return null;
   }

   public static STPBox3D toSTPBox3D(RigidBodyTransformReadOnly originPose, STPBox3DDefinition definition)
   {
      STPBox3D stpBox3D = new STPBox3D();
      stpBox3D.getSize().set(definition.getSizeX(), definition.getSizeY(), definition.getSizeZ());
      stpBox3D.setMargins(definition.getMinimumMargin(), definition.getMaximumMargin());
      if (originPose != null)
         stpBox3D.getPose().set(originPose);
      return stpBox3D;
   }

   public static Box3D toBox3D(RigidBodyTransformReadOnly originPose, Box3DDefinition definition)
   {
      Box3D box3D = new Box3D();
      box3D.getSize().set(definition.getSizeX(), definition.getSizeY(), definition.getSizeZ());
      if (originPose != null)
         box3D.getPose().set(originPose);
      return box3D;
   }

   public static STPCapsule3D toSTPCapsule3D(RigidBodyTransformReadOnly originPose, STPCapsule3DDefinition definition)
   {
      if (!definition.isRegular())
         throw new UnsupportedOperationException("Only regular capsules are supported.");
      STPCapsule3D stpCapsule3D = new STPCapsule3D();
      stpCapsule3D.setSize(definition.getLength(), definition.getRadiusX());
      stpCapsule3D.setMargins(definition.getMinimumMargin(), definition.getMaximumMargin());
      if (originPose != null)
         stpCapsule3D.applyTransform(originPose);
      return stpCapsule3D;
   }

   public static Capsule3D toCapsule3D(RigidBodyTransformReadOnly originPose, Capsule3DDefinition definition)
   {
      if (!definition.isRegular())
         throw new UnsupportedOperationException("Only regular capsules are supported.");
      Capsule3D capsule3D = new Capsule3D();
      capsule3D.setSize(definition.getLength(), definition.getRadiusX());
      if (originPose != null)
         capsule3D.applyTransform(originPose);
      return capsule3D;
   }

   public static ConvexPolytope3D toConvexPolytope3D(RigidBodyTransformReadOnly originPose, ConvexPolytope3DDefinition definition)
   {
      ConvexPolytope3D cone3D = new ConvexPolytope3D(definition.getConvexPolytope());
      if (originPose != null)
         cone3D.applyTransform(originPose);
      return cone3D;
   }

   public static ConvexPolytope3D toConvexPolytope3D(RigidBodyTransformReadOnly originPose, Cone3DDefinition definition)
   {
      ConvexPolytope3D cone3D = EuclidPolytopeFactories.newCone(definition.getHeight(), definition.getRadius(), CONE_NUMBER_OF_DIVISIONS);
      if (originPose != null)
         cone3D.applyTransform(originPose);
      return cone3D;
   }

   public static STPCylinder3D toSTPCylinder3D(RigidBodyTransformReadOnly originPose, STPCylinder3DDefinition definition)
   {
      STPCylinder3D stpCylinder3D = new STPCylinder3D();
      stpCylinder3D.setSize(definition.getLength(), definition.getRadius());
      stpCylinder3D.setMargins(definition.getMinimumMargin(), definition.getMaximumMargin());
      if (originPose != null)
         stpCylinder3D.applyTransform(originPose);
      return stpCylinder3D;
   }

   public static Cylinder3D toCylinder3D(RigidBodyTransformReadOnly originPose, Cylinder3DDefinition definition)
   {
      Cylinder3D cylinder3D = new Cylinder3D();
      cylinder3D.setSize(definition.getLength(), definition.getRadius());
      if (originPose != null)
         cylinder3D.applyTransform(originPose);
      return cylinder3D;
   }

   public static Ellipsoid3D toEllipsoid3D(RigidBodyTransformReadOnly originPose, Ellipsoid3DDefinition definition)
   {
      Ellipsoid3D ellipsoid3D = new Ellipsoid3D();
      ellipsoid3D.getRadii().set(definition.getRadiusX(), definition.getRadiusY(), definition.getRadiusZ());
      if (originPose != null)
         ellipsoid3D.getPose().set(originPose);
      return ellipsoid3D;
   }

   public static PointShape3D toPointShape3D(RigidBodyTransformReadOnly originPose, Point3DDefinition definition)
   {
      PointShape3D pointShape3D = new PointShape3D(definition.getPosition());
      if (originPose != null)
         pointShape3D.applyTransform(originPose);
      return pointShape3D;
   }

   public static Sphere3D toSphere3D(RigidBodyTransformReadOnly originPose, Sphere3DDefinition definition)
   {
      Sphere3D sphere3D = new Sphere3D();
      sphere3D.setRadius(definition.getRadius());
      if (originPose != null)
         sphere3D.applyTransform(originPose);
      return sphere3D;
   }

   public static Torus3D toTorus3D(RigidBodyTransformReadOnly originPose, Torus3DDefinition definition)
   {
      Torus3D torus3D = new Torus3D();
      double tubeRadius = 0.5 * (definition.getMajorRadius() - definition.getMinorRadius());
      double radius = definition.getMinorRadius() + tubeRadius;
      torus3D.setRadii(radius, tubeRadius);
      if (originPose != null)
         torus3D.applyTransform(originPose);
      return torus3D;
   }

   public static STPRamp3D toSTPRamp3D(RigidBodyTransformReadOnly originPose, STPRamp3DDefinition definition)
   {
      STPRamp3D stpRamp3D = new STPRamp3D();
      stpRamp3D.getSize().set(definition.getSizeX(), definition.getSizeY(), definition.getSizeZ());
      stpRamp3D.setMargins(definition.getMinimumMargin(), definition.getMaximumMargin());
      if (originPose != null)
         stpRamp3D.getPose().set(originPose);
      // Ramp3DDefinition assume the origin at the center of the bottom face while Euclid places it at the bottom of the ramp.
      stpRamp3D.getPose().appendTranslation(-0.5 * definition.getSizeX(), 0.0, 0.0);
      return stpRamp3D;
   }

   public static Ramp3D toRamp3D(RigidBodyTransformReadOnly originPose, Ramp3DDefinition definition)
   {
      Ramp3D ramp3D = new Ramp3D();
      ramp3D.getSize().set(definition.getSizeX(), definition.getSizeY(), definition.getSizeZ());
      if (originPose != null)
         ramp3D.getPose().set(originPose);
      // Ramp3DDefinition assume the origin at the center of the bottom face while Euclid places it at the bottom of the ramp.
      ramp3D.getPose().appendTranslation(-0.5 * definition.getSizeX(), 0.0, 0.0);
      return ramp3D;
   }

   public static ConvexPolytope3D toConvexPolytope3D(RigidBodyTransformReadOnly originPose, ModelFileGeometryDefinition definition)
   {
      if (!FilenameUtils.isExtension(definition.getFileName(), "obj"))
         throw new UnsupportedOperationException("Only Wavefront OBJ files are supported.");

      URL objFileURL = DefinitionIOTools.resolveModelFileURL(definition);
      List<Point3D> vertices = DefinitionIOTools.loadOBJVertices(objFileURL);
      if (!originPose.hasRotation())
      {
         if (originPose.hasTranslation())
            vertices.forEach(vertex -> vertex.add(originPose.getTranslation()));
      }
      else if (!originPose.hasTranslation())
      {
         vertices.forEach(originPose::transform);
      }
      else
      {
         vertices.forEach(vertex -> originPose.getRotation().transform(vertex));
      }
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));
   }
}
