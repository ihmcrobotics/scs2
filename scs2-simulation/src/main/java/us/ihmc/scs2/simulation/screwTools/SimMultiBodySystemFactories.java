package us.ihmc.scs2.simulation.screwTools;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFixedJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPlanarJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRigidBody;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSixDoFJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSphericalJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimMultiBodySystemFactories
{
   public static class SimJointBuilder implements JointBuilder
   {
      public SimJointBuilder()
      {
      }

      @Override
      public SimJointBasics buildJoint(Class<? extends JointReadOnly> jointType,
                                       String name,
                                       RigidBodyBasics predecessor,
                                       RigidBodyTransformReadOnly transformToParent)
      {
         if (SixDoFJointReadOnly.class.isAssignableFrom(jointType))
            return buildSixDoFJoint(name, predecessor, transformToParent);
         if (PlanarJointReadOnly.class.isAssignableFrom(jointType))
            return buildPlanarJoint(name, predecessor, transformToParent);
         if (SphericalJointReadOnly.class.isAssignableFrom(jointType))
            return buildSphericalJoint(name, predecessor, transformToParent);
         if (FixedJointReadOnly.class.isAssignableFrom(jointType))
            return buildFixedJoint(name, predecessor, transformToParent);
         return null;
      }

      @Override
      public SimOneDoFJointBasics buildOneDoFJoint(Class<? extends OneDoFJointReadOnly> jointType,
                                                   String name,
                                                   RigidBodyBasics predecessor,
                                                   RigidBodyTransformReadOnly transformToParent,
                                                   Vector3DReadOnly jointAxis)
      {
         if (RevoluteJointBasics.class.isAssignableFrom(jointType))
            return buildRevoluteJoint(name, predecessor, transformToParent, jointAxis);
         if (PrismaticJointBasics.class.isAssignableFrom(jointType))
            return buildPrismaticJoint(name, predecessor, transformToParent, jointAxis);
         return null;
      }

      @Override
      public SimSixDoFJoint buildSixDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
      {
         return new SimSixDoFJoint(name, (SimRigidBodyBasics) predecessor, transformToParent);
      }

      @Override
      public SimPlanarJoint buildPlanarJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
      {
         return new SimPlanarJoint(name, (SimRigidBodyBasics) predecessor, transformToParent);
      }

      @Override
      public SimSphericalJoint buildSphericalJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
      {
         return new SimSphericalJoint(name, (SimRigidBodyBasics) predecessor, transformToParent);
      }

      @Override
      public SimRevoluteJoint buildRevoluteJoint(String name,
                                                 RigidBodyBasics predecessor,
                                                 RigidBodyTransformReadOnly transformToParent,
                                                 Vector3DReadOnly jointAxis)
      {
         return new SimRevoluteJoint(name, (SimRigidBodyBasics) predecessor, transformToParent, jointAxis);
      }

      @Override
      public SimPrismaticJoint buildPrismaticJoint(String name,
                                                   RigidBodyBasics predecessor,
                                                   RigidBodyTransformReadOnly transformToParent,
                                                   Vector3DReadOnly jointAxis)
      {
         return new SimPrismaticJoint(name, (SimRigidBodyBasics) predecessor, transformToParent, jointAxis);
      }

      @Override
      public SimFixedJoint buildFixedJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
      {
         return new SimFixedJoint(name, (SimRigidBodyBasics) predecessor, transformToParent);
      }
   }

   public static class SimRigidBodyBuilder implements RigidBodyBuilder
   {
      private final YoRegistry registry;

      private final YoRegistry secondaryRegistry;

      public SimRigidBodyBuilder(YoRegistry registry, YoRegistry secondaryRegistry)
      {
         this.registry = registry;
         this.secondaryRegistry = secondaryRegistry;
      }

      @Override
      public RigidBodyBasics buildRoot(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
      {
         return new SimRigidBody(bodyName, transformToParent, parentStationaryFrame, registry, secondaryRegistry);
      }

      @Override
      public RigidBodyBasics build(String bodyName,
                                   JointBasics parentJoint,
                                   Matrix3DReadOnly momentOfInertia,
                                   double mass,
                                   RigidBodyTransformReadOnly inertiaPose)
      {
         return new SimRigidBody(bodyName, (SimJointBasics) parentJoint, momentOfInertia, mass, inertiaPose);
      }
   }
}
