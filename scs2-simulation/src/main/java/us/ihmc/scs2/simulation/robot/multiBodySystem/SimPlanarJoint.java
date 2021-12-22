package us.ihmc.scs2.simulation.robot.multiBodySystem;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoPlanarJoint;
import us.ihmc.mecano.yoVariables.tools.YoMecanoFactories;
import us.ihmc.scs2.definition.robot.PlanarJointDefinition;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimPlanarJoint extends YoPlanarJoint implements SimJointBasics, SimFloatingJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;
   private final FixedFrameTwistBasics jointDeltaTwist;

   public SimPlanarJoint(PlanarJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(), predecessor, definition.getTransformToParent());
   }

   public SimPlanarJoint(String name, SimRigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   public SimPlanarJoint(String name, SimRigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, predecessor, transformToParent, predecessor.getRegistry());
      registry = predecessor.getRegistry();
      auxiliaryData = new SimJointAuxiliaryData(this);

      YoDouble angularDeltaVelocityY = new YoDouble(name + "AngularDeltaVelocityY", registry);
      YoDouble linearDeltaVelocityX = new YoDouble(name + "LinearDeltaVelocityX", registry);
      YoDouble linearDeltaVelocityZ = new YoDouble(name + "LinearDeltaVelocityZ", registry);
      jointDeltaTwist = YoMecanoFactories.newPlanarYoFixedFrameTwistBasics(angularDeltaVelocityY,
                                                                           linearDeltaVelocityX,
                                                                           linearDeltaVelocityZ,
                                                                           afterJointFrame,
                                                                           beforeJointFrame,
                                                                           afterJointFrame);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public SimJointAuxiliaryData getAuxialiryData()
   {
      return auxiliaryData;
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      if (successor instanceof SimRigidBodyBasics)
         super.setSuccessor(successor);
      else
         throw new IllegalArgumentException("Can only set a " + SimRigidBodyBasics.class.getSimpleName() + " as successor of a " + getClass().getSimpleName());
   }

   @Override
   public SimRigidBodyBasics getPredecessor()
   {
      return (SimRigidBodyBasics) super.getPredecessor();
   }

   @Override
   public SimRigidBodyBasics getSuccessor()
   {
      return (SimRigidBodyBasics) super.getSuccessor();
   }

   @Override
   public FixedFrameTwistBasics getJointDeltaTwist()
   {
      return jointDeltaTwist;
   }

   @Override
   public int getJointDeltaVelocity(int rowStart, DMatrix matrixToPack)
   {
      matrixToPack.set(rowStart + 0, 0, getJointTwist().getAngularPartY());
      matrixToPack.set(rowStart + 1, 0, getJointTwist().getLinearPartX());
      matrixToPack.set(rowStart + 2, 0, getJointTwist().getLinearPartZ());
      return rowStart + getDegreesOfFreedom();
   }

   @Override
   public void setJointDeltaTwist(JointReadOnly other)
   {
      setJointDeltaTwist(MecanoTools.checkTypeAndCast(other, SimPlanarJoint.class));
   }

   public void setJointDeltaTwist(SimPlanarJoint other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherAngularDeltaVelocity = other.getJointDeltaTwist().getAngularPart();
      Vector3DReadOnly otherLinearDeltaVelocity = other.getJointDeltaTwist().getLinearPart();

      setJointAngularDeltaVelocity(otherAngularDeltaVelocity);
      setJointLinearDeltaVelocity(otherLinearDeltaVelocity);
   }

   @Override
   public int setJointDeltaVelocity(int rowStart, DMatrix matrix)
   {
      int index = rowStart;
      double qdRot = matrix.get(index++, 0);
      double xd = matrix.get(index++, 0);
      double zd = matrix.get(index++, 0);
      getJointDeltaTwist().setToZero();
      getJointDeltaTwist().setAngularPartY(qdRot);
      getJointDeltaTwist().getLinearPart().set(xd, 0.0, zd);
      return rowStart + getDegreesOfFreedom();
   }
}
