package us.ihmc.scs2.simulation.robot.multiBodySystem;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoCrossFourBarJoint;
import us.ihmc.scs2.definition.robot.CrossFourBarJointDefinition;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimCrossFourBarJoint extends YoCrossFourBarJoint implements SimOneDoFJointBasics, CrossFourBarJointBasics
{
   private final YoRegistry registry;
   private final SimJointAuxiliaryData auxiliaryData;

   private final TwistReadOnly jointDeltaTwist;
   private final YoDouble deltaQd;
   private final YoBoolean isPinned;

   public SimCrossFourBarJoint(CrossFourBarJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(),
           predecessor,
           definition.getJointNameA(),
           definition.getJointNameB(),
           definition.getJointNameC(),
           definition.getJointNameD(),
           definition.getBodyDA().getName(),
           definition.getBodyBC().getName(),
           definition.getTransformAToPredecessor(),
           definition.getTransformBToPredecessor(),
           definition.getTransformCToB(),
           definition.getTransformDToA(),
           definition.getBodyDA().getMomentOfInertia(),
           definition.getBodyBC().getMomentOfInertia(),
           definition.getBodyDA().getMass(),
           definition.getBodyBC().getMass(),
           definition.getBodyDA().getInertiaPose(),
           definition.getBodyBC().getInertiaPose(),
           definition.getActuatedJointIndex(),
           definition.getLoopClosureJointIndex(),
           definition.getAxis());
      setJointLimits(definition.getPositionLowerLimit(), definition.getPositionUpperLimit());
      setVelocityLimits(definition.getVelocityLowerLimit(), definition.getVelocityUpperLimit());
      setEffortLimits(definition.getEffortLowerLimit(), definition.getEffortUpperLimit());
   }

   public SimCrossFourBarJoint(String name,
                               SimRigidBodyBasics predecessor,
                               String jointNameA,
                               String jointNameB,
                               String jointNameC,
                               String jointNameD,
                               String bodyNameDA,
                               String bodyNameBC,
                               RigidBodyTransformReadOnly transformAToPredecessor,
                               RigidBodyTransformReadOnly transformBToPredecessor,
                               RigidBodyTransformReadOnly transformCToB,
                               RigidBodyTransformReadOnly transformDToA,
                               Matrix3DReadOnly bodyInertiaDA,
                               Matrix3DReadOnly bodyInertiaBC,
                               double bodyMassDA,
                               double bodyMassBC,
                               RigidBodyTransformReadOnly bodyInertiaPoseDA,
                               RigidBodyTransformReadOnly bodyInertiaPoseBC,
                               int actuatedJointIndex,
                               int loopClosureJointIndex,
                               Vector3DReadOnly jointAxis)
   {
      super(name,
            predecessor,
            jointNameA,
            jointNameB,
            jointNameC,
            jointNameD,
            bodyNameDA,
            bodyNameBC,
            transformAToPredecessor,
            transformBToPredecessor,
            transformDToA,
            transformCToB,
            bodyInertiaDA,
            bodyInertiaBC,
            bodyMassDA,
            bodyMassBC,
            bodyInertiaPoseDA,
            bodyInertiaPoseBC,
            actuatedJointIndex,
            loopClosureJointIndex,
            jointAxis,
            predecessor.getRegistry());

      this.registry = predecessor.getRegistry();
      auxiliaryData = new SimJointAuxiliaryData(this);
      deltaQd = new YoDouble("qd_delta_" + getName(), registry);
      jointDeltaTwist = MecanoFactories.newTwistReadOnly(this::getDeltaQd, getUnitJointTwist());
      isPinned = new YoBoolean("is_" + getName() + "_pinned", registry);
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
   public void setQ(double q)
   {
      if (!Double.isFinite(q))
         throw new IllegalStateException("Invalid joint configuration: " + q);

      super.setQ(q);
   }

   @Override
   public double computeActuatedJointQ(double q)
   {
      if (!Double.isFinite(q))
         throw new IllegalStateException("Invalid joint configuration: " + q);

      double actuatedJointQ = super.computeActuatedJointQ(q);

      if (!Double.isFinite(actuatedJointQ))
         throw new IllegalStateException("Invalid joint configuration: " + actuatedJointQ);

      return actuatedJointQ;
   }

   @Override
   public double getDeltaQd()
   {
      return deltaQd.getValue();
   }

   @Override
   public void setDeltaQd(double deltaQd)
   {
      this.deltaQd.set(deltaQd);
   }

   @Override
   public void setJointAngularDeltaVelocity(Vector3DReadOnly jointAngularDeltaVelocity)
   {
      setDeltaQd(getJointAxis().dot(jointAngularDeltaVelocity));
   }

   @Override
   public void setJointLinearDeltaVelocity(Vector3DReadOnly jointLinearDeltaVelocity)
   {
   }

   @Override
   public TwistReadOnly getJointDeltaTwist()
   {
      return jointDeltaTwist;
   }

   @Override
   public void setPinned(boolean isPinned)
   {
      this.isPinned.set(isPinned);
   }

   @Override
   public boolean isPinned()
   {
      return isPinned.getValue();
   }
}
