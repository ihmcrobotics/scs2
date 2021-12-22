package us.ihmc.scs2.simulation.screwTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
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

public class SimMultiBodySystemRandomTools
{
   /**
    * Generates a random state and update the given {@code joint} with it.
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized.
    * @param joint            the joint to set the state of. Modified.
    */
   public static void nextState(Random random, SimJointStateType stateToRandomize, SimJointBasics joint)
   {
      switch (stateToRandomize)
      {
         case CONFIGURATION:
            joint.setJointOrientation(EuclidCoreRandomTools.nextQuaternion(random));
            joint.setJointPosition(EuclidCoreRandomTools.nextVector3D(random));
            break;
         case VELOCITY:
            joint.setJointAngularVelocity(EuclidCoreRandomTools.nextVector3D(random));
            joint.setJointLinearVelocity(EuclidCoreRandomTools.nextVector3D(random));
            break;
         case VELOCITY_CHANGE:
            joint.setJointAngularDeltaVelocity(EuclidCoreRandomTools.nextVector3D(random));
            joint.setJointLinearDeltaVelocity(EuclidCoreRandomTools.nextVector3D(random));
            break;
         case ACCELERATION:
            joint.setJointAngularAcceleration(EuclidCoreRandomTools.nextVector3D(random));
            joint.setJointLinearAcceleration(EuclidCoreRandomTools.nextVector3D(random));
            break;
         case EFFORT:
            joint.setJointTorque(EuclidCoreRandomTools.nextVector3D(random));
            joint.setJointForce(EuclidCoreRandomTools.nextVector3D(random));
            break;
         default:
            throw new RuntimeException("Unhandled state selection: " + stateToRandomize);
      }
   }

   /**
    * Generates a random state and update the given {@code joints} with it.
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized.
    * @param joints           the joints to set the state of. Modified.
    */
   public static void nextState(Random random, SimJointStateType stateToRandomize, SimJointBasics[] joints)
   {
      for (SimJointBasics joint : joints)
         nextState(random, stateToRandomize, joint);
   }

   /**
    * Generates a random state and update the given {@code joints} with it.
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized.
    * @param joints           the joints to set the state of. Modified.
    */
   public static void nextState(Random random, SimJointStateType stateToRandomize, Iterable<? extends SimJointBasics> joints)
   {
      joints.forEach(joint -> nextState(random, stateToRandomize, joint));
   }

   /**
    * Generates a random state and update the given {@code joint} with it.
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized.
    * @param min              the minimum value for the generated random value.
    * @param max              the maximum value for the generated random value.
    * @param joint            the joints to set the state of. Modified.
    */
   public static void nextState(Random random, SimJointStateType stateToRandomize, double min, double max, SimOneDoFJointBasics joint)
   {
      switch (stateToRandomize)
      {
         case CONFIGURATION:
            joint.setQ(EuclidCoreRandomTools.nextDouble(random, min, max));
            break;
         case VELOCITY:
            joint.setQd(EuclidCoreRandomTools.nextDouble(random, min, max));
            break;
         case VELOCITY_CHANGE:
            joint.setDeltaQd(EuclidCoreRandomTools.nextDouble(random, min, max));
            break;
         case ACCELERATION:
            joint.setQdd(EuclidCoreRandomTools.nextDouble(random, min, max));
            break;
         case EFFORT:
            joint.setTau(EuclidCoreRandomTools.nextDouble(random, min, max));
            break;
         default:
            throw new RuntimeException("Unhandled state selection: " + stateToRandomize);
      }
   }

   /**
    * Generates a random state and update the given {@code joints} with it.
    * <p>
    * The random state is guaranteed to be within the joint limits. For instance, a random
    * configuration is constrained to be in: [{@code joint.getJointLimitLower()},
    * {@code joint.getJointLimitUpper()}].
    * </p>
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized. As no limits are imposed on the
    *                         joint accelerations, the state to randomize cannot be the acceleration.
    *                         For generating random acceleration, please see
    *                         {@link #nextState(Random, JointStateType, double, double, OneDoFJointBasics)}.
    * @param joints           the joints to set the state of. Modified.
    */
   public static void nextStateWithinJointLimits(Random random, SimJointStateType stateToRandomize, SimOneDoFJointBasics[] joints)
   {
      for (SimOneDoFJointBasics joint : joints)
         nextStateWithinJointLimits(random, stateToRandomize, joint);
   }

   /**
    * Generates a random state and update the given {@code joints} with it.
    * <p>
    * The random state is guaranteed to be within the joint limits. For instance, a random
    * configuration is constrained to be in: [{@code joint.getJointLimitLower()},
    * {@code joint.getJointLimitUpper()}].
    * </p>
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized. As no limits are imposed on the
    *                         joint accelerations, the state to randomize cannot be the acceleration.
    *                         For generating random acceleration, please see
    *                         {@link #nextState(Random, JointStateType, double, double, OneDoFJointBasics)}.
    * @param joints           the joints to set the state of. Modified.
    */
   public static void nextStateWithinJointLimits(Random random, SimJointStateType stateToRandomize, Iterable<? extends SimOneDoFJointBasics> joints)
   {
      joints.forEach(joint -> nextStateWithinJointLimits(random, stateToRandomize, joint));
   }

   /**
    * Generates a random state and update the given {@code joint} with it.
    * <p>
    * The random state is guaranteed to be within the joint limits. For instance, a random
    * configuration is constrained to be in: [{@code joint.getJointLimitLower()},
    * {@code joint.getJointLimitUpper()}].
    * </p>
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized. As no limits are imposed on the
    *                         joint accelerations, the state to randomize cannot be the acceleration.
    *                         For generating random acceleration, please see
    *                         {@link #nextState(Random, JointStateType, double, double, OneDoFJointBasics)}.
    * @param joint            the joints to set the state of. Modified.
    */
   public static void nextStateWithinJointLimits(Random random, SimJointStateType stateToRandomize, SimOneDoFJointBasics joint)
   {
      switch (stateToRandomize)
      {
         case CONFIGURATION:
            joint.setQ(EuclidCoreRandomTools.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper()));
            break;
         case VELOCITY:
            joint.setQd(EuclidCoreRandomTools.nextDouble(random, joint.getVelocityLimitLower(), joint.getVelocityLimitUpper()));
            break;
         case VELOCITY_CHANGE:
            double deltaQdMax = joint.getVelocityLimitUpper() - joint.getQd();
            double deltaQdMin = joint.getVelocityLimitLower() - joint.getQd();
            joint.setDeltaQd(EuclidCoreRandomTools.nextDouble(random, deltaQdMin, deltaQdMax));
            break;
         case EFFORT:
            joint.setTau(EuclidCoreRandomTools.nextDouble(random, joint.getEffortLimitLower(), joint.getEffortLimitUpper()));
            break;
         default:
            throw new RuntimeException("Unhandled state selection: " + stateToRandomize);
      }
   }

   /**
    * Generates a random state and update the given {@code joints} with it.
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized.
    * @param min              the minimum value for the generated random values.
    * @param max              the maximum value for the generated random values.
    * @param joints           the joints to set the state of. Modified.
    */
   public static void nextState(Random random, SimJointStateType stateToRandomize, double min, double max, SimOneDoFJointBasics[] joints)
   {
      for (SimOneDoFJointBasics joint : joints)
         nextState(random, stateToRandomize, min, max, joint);
   }

   /**
    * Generates a random state and update the given {@code joints} with it.
    *
    * @param random           the random generator to use.
    * @param stateToRandomize the joint state that is to be randomized.
    * @param min              the minimum value for the generated random values.
    * @param max              the maximum value for the generated random values.
    * @param joints           the joints to set the state of. Modified.
    */
   public static void nextState(Random random, SimJointStateType stateToRandomize, double min, double max, Iterable<? extends SimOneDoFJointBasics> joints)
   {
      joints.forEach(joint -> nextState(random, stateToRandomize, min, max, joint));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointChain(Random random, int numberOfJoints)
   {
      return nextPrismaticJointChain(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointChain(Random random, String prefix, int numberOfJoints)
   {
      return nextPrismaticJointChain(random, prefix, MecanoRandomTools.nextVector3DArray(random, numberOfJoints, 1.0));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random    the random generator to use.
    * @param prefix    provides a common prefix used for all the joint and rigid-body names.
    * @param jointAxes array containing in order the axis for each joint. The length of the array also
    *                  defines the number of joints for the generated kinematic chain.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointChain(Random random, String prefix, Vector3DReadOnly[] jointAxes)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody(prefix + "RootBody", worldFrame, null);
      return nextPrismaticJointChain(random, prefix, rootBody, jointAxes);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointChain(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextPrismaticJointChain(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextPrismaticJointChain(random, prefix, rootBody, MecanoRandomTools.nextVector3DArray(random, numberOfJoints, 1.0));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random    the random generator to use.
    * @param prefix    provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody  the root to which the kinematic chain is to be attached.
    * @param jointAxes array containing in order the axis for each joint. The length of the array also
    *                  defines the number of joints for the generated kinematic chain.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, Vector3DReadOnly[] jointAxes)
   {
      SimRigidBodyBasics predecessor = rootBody;

      List<SimPrismaticJoint> prismaticJoints = new ArrayList<>();

      for (int i = 0; i < jointAxes.length; i++)
      {
         SimPrismaticJoint joint = nextPrismaticJoint(random, prefix + "SimJointBasics" + i, jointAxes[i], predecessor);
         prismaticJoints.add(joint);
         predecessor = nextRigidBody(random, prefix + "Body" + i, joint);
      }
      return prismaticJoints;
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointChain(Random random, int numberOfJoints)
   {
      return nextRevoluteJointChain(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointChain(Random random, String prefix, int numberOfJoints)
   {
      return nextRevoluteJointChain(random, prefix, MecanoRandomTools.nextVector3DArray(random, numberOfJoints, 1.0));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random    the random generator to use.
    * @param prefix    provides a common prefix used for all the joint and rigid-body names.
    * @param jointAxes array containing in order the axis for each joint. The length of the array also
    *                  defines the number of joints for the generated kinematic chain.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointChain(Random random, String prefix, Vector3DReadOnly[] jointAxes)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody(prefix + "RootBody", worldFrame, null);
      return nextRevoluteJointChain(random, prefix, rootBody, jointAxes);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointChain(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextRevoluteJointChain(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextRevoluteJointChain(random, prefix, rootBody, MecanoRandomTools.nextVector3DArray(random, numberOfJoints, 1.0));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random    the random generator to use.
    * @param prefix    provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody  the root to which the kinematic chain is to be attached.
    * @param jointAxes array containing in order the axis for each joint. The length of the array also
    *                  defines the number of joints for the generated kinematic chain.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, Vector3DReadOnly[] jointAxes)
   {
      SimRigidBodyBasics predecessor = rootBody;

      List<SimRevoluteJoint> revoluteJoints = new ArrayList<>();

      for (int i = 0; i < jointAxes.length; i++)
      {
         SimRevoluteJoint joint = nextRevoluteJoint(random, prefix + "SimJointBasics" + i, jointAxes[i], predecessor);
         revoluteJoints.add(joint);
         predecessor = nextRigidBody(random, prefix + "Body" + i, joint);
      }
      return revoluteJoints;
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointChain(Random random, int numberOfJoints)
   {
      return nextOneDoFJointChain(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointChain(Random random, String prefix, int numberOfJoints)
   {
      return nextOneDoFJointChain(random, prefix, MecanoRandomTools.nextVector3DArray(random, numberOfJoints, 1.0));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random    the random generator to use.
    * @param prefix    provides a common prefix used for all the joint and rigid-body names.
    * @param jointAxes array containing in order the axis for each joint. The length of the array also
    *                  defines the number of joints for the generated kinematic chain.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointChain(Random random, String prefix, Vector3DReadOnly[] jointAxes)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody(prefix + "RootBody", worldFrame, null);
      return nextOneDoFJointChain(random, prefix, rootBody, jointAxes);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointChain(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextOneDoFJointChain(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextOneDoFJointChain(random, prefix, rootBody, MecanoRandomTools.nextVector3DArray(random, numberOfJoints, 1.0));
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random    the random generator to use.
    * @param prefix    provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody  the root to which the kinematic chain is to be attached.
    * @param jointAxes array containing in order the axis for each joint. The length of the array also
    *                  defines the number of joints for the generated kinematic chain.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, Vector3DReadOnly[] jointAxes)
   {
      SimRigidBodyBasics predecessor = rootBody;

      List<SimOneDoFJointBasics> oneDoFJoints = new ArrayList<>();

      for (int i = 0; i < jointAxes.length; i++)
      {
         SimOneDoFJointBasics joint = nextOneDoFJoint(random, prefix + "SimJointBasics" + i, jointAxes[i], predecessor);
         oneDoFJoints.add(joint);
         predecessor = nextRigidBody(random, prefix + "Body" + i, joint);
      }
      return oneDoFJoints;
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimJointBasics> nextJointChain(Random random, int numberOfJoints)
   {
      return nextJointChain(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimJointBasics> nextJointChain(Random random, String prefix, int numberOfJoints)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody(prefix + "RootBody", worldFrame, null);
      return nextJointChain(random, prefix, rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimJointBasics> nextJointChain(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextJointChain(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic chain composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic chain, i.e. every rigid-body has only one child
    * joint.
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic chain is to be attached.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the joints composing the kinematic chain.
    */
   public static List<SimJointBasics> nextJointChain(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      SimRigidBodyBasics predecessor = rootBody;

      List<SimJointBasics> joints = new ArrayList<>();

      for (int i = 0; i < numberOfJoints; i++)
      {
         SimJointBasics joint = nextJoint(random, prefix + "SimJointBasics" + i, predecessor);
         joints.add(joint);
         predecessor = nextRigidBody(random, prefix + "Body" + i, joint);
      }
      return joints;
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointTree(Random random, int numberOfJoints)
   {
      return nextPrismaticJointTree(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointTree(Random random, String prefix, int numberOfJoints)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody(prefix + "RootBody", worldFrame, null);
      return nextPrismaticJointTree(random, prefix, rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointTree(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextPrismaticJointTree(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and prismatic joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimPrismaticJoint> nextPrismaticJointTree(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      List<SimPrismaticJoint> prismaticJoints = new ArrayList<>();

      SimRigidBodyBasics predecessor = rootBody;

      for (int i = 0; i < numberOfJoints; i++)
      {
         SimPrismaticJoint joint = nextPrismaticJoint(random, prefix + "SimJointBasics" + i, predecessor);
         nextRigidBody(random, prefix + "Body" + i, joint);
         prismaticJoints.add(joint);
         predecessor = prismaticJoints.get(random.nextInt(prismaticJoints.size())).getSuccessor();
      }

      return SubtreeStreams.from(SimPrismaticJoint.class, rootBody.getChildrenJoints()).collect(Collectors.toList());
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointTree(Random random, int numberOfJoints)
   {
      return nextRevoluteJointTree(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointTree(Random random, String prefix, int numberOfJoints)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody("RootBody", worldFrame, null);
      return nextRevoluteJointTree(random, prefix, rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointTree(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextRevoluteJointTree(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and revolute joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimRevoluteJoint> nextRevoluteJointTree(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      List<SimRevoluteJoint> revoluteJoints = new ArrayList<>();

      SimRigidBodyBasics predecessor = rootBody;

      for (int i = 0; i < numberOfJoints; i++)
      {
         SimRevoluteJoint joint = nextRevoluteJoint(random, prefix + "SimJointBasics" + i, predecessor);
         nextRigidBody(random, prefix + "Body" + i, joint);
         revoluteJoints.add(joint);
         predecessor = revoluteJoints.get(random.nextInt(revoluteJoints.size())).getSuccessor();
      }

      return SubtreeStreams.from(SimRevoluteJoint.class, rootBody.getChildrenJoints()).collect(Collectors.toList());
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointTree(Random random, int numberOfJoints)
   {
      return nextOneDoFJointTree(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointTree(Random random, String prefix, int numberOfJoints)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody("RootBody", worldFrame, null);
      return nextOneDoFJointTree(random, prefix, rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointTree(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextOneDoFJointTree(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimOneDoFJointBasics> nextOneDoFJointTree(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      List<SimOneDoFJointBasics> oneDoFJoints = new ArrayList<>();

      SimRigidBodyBasics predecessor = rootBody;

      for (int i = 0; i < numberOfJoints; i++)
      {
         SimOneDoFJointBasics joint = nextOneDoFJoint(random, prefix + "SimJointBasics" + i, predecessor);
         nextRigidBody(random, prefix + "Body" + i, joint);
         oneDoFJoints.add(joint);
         predecessor = oneDoFJoints.get(random.nextInt(oneDoFJoints.size())).getSuccessor();
      }

      return SubtreeStreams.from(SimOneDoFJointBasics.class, rootBody.getChildrenJoints()).collect(Collectors.toList());
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimJointBasics> nextJointTree(Random random, int numberOfJoints)
   {
      return nextJointTree(random, "", numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and 1-DoF joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimJointBasics> nextJointTree(Random random, String prefix, int numberOfJoints)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SimRigidBody rootBody = new SimRigidBody("RootBody", worldFrame, null);
      return nextJointTree(random, prefix, rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimJointBasics> nextJointTree(Random random, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      return nextJointTree(random, "", rootBody, numberOfJoints);
   }

   /**
    * Generates a random kinematic tree composed of rigid-bodies and joints.
    * <p>
    * The type of each joint is chosen at random.
    * </p>
    * <p>
    * The joints and rigid-bodies have random physical parameters.
    * </p>
    * <p>
    * The generated multi-body system is a kinematic tree, i.e. every rigid-body can have one or more
    * child joint(s).
    * </p>
    *
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param rootBody       the root to which the kinematic tree is to be attached.
    * @param numberOfJoints how many joints the kinematic tree should be composed of.
    * @return the list of all the joints composing the kinematic tree.
    */
   public static List<SimJointBasics> nextJointTree(Random random, String prefix, SimRigidBodyBasics rootBody, int numberOfJoints)
   {
      List<SimJointBasics> joints = new ArrayList<>();

      SimRigidBodyBasics predecessor = rootBody;

      for (int i = 0; i < numberOfJoints; i++)
      {
         SimJointBasics joint = nextJoint(random, prefix + "SimJointBasics" + i, predecessor);
         nextRigidBody(random, prefix + "Body" + i, joint);
         joints.add(joint);
         predecessor = joints.get(random.nextInt(joints.size())).getSuccessor();
      }

      return SubtreeStreams.from(SimJointBasics.class, rootBody.getChildrenJoints()).collect(Collectors.toList());
   }

   /**
    * Generates a random kinematic chain and attached it to another multi-body system to form a
    * kinematic loop.
    * 
    * @param random         the random generator to use.
    * @param prefix         provides a common prefix used for all the joint and rigid-body names.
    * @param start          the predecessor of the kinematic loop.
    * @param end            the successor of the kinematic loop.
    * @param numberOfJoints how many joints the kinematic chain should be composed of.
    * @return the list of all the newly created joints.
    * @throws IllegalArgumentException if {@code start} is not the ancestor of {@code end}.
    */
   public static List<SimRevoluteJoint> nextKinematicLoopRevoluteJoints(Random random, String prefix, SimRigidBodyBasics start, SimRigidBodyBasics end,
                                                                     int numberOfJoints)
   {
      if (!MultiBodySystemTools.isAncestor(end, start))
         throw new IllegalArgumentException("Improper rigid-bodies configuration: the end must be a descendant of start. Given bodies: [start: "
               + start.getName() + ", end: " + end.getName() + "].");

      List<SimRevoluteJoint> loopChain = nextRevoluteJointChain(random, prefix, start, numberOfJoints);
      SimRevoluteJoint loopClosureJoint = loopChain.get(numberOfJoints - 1);
      start.updateFramesRecursively();
      RigidBodyTransform transformFromSuccessorParentJoint = end.getParentJoint().getFrameAfterJoint()
                                                                .getTransformToDesiredFrame(loopClosureJoint.getFrameAfterJoint());

      loopClosureJoint.setupLoopClosure(end, transformFromSuccessorParentJoint);
      return loopChain;
   }

   /**
    * Generates a prismatic joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimPrismaticJoint nextPrismaticJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      Vector3D jointAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      return nextPrismaticJoint(random, name, jointAxis, predecessor);
   }

   /**
    * Generates a prismatic joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param jointAxis   used to define the joint axis.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimPrismaticJoint nextPrismaticJoint(Random random, String name, Vector3DReadOnly jointAxis, SimRigidBodyBasics predecessor)
   {
      RigidBodyTransform transformToParent = predecessor.isRootBody() ? null : EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return new SimPrismaticJoint(name, predecessor, transformToParent, jointAxis);
   }

   /**
    * Generates a revolute joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimRevoluteJoint nextRevoluteJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      Vector3D jointAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      return nextRevoluteJoint(random, name, jointAxis, predecessor);
   }

   /**
    * Generates a revolute joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param jointAxis   used to define the joint axis.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimRevoluteJoint nextRevoluteJoint(Random random, String name, Vector3DReadOnly jointAxis, SimRigidBodyBasics predecessor)
   {
      RigidBodyTransform transformToParent = predecessor.isRootBody() ? null : EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return new SimRevoluteJoint(name, predecessor, transformToParent, jointAxis);
   }

   /**
    * Generates a 1-DoF joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimOneDoFJointBasics nextOneDoFJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      if (random.nextBoolean())
         return nextPrismaticJoint(random, name, predecessor);
      else
         return nextRevoluteJoint(random, name, predecessor);
   }

   /**
    * Generates a 1-DoF joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param jointAxis   used to define the joint axis.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimOneDoFJointBasics nextOneDoFJoint(Random random, String name, Vector3DReadOnly jointAxis, SimRigidBodyBasics predecessor)
   {
      if (random.nextBoolean())
         return nextPrismaticJoint(random, name, jointAxis, predecessor);
      else
         return nextRevoluteJoint(random, name, jointAxis, predecessor);
   }

   /**
    * Generates a 6-DoF floating joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimSixDoFJoint nextSixDoFJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      RigidBodyTransform transformToParent = predecessor.isRootBody() ? null : EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return new SimSixDoFJoint(name, predecessor, transformToParent);
   }

   /**
    * Generates a 3-DoF floating joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimPlanarJoint nextPlanarJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      RigidBodyTransform transformToParent = predecessor.isRootBody() ? null : EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return new SimPlanarJoint(name, predecessor, transformToParent);
   }

   /**
    * Generates a 3-DoF spherical joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimSphericalJoint nextSphericalJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      RigidBodyTransform transformToParent = predecessor.isRootBody() ? null : EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return new SimSphericalJoint(name, predecessor, transformToParent);
   }

   /**
    * Generates a 0-DoF fixed joint with random physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimFixedJoint nextFixedJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      RigidBodyTransform transformToParent = predecessor.isRootBody() ? null : EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return new SimFixedJoint(name, predecessor, transformToParent);
   }

   /**
    * Generates a joint with random type and physical parameters and attaches it to the given
    * {@code predecessor}.
    *
    * @param random      the random generator to use.
    * @param name        the joint name.
    * @param predecessor the rigid-body to which the joint is added as a child.
    * @return the random joint.
    */
   public static SimJointBasics nextJoint(Random random, String name, SimRigidBodyBasics predecessor)
   {
      switch (random.nextInt(6))
      {
         case 0:
            return nextSixDoFJoint(random, name, predecessor);
         case 1:
            return nextPlanarJoint(random, name, predecessor);
         case 2:
            return nextSphericalJoint(random, name, predecessor);
         case 3:
            return nextPrismaticJoint(random, name, predecessor);
         case 4:
            return nextRevoluteJoint(random, name, predecessor);
         default:
            return nextFixedJoint(random, name, predecessor);
      }
   }

   /**
    * Generates a rigid-body with random physical parameters and attaches it to the given
    * {@code parentJoint}.
    *
    * @param random      the random generator to use.
    * @param name        the rigid-body name.
    * @param parentJoint the joint to which the rigid-body is added as its successor.
    * @return the random rigid-body.
    */
   public static SimRigidBody nextRigidBody(Random random, String name, SimJointBasics parentJoint)
   {
      Matrix3D momentOfInertia = MecanoRandomTools.nextSymmetricPositiveDefiniteMatrix3D(random, 1.0e-4, 2.0, 0.5);
      double mass = 0.1 + random.nextDouble();
      Vector3D comOffset = EuclidCoreRandomTools.nextVector3D(random);
      return new SimRigidBody(name, parentJoint, momentOfInertia, mass, comOffset);
   }

   /**
    * Random multi-body system which root joint is a floating joint followed by a chain of revolute
    * joints.
    * <p>
    * Mostly use for convenience when writing JUnit tests.
    * </p>
    */
   public static class RandomFloatingRevoluteJointChain
   {
      private final SimRigidBody elevator;
      private final SimSixDoFJoint rootJoint;
      private final List<SimRevoluteJoint> revoluteJoints;
      private final List<SimJointBasics> joints = new ArrayList<>();

      /**
       * Creates a new random multi-body system.
       *
       * @param random                 the random generator to use.
       * @param numberOfRevoluteJoints the number of revolute joints to add to this kinematic chain.
       */
      public RandomFloatingRevoluteJointChain(Random random, int numberOfRevoluteJoints)
      {
         this(random, MecanoRandomTools.nextVector3DArray(random, numberOfRevoluteJoints, 1.0));
      }

      /**
       * Creates a new random multi-body system.
       *
       * @param random    the random generator to use.
       * @param jointAxes array containing in order the axis for each revolute joint. The length of the
       *                  array also defines the number of revolute joints for the generated kinematic
       *                  chain.
       */
      public RandomFloatingRevoluteJointChain(Random random, Vector3D[] jointAxes)
      {
         elevator = new SimRigidBody("elevator", ReferenceFrame.getWorldFrame(), null);

         rootJoint = new SimSixDoFJoint("rootJoint", elevator, null);
         SimRigidBody rootBody = nextRigidBody(random, "rootBody", rootJoint);
         revoluteJoints = nextRevoluteJointChain(random, "test", rootBody, jointAxes);

         joints.add(rootJoint);
         joints.addAll(revoluteJoints);
      }

      /**
       * Randomizes the state of this multi-body system and updates its reference frames.
       *
       * @param random          the random generator to use.
       * @param stateSelections the states to randomize.
       */
      public void nextState(Random random, SimJointStateType... stateSelections)
      {
         for (SimJointStateType selection : stateSelections)
            SimMultiBodySystemRandomTools.nextState(random, selection, getJoints());
         getElevator().updateFramesRecursively();
      }

      /**
       * Gets the root body of this floating kinematic chain: the {@code elevator}.
       *
       * @return the elevator for this multi-body system.
       */
      public SimRigidBody getElevator()
      {
         return elevator;
      }

      /**
       * Gets the only floating joint of this floating kinematic chain, i.e. the root joint.
       *
       * @return the root joint for this multi-body system.
       */
      public SimSixDoFJoint getRootJoint()
      {
         return rootJoint;
      }

      /**
       * Gets all the revolute joints composing this floating kinematic chain.
       *
       * @return this multi-body system's revolute joints.
       */
      public List<SimRevoluteJoint> getRevoluteJoints()
      {
         return revoluteJoints;
      }

      /**
       * Gets all the joints, i.e. the floating joint and the revolute joints, composing this floating
       * kinematic chain.
       *
       * @return this multi-body's joints.
       */
      public List<SimJointBasics> getJoints()
      {
         return joints;
      }

      /**
       * Gets the only leaf body, i.e. the rigid-body without any children joints, it is also called the
       * end-effector.
       *
       * @return the leaf body.
       */
      public SimRigidBodyBasics getLeafBody()
      {
         int nRevoluteJoints = revoluteJoints.size();
         if (nRevoluteJoints > 0)
            return revoluteJoints.get(nRevoluteJoints - 1).getSuccessor();
         else
            return rootJoint.getSuccessor();
      }
   }
}
