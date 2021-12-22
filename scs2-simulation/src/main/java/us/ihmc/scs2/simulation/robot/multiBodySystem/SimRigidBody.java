package us.ihmc.scs2.simulation.robot.multiBodySystem;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollisionTools;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimRigidBody extends RigidBody implements SimRigidBodyBasics
{
   private final YoRegistry registry;
   private final List<Collidable> collidables = new ArrayList<>();

   public SimRigidBody(String bodyName, ReferenceFrame parentStationaryFrame, YoRegistry registry)
   {
      this(bodyName, new RigidBodyTransform(), parentStationaryFrame, registry);
   }

   public SimRigidBody(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame, YoRegistry registry)
   {
      super(bodyName, transformToParent, parentStationaryFrame);
      this.registry = registry;
   }

   public SimRigidBody(String bodyName, SimJointBasics parentJoint, double Ixx, double Iyy, double Izz, double mass, Tuple3DReadOnly centerOfMassOffset)
   {
      this(bodyName, parentJoint, new Matrix3D(Ixx, 0, 0, 0, Iyy, 0, 0, 0, Izz), mass, centerOfMassOffset);
   }

   public SimRigidBody(String bodyName, SimJointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, Tuple3DReadOnly centerOfMassOffset)
   {
      this(bodyName, parentJoint, momentOfInertia, mass, new RigidBodyTransform(new Quaternion(), centerOfMassOffset));
   }

   public SimRigidBody(String bodyName, SimJointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransformReadOnly inertiaPose)
   {
      super(bodyName, parentJoint, momentOfInertia, mass, inertiaPose);
      this.registry = parentJoint.getRegistry();
   }

   public SimRigidBody(RigidBodyDefinition definition, ReferenceFrame parentStationaryFrame, YoRegistry registry)
   {
      super(definition.getName(), definition.getInertiaPose(), parentStationaryFrame);
      this.registry = registry;
      collidables.addAll(CollisionTools.toCollidableRigidBody(definition, this));
   }

   public SimRigidBody(RigidBodyDefinition definition, SimJointBasics parentJoint)
   {
      super(definition.getName(), parentJoint, definition.getMomentOfInertia(), definition.getMass(), definition.getInertiaPose());
      this.registry = parentJoint.getRegistry();
      collidables.addAll(CollisionTools.toCollidableRigidBody(definition, this));
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public SimJointBasics getParentJoint()
   {
      return (SimJointBasics) super.getParentJoint();
   }

   @Override
   public void addChildJoint(JointBasics joint)
   {
      if (joint instanceof SimJointBasics)
         super.addChildJoint(joint);
      else
         throw new IllegalArgumentException("Can only add a " + SimJointBasics.class.getSimpleName() + " as child of a " + getClass().getSimpleName());
   }

   @Override
   public void addParentLoopClosureJoint(JointBasics parentLoopClosureJoint)
   {
      if (parentLoopClosureJoint instanceof SimJointBasics)
         super.addParentLoopClosureJoint(parentLoopClosureJoint);
      else
         throw new IllegalArgumentException("Can only add a " + SimJointBasics.class.getSimpleName() + " as parent of a " + getClass().getSimpleName());
   }

   @Override
   public List<Collidable> getCollidables()
   {
      return collidables;
   }

   @Override
   public Iterable<? extends SimRigidBody> subtreeIterable()
   {
      return new RigidBodyIterable<>(SimRigidBody.class, null, this);
   }

   @Override
   public Iterable<? extends SimJointBasics> childrenSubtreeIterable()
   {
      return new JointIterable<>(SimJointBasics.class, null, getChildrenJoints());
   }

   @Override
   public Stream<? extends SimRigidBody> subtreeStream()
   {
      return SubtreeStreams.from(SimRigidBody.class, this);
   }

   @Override
   public List<? extends SimRigidBody> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   public SimRigidBody[] subtreeArray()
   {
      return subtreeStream().toArray(SimRigidBody[]::new);
   }
}
