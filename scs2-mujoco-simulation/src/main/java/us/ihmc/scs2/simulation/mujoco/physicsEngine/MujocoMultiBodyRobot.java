package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.util.LinkedHashMap;
import java.util.Map;

import org.bytedeco.javacpp.BytePointer;

import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;

/**
 * Native-side view of a single SCS2 robot inside the shared `mjModel`. Stores the joint name to
 * MuJoCo (jointId, qposadr, qveladr) triples once at construction so push/pull are O(N) array
 * indexing with no string lookups per step.
 *
 * <p>The floating root joint, if any, is stored separately: in MuJoCo it occupies 7 qpos entries
 * (xyz + quat wxyz) and 6 qvel entries. All other joints in v1 are 1-DoF (revolute or prismatic).
 */
public class MujocoMultiBodyRobot
{
   private final String robotName;
   private final mjModel model;
   private final Map<String, JointAddress> jointAddressByName = new LinkedHashMap<>();
   private final Map<String, Integer> bodyIdByName = new LinkedHashMap<>();
   private JointAddress rootJointAddress;  // null if robot has no floating joint

   public MujocoMultiBodyRobot(String robotName, mjModel model)
   {
      this.robotName = robotName;
      this.model = model;
   }

   /**
    * Resolve the MuJoCo body id for {@code scs2BodyName}. Required for routing external wrenches
    * to {@code mjData.xfrc_applied[bodyId * 6 + i]}.
    */
   public void registerBody(String scs2BodyName)
   {
      int bodyId;
      try (BytePointer name = new BytePointer(scs2BodyName))
      {
         bodyId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, name);
      }
      if (bodyId < 0)
      {
         System.err.println("[MujocoMultiBodyRobot] body not found in MJCF: '" + scs2BodyName + "'");
         return;
      }
      bodyIdByName.put(scs2BodyName, bodyId);
   }

   public int getBodyId(String scs2BodyName)
   {
      Integer id = bodyIdByName.get(scs2BodyName);
      return id == null ? -1 : id;
   }

   /**
    * Resolve the MuJoCo joint id for the given SCS2 joint name and cache its qpos/qvel addresses.
    * Throws if the joint isn't found in the compiled `mjModel`.
    */
   public void registerJoint(String scs2JointName, boolean isFloatingRoot)
   {
      int jointId;
      try (BytePointer name = new BytePointer(scs2JointName))
      {
         jointId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_JOINT, name);
      }
      if (jointId < 0)
         throw new RuntimeException("MuJoCo joint not found: '" + scs2JointName + "' in robot '" + robotName + "'");

      int qposadr = model.jnt_qposadr().get(jointId);
      int qveladr = model.jnt_dofadr().get(jointId);
      JointAddress address = new JointAddress(scs2JointName, jointId, qposadr, qveladr, isFloatingRoot);
      jointAddressByName.put(scs2JointName, address);
      if (isFloatingRoot)
      {
         if (rootJointAddress != null)
            throw new IllegalStateException("Robot '" + robotName + "' has more than one floating root joint.");
         rootJointAddress = address;
      }
   }

   public JointAddress getJointAddress(String scs2JointName)
   {
      return jointAddressByName.get(scs2JointName);
   }

   public JointAddress getRootJointAddress()
   {
      return rootJointAddress;
   }

   public Map<String, JointAddress> getJointAddresses()
   {
      return jointAddressByName;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public mjModel getModel()
   {
      return model;
   }

   public static final class JointAddress
   {
      public final String name;
      public final int jointId;
      public final int qposadr;
      public final int qveladr;
      public final boolean isFloatingRoot;

      JointAddress(String name, int jointId, int qposadr, int qveladr, boolean isFloatingRoot)
      {
         this.name = name;
         this.jointId = jointId;
         this.qposadr = qposadr;
         this.qveladr = qveladr;
         this.isFloatingRoot = isFloatingRoot;
      }
   }
}
