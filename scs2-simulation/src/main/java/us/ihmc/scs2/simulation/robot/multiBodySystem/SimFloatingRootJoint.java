package us.ihmc.scs2.simulation.robot.multiBodySystem;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialAcceleration;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimFloatingRootJoint extends SimSixDoFJoint implements SimJointBasics, SimFloatingJointBasics
{
   private final YoRegistry registry;
   private final YoFrameYawPitchRoll jointYawPitchRoll;
   private final YoFrameVector3D jointLinearVelocity;
   private final YoFrameVector3D jointLinearAcceleration;

   public SimFloatingRootJoint(SixDoFJointDefinition definition, SimRigidBodyBasics predecessor)
   {
      this(definition.getName(), predecessor, definition.getTransformToParent());
   }

   public SimFloatingRootJoint(String name, SimRigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   public SimFloatingRootJoint(String name, SimRigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, predecessor, transformToParent);
      registry = predecessor.getRegistry();

      String varName = !name.isEmpty() ? "_" + name + "_" : "_";

      jointYawPitchRoll = new YoFrameYawPitchRoll("q" + varName, beforeJointFrame, registry);
      YoFrameQuaternion jointQuaternion = getJointPose().getOrientation();

      MutableBoolean updatingYPR = new MutableBoolean(false);
      MutableBoolean updatingQuat = new MutableBoolean(false);

      jointQuaternion.getYoQs().addListener(v ->
                                            {
                                               if (updatingQuat.booleanValue())
                                                  return;

                                               updatingYPR.setTrue();
                                               try
                                               {
                                                  jointYawPitchRoll.set(jointQuaternion);
                                               }
                                               finally
                                               {
                                                  updatingYPR.setFalse();
                                               }
                                            });

      jointYawPitchRoll.attachVariableChangedListener(v ->
                                                      {
                                                         if (updatingYPR.booleanValue())
                                                            return;

                                                         updatingQuat.setTrue();
                                                         try
                                                         {
                                                            jointQuaternion.set(jointYawPitchRoll);
                                                         }
                                                         finally
                                                         {
                                                            updatingQuat.setFalse();
                                                         }
                                                      });

      jointLinearVelocity = new YoFrameVector3D("qd" + varName + "world_", beforeJointFrame, registry);
      YoFixedFrameTwist jointTwist = getJointTwist();

      MutableBoolean updatingLinVel = new MutableBoolean(false);
      MutableBoolean updatingTwist = new MutableBoolean(false);

      jointTwist.getLinearPart().attachVariableChangedListener(v ->
                                                               {
                                                                  if (updatingTwist.booleanValue())
                                                                     return;
                                                                  updatingLinVel.setTrue();
                                                                  jointQuaternion.transform((Tuple3DReadOnly) jointTwist.getLinearPart(),
                                                                                            (Tuple3DBasics) jointLinearVelocity);
                                                                  updatingLinVel.setFalse();
                                                               });

      jointLinearVelocity.attachVariableChangedListener(v ->
                                                        {
                                                           if (updatingLinVel.booleanValue())
                                                              return;
                                                           updatingTwist.setTrue();
                                                           jointQuaternion.inverseTransform((Tuple3DReadOnly) jointLinearVelocity,
                                                                                            (Tuple3DBasics) jointTwist.getLinearPart());
                                                           updatingTwist.setFalse();
                                                        });

      jointLinearAcceleration = new YoFrameVector3D("qdd" + varName + "world_", beforeJointFrame, registry);
      YoFixedFrameSpatialAcceleration jointSpatialAcceleration = getJointAcceleration();

      MutableBoolean updatingLinAcc = new MutableBoolean(false);
      MutableBoolean updatingSpAcc = new MutableBoolean(false);

      jointSpatialAcceleration.getLinearPart().attachVariableChangedListener(v ->
                                                                             {
                                                                                if (updatingSpAcc.booleanValue())
                                                                                   return;
                                                                                updatingLinAcc.setTrue();
                                                                                jointLinearAcceleration.set((Vector3DReadOnly) jointSpatialAcceleration.getLinearPart());
                                                                                MecanoTools.addCrossToVector(jointTwist.getAngularPart(),
                                                                                                             jointTwist.getLinearPart(),
                                                                                                             jointLinearAcceleration);
                                                                                jointQuaternion.transform((Tuple3DBasics) jointLinearAcceleration);
                                                                                updatingLinAcc.setFalse();
                                                                             });

      jointLinearAcceleration.attachVariableChangedListener(v ->
                                                            {
                                                               if (updatingLinAcc.booleanValue())
                                                                  return;
                                                               updatingSpAcc.setTrue();
                                                               jointQuaternion.inverseTransform((Tuple3DReadOnly) jointSpatialAcceleration.getLinearPart(),
                                                                                                (Tuple3DBasics) jointLinearAcceleration);
                                                               jointSpatialAcceleration.addCrossToLinearPart(jointTwist.getLinearPart(),
                                                                                                             jointTwist.getAngularPart());
                                                               updatingSpAcc.setFalse();
                                                            });
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}
