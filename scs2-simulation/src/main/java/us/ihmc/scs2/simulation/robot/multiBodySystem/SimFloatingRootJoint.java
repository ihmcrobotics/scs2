package us.ihmc.scs2.simulation.robot.multiBodySystem;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
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

import java.util.function.BiConsumer;

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

      buildOrientationListeners(jointQuaternion, jointYawPitchRoll);

      jointLinearVelocity = new YoFrameVector3D("qd" + varName + "world_", beforeJointFrame, registry);
      YoFixedFrameTwist jointTwist = getJointTwist();

      buildVectorListeners(jointQuaternion, jointTwist.getLinearPart(), jointLinearVelocity, jointQuaternion::transform, jointQuaternion::inverseTransform);

      jointLinearAcceleration = new YoFrameVector3D("qdd" + varName + "world_", beforeJointFrame, registry);
      YoFixedFrameSpatialAcceleration jointSpatialAcceleration = getJointAcceleration();

      buildVectorListeners(jointQuaternion,
                           jointSpatialAcceleration.getLinearPart(),
                           jointLinearAcceleration,
                           (a, b) -> transformLinearAcceleration(jointQuaternion, jointTwist, a, b),
                           (b, a) -> inverseTransformLinearAcceleration(jointQuaternion, jointTwist, b, a));
   }

   private static void inverseTransformLinearAcceleration(QuaternionReadOnly quaternion,
                                                          TwistReadOnly twist,
                                                          Vector3DReadOnly linearAccelerationOriginal,
                                                          Vector3DBasics linearAccelerationTransformed)
   {
      quaternion.inverseTransform(linearAccelerationOriginal, linearAccelerationTransformed);
      MecanoTools.addCrossToVector(twist.getLinearPart(), twist.getAngularPart(), linearAccelerationTransformed);
   }

   private static void transformLinearAcceleration(QuaternionReadOnly quaternion,
                                                   TwistReadOnly twist,
                                                   Vector3DReadOnly linearAccelerationOriginal,
                                                   Vector3DBasics linearAccelerationTransformed)
   {
      linearAccelerationTransformed.set(linearAccelerationOriginal);
      MecanoTools.addCrossToVector(twist.getAngularPart(), twist.getLinearPart(), linearAccelerationTransformed);
      quaternion.transform(linearAccelerationTransformed);
   }

   private static void buildOrientationListeners(YoFrameQuaternion quaternion, YoFrameYawPitchRoll yawPitchRoll)
   {
      MutableBoolean updatingYPR = new MutableBoolean(false);
      MutableBoolean updatingQuat = new MutableBoolean(false);

      quaternion.getYoQs().addListener(v ->
                                       {
                                          if (updatingQuat.booleanValue())
                                             return;

                                          updatingYPR.setTrue();
                                          try
                                          {
                                             yawPitchRoll.set(quaternion);
                                          }
                                          finally
                                          {
                                             updatingYPR.setFalse();
                                          }
                                       });

      double updateThreshold = 1.0e-6;

      // Doing 1 component at a time to avoid updating the quaternion with old data.
      yawPitchRoll.getYoYaw().addListener(v ->
                                          {
                                             if (updatingYPR.booleanValue())
                                                return;
                                             updatingQuat.setTrue();
                                             try
                                             {
                                                if (!EuclidCoreTools.epsilonEquals(quaternion.getYaw(), yawPitchRoll.getYaw(), updateThreshold))
                                                   quaternion.setYawPitchRoll(yawPitchRoll.getYaw(), quaternion.getPitch(), quaternion.getRoll());
                                             }
                                             finally
                                             {
                                                updatingQuat.setFalse();
                                             }
                                          });

      yawPitchRoll.getYoPitch().addListener(v ->
                                            {
                                               if (updatingYPR.booleanValue())
                                                  return;
                                               updatingQuat.setTrue();
                                               try
                                               {
                                                  if (!EuclidCoreTools.epsilonEquals(quaternion.getPitch(), yawPitchRoll.getPitch(), updateThreshold))
                                                     quaternion.setYawPitchRoll(quaternion.getYaw(), yawPitchRoll.getPitch(), quaternion.getRoll());
                                               }
                                               finally
                                               {
                                                  updatingQuat.setFalse();
                                               }
                                            });

      yawPitchRoll.getYoRoll().addListener(v ->
                                           {
                                              if (updatingYPR.booleanValue())
                                                 return;
                                              updatingQuat.setTrue();
                                              try
                                              {
                                                 if (!EuclidCoreTools.epsilonEquals(quaternion.getRoll(), yawPitchRoll.getRoll(), updateThreshold))
                                                    quaternion.setYawPitchRoll(quaternion.getYaw(), quaternion.getPitch(), yawPitchRoll.getRoll());
                                              }
                                              finally
                                              {
                                                 updatingQuat.setFalse();
                                              }
                                           });
   }

   private static void buildVectorListeners(YoFrameQuaternion quaternion,
                                            YoFrameVector3D vectorA,
                                            YoFrameVector3D vectorB,
                                            BiConsumer<Vector3DReadOnly, Vector3DBasics> transformAToB,
                                            BiConsumer<Vector3DReadOnly, Vector3DBasics> transformBToA)
   {
      MutableBoolean updatingA = new MutableBoolean(false);
      MutableBoolean updatingB = new MutableBoolean(false);

      vectorA.attachVariableChangedListener(v ->
                                            {
                                               if (updatingA.booleanValue())
                                                  return;
                                               updatingB.setTrue();
                                               try
                                               {
                                                  transformAToB.accept(vectorA, vectorB);
                                               }
                                               finally
                                               {
                                                  updatingB.setFalse();
                                               }
                                            });
      quaternion.getYoQs().addListener(v -> transformAToB.accept(vectorA, vectorB)); // Unidirectional update no need to check for updatingB.

      double updateThreshold = 1.0e-6;
      Vector3D intermediateVector = new Vector3D();

      // Doing 1 component at a time to avoid updating the vectorA with old data.
      vectorB.attachVariableChangedListener(v ->
                                            {
                                               if (updatingB.booleanValue())
                                                  return;
                                               updatingA.setTrue();
                                               try
                                               {
                                                  transformAToB.accept(vectorA, intermediateVector);
                                                  Axis3D axis = v == vectorB.getYoX() ? Axis3D.X : v == vectorB.getYoY() ? Axis3D.Y : Axis3D.Z;

                                                  if (!EuclidCoreTools.epsilonEquals(intermediateVector.getElement(axis),
                                                                                     vectorB.getElement(axis),
                                                                                     updateThreshold))
                                                  {
                                                     intermediateVector.setElement(axis, vectorB.getElement(axis));
                                                     transformBToA.accept(intermediateVector, vectorA);
                                                  }
                                               }
                                               finally
                                               {
                                                  updatingA.setFalse();
                                               }
                                            });
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}
