package us.ihmc.scs2.definition.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.addPrefixToMessage;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.throwNotEqualAssertionError;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;

import javax.xml.bind.JAXBException;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class ValkyrieModelLoadingTest
{
   private static final double EPSILON = 1.0e-5;

   private static final String LeftHipYawName = "leftHipYaw";
   private static final String LeftHipRollName = "leftHipRoll";
   private static final String LeftHipPitchName = "leftHipPitch";
   private static final String LeftKneePitchName = "leftKneePitch";
   private static final String LeftAnklePitchName = "leftAnklePitch";
   private static final String LeftAnkleRollName = "leftAnkleRoll";

   private static final String RightHipYawName = "rightHipYaw";
   private static final String RightHipRollName = "rightHipRoll";
   private static final String RightHipPitchName = "rightHipPitch";
   private static final String RightKneePitchName = "rightKneePitch";
   private static final String RightAnklePitchName = "rightAnklePitch";
   private static final String RightAnkleRollName = "rightAnkleRoll";

   private static final String TorsoYawName = "torsoYaw";
   private static final String TorsoPitchName = "torsoPitch";
   private static final String TorsoRollName = "torsoRoll";

   private static final String NeckLowerPitchName = "lowerNeckPitch";
   private static final String NeckYawName = "neckYaw";
   private static final String NeckUpperPitchName = "upperNeckPitch";

   private static final String HokuyoJointName = "hokuyo_joint";

   private static final String LeftShoulderPitchName = "leftShoulderPitch";
   private static final String LeftShoulderRollName = "leftShoulderRoll";
   private static final String LeftShoulderYawName = "leftShoulderYaw";
   private static final String LeftElbowPitchName = "leftElbowPitch";
   private static final String LeftForearmYawName = "leftForearmYaw";
   private static final String LeftWristPitchName = "leftWristPitch";
   private static final String LeftWristRollName = "leftWristRoll";

   private static final String RightShoulderPitchName = "rightShoulderPitch";
   private static final String RightShoulderRollName = "rightShoulderRoll";
   private static final String RightShoulderYawName = "rightShoulderYaw";
   private static final String RightElbowPitchName = "rightElbowPitch";
   private static final String RightForearmYawName = "rightForearmYaw";
   private static final String RightWristPitchName = "rightWristPitch";
   private static final String RightWristRollName = "rightWristRoll";

   private static final String PelvisName = "pelvis";

   private static final String LeftHipYawLinkName = "leftHipYawLink";
   private static final String LeftHipRollLinkName = "leftHipRollLink";
   private static final String LeftHipPitchLinkName = "leftHipPitchLink";
   private static final String LeftKneePitchLinkName = "leftKneePitchLink";
   private static final String LeftAnklePitchLinkName = "leftAnklePitchLink";
   private static final String LeftFootName = "leftFoot";

   private static final String RightHipYawLinkName = "rightHipYawLink";
   private static final String RightHipRollLinkName = "rightHipRollLink";
   private static final String RightHipPitchLinkName = "rightHipPitchLink";
   private static final String RightKneePitchLinkName = "rightKneePitchLink";
   private static final String RightAnklePitchLinkName = "rightAnklePitchLink";
   private static final String RightFootName = "rightFoot";

   private static final String TorsoYawLinkName = "torsoYawLink";
   private static final String TorsoPitchLinkName = "torsoPitchLink";
   private static final String TorsoName = "torso";

   private static final String NeckLowerPitchLinkName = "lowerNeckPitchLink";
   private static final String NeckYawLinkName = "neckYawLink";
   private static final String NeckUpperPitchLinkName = "upperNeckPitchLink";

   private static final String HokuyoLinkName = "hokuyo_link";

   private static final String LeftShoulderPitchLinkName = "leftShoulderPitchLink";
   private static final String LeftShoulderRollLinkName = "leftShoulderRollLink";
   private static final String LeftShoulderYawLinkName = "leftShoulderYawLink";
   private static final String LeftElbowPitchLinkName = "leftElbowPitchLink";
   private static final String LeftForearmLinkName = "leftForearmLink";
   private static final String LeftWristRollLinkName = "leftWristRollLink";
   private static final String LeftPalmName = "leftPalm";

   private static final String RightShoulderPitchLinkName = "rightShoulderPitchLink";
   private static final String RightShoulderRollLinkName = "rightShoulderRollLink";
   private static final String RightShoulderYawLinkName = "rightShoulderYawLink";
   private static final String RightElbowPitchLinkName = "rightElbowPitchLink";
   private static final String RightForearmLinkName = "rightForearmLink";
   private static final String RightWristRollLinkName = "rightWristRollLink";
   private static final String RightPalmName = "rightPalm";

   private static final String[] LeftLegJointNames = {LeftHipYawName,
                                                      LeftHipRollName,
                                                      LeftHipPitchName,
                                                      LeftKneePitchName,
                                                      LeftAnklePitchName,
                                                      LeftAnkleRollName};
   private static final String[] RightLegJointNames = {RightHipYawName,
                                                       RightHipRollName,
                                                       RightHipPitchName,
                                                       RightKneePitchName,
                                                       RightAnklePitchName,
                                                       RightAnkleRollName};
   private static final String[] TorsoJointNames = {TorsoYawName, TorsoPitchName, TorsoRollName};
   private static final String[] NeckJointNames = {NeckLowerPitchName, NeckYawName, NeckUpperPitchName};
   private static final String[] LeftArmJointNames = {LeftShoulderPitchName,
                                                      LeftShoulderRollName,
                                                      LeftShoulderYawName,
                                                      LeftElbowPitchName,
                                                      LeftForearmYawName,
                                                      LeftWristRollName,
                                                      LeftWristPitchName};
   private static final String[] LeftIndexFingerJointNames = {"leftIndexFingerPitch1", "leftIndexFingerPitch2", "leftIndexFingerPitch3"};
   private static final String[] LeftMiddleFingerJointNames = {"leftMiddleFingerPitch1", "leftMiddleFingerPitch2", "leftMiddleFingerPitch3"};
   private static final String[] LeftPinkyJointNames = {"leftPinkyPitch1", "leftPinkyPitch2", "leftPinkyPitch3"};
   private static final String[] LeftThumbJointNames = {"leftThumbRoll", "leftThumbPitch1", "leftThumbPitch2", "leftThumbPitch3"};
   private static final String[] RightArmJointNames = {RightShoulderPitchName,
                                                       RightShoulderRollName,
                                                       RightShoulderYawName,
                                                       RightElbowPitchName,
                                                       RightForearmYawName,
                                                       RightWristRollName,
                                                       RightWristPitchName};
   private static final String[] RightIndexFingerJointNames = {"rightIndexFingerPitch1", "rightIndexFingerPitch2", "rightIndexFingerPitch3"};
   private static final String[] RightMiddleFingerJointNames = {"rightMiddleFingerPitch1", "rightMiddleFingerPitch2", "rightMiddleFingerPitch3"};
   private static final String[] RightPinkyJointNames = {"rightPinkyPitch1", "rightPinkyPitch2", "rightPinkyPitch3"};
   private static final String[] RightThumbJointNames = {"rightThumbRoll", "rightThumbPitch1", "rightThumbPitch2", "rightThumbPitch3"};
   private static final String[] AllJointNames = concatenate(new String[] {PelvisName, HokuyoJointName},
                                                             LeftLegJointNames,
                                                             RightLegJointNames,
                                                             TorsoJointNames,
                                                             NeckJointNames,
                                                             LeftArmJointNames,
                                                             LeftIndexFingerJointNames,
                                                             LeftMiddleFingerJointNames,
                                                             LeftPinkyJointNames,
                                                             LeftThumbJointNames,
                                                             RightArmJointNames,
                                                             RightIndexFingerJointNames,
                                                             RightMiddleFingerJointNames,
                                                             RightPinkyJointNames,
                                                             RightThumbJointNames);

   private static final String[] LeftLegLinkNames = {LeftHipYawLinkName,
                                                     LeftHipRollLinkName,
                                                     LeftHipPitchLinkName,
                                                     LeftKneePitchLinkName,
                                                     LeftAnklePitchLinkName,
                                                     LeftFootName};
   private static final String[] RightLegLinkNames = {RightHipYawLinkName,
                                                      RightHipRollLinkName,
                                                      RightHipPitchLinkName,
                                                      RightKneePitchLinkName,
                                                      RightAnklePitchLinkName,
                                                      RightFootName};
   private static final String[] TorsoLinkNames = {TorsoYawLinkName, TorsoPitchLinkName, TorsoName};
   private static final String[] NeckLinkNames = {NeckLowerPitchLinkName, NeckYawLinkName, NeckUpperPitchLinkName};
   private static final String[] LeftArmLinkNames = {LeftShoulderPitchLinkName,
                                                     LeftShoulderRollLinkName,
                                                     LeftShoulderYawLinkName,
                                                     LeftElbowPitchLinkName,
                                                     LeftForearmLinkName,
                                                     LeftWristRollLinkName,
                                                     LeftPalmName};
   private static final String[] LeftIndexFingerLinkNames = Stream.of(LeftIndexFingerJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] LeftMiddleFingerLinkNames = Stream.of(LeftMiddleFingerJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] LeftPinkyLinkNames = Stream.of(LeftPinkyJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] LeftThumbLinkNames = Stream.of(LeftThumbJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] RightArmLinkNames = {RightShoulderPitchLinkName,
                                                      RightShoulderRollLinkName,
                                                      RightShoulderYawLinkName,
                                                      RightElbowPitchLinkName,
                                                      RightForearmLinkName,
                                                      RightWristRollLinkName,
                                                      RightPalmName};
   private static final String[] RightIndexFingerLinkNames = Stream.of(RightIndexFingerJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] RightMiddleFingerLinkNames = Stream.of(RightMiddleFingerJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] RightPinkyLinkNames = Stream.of(RightPinkyJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] RightThumbLinkNames = Stream.of(RightThumbJointNames).map(s -> s + "Link").toArray(String[]::new);
   private static final String[] AllLinkNames = concatenate(new String[] {PelvisName, HokuyoLinkName},
                                                            LeftLegLinkNames,
                                                            RightLegLinkNames,
                                                            TorsoLinkNames,
                                                            NeckLinkNames,
                                                            LeftArmLinkNames,
                                                            LeftIndexFingerLinkNames,
                                                            LeftMiddleFingerLinkNames,
                                                            LeftPinkyLinkNames,
                                                            LeftThumbLinkNames,
                                                            RightArmLinkNames,
                                                            RightIndexFingerLinkNames,
                                                            RightMiddleFingerLinkNames,
                                                            RightPinkyLinkNames,
                                                            RightThumbLinkNames);

   @Test
   public void testSDFTools() throws Exception
   {
      InputStream resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/valkyrie/valkyrie_sim.sdf");
      SDFRoot sdfRoot = SDFTools.loadSDFRoot(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition robotDefinition = SDFTools.toFloatingRobotDefinition(sdfRoot.getModels().get(0));
      robotDefinition.transformsAllFrameToZUp();
      performAssertionsOnRobotDefinition(robotDefinition);
   }

   @Test
   public void testURDFTools() throws Exception
   {
      InputStream resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/valkyrie/valkyrie_sim.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition robotDefinition = URDFTools.toFloatingRobotDefinition(urdfModel);
      robotDefinition.simplifyKinematics();
      robotDefinition.transformsAllFrameToZUp();
      performAssertionsOnRobotDefinition(robotDefinition);
   }

   @Test
   public void testSDF_vs_URDFTools() throws Exception
   {
      InputStream resourceAsStream;
      resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/valkyrie/valkyrie_sim.sdf");
      SDFRoot sdfRoot = SDFTools.loadSDFRoot(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition robotSDF = SDFTools.toFloatingRobotDefinition(sdfRoot.getModels().get(0));

      resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/valkyrie/valkyrie_sim.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition robotURDF = URDFTools.toFloatingRobotDefinition(urdfModel);
      robotURDF.simplifyKinematics();
      robotURDF.transformsAllFrameToZUp();

      for (RigidBodyDefinition bodySDF : robotSDF.getAllRigidBodies())
      {
         RigidBodyDefinition bodyURDF = robotSDF.getRigidBodyDefinition(bodySDF.getName());
         assertNotNull(bodyURDF, "Couldn't find the rigid-body %s in the URDF robot definition.".formatted(bodySDF.getName()));
         assertRigidBodyEquals(bodySDF, bodyURDF, EPSILON);
      }

      for (JointDefinition jointSDF : robotSDF.getAllJoints())
      {
         if (jointSDF.getName().equals(HokuyoJointName))
            continue; // Something wonky about that guy
         JointDefinition jointURDF = robotURDF.getJointDefinition(jointSDF.getName());
         assertNotNull(jointURDF, "Couldn't find the joint %s in the URDF robot definition.".formatted(jointSDF.getName()));
         jointURDF.getSensorDefinitions().removeIf(sensor -> sensor instanceof WrenchSensorDefinition); // SDF doesn't have tags for that sensor
         assertJointEquals(jointSDF, jointURDF, EPSILON);
      }
   }

   public static void assertRigidBodyEquals(RigidBodyDefinition expected, RigidBodyDefinition actual, double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError("Rigid-bodies do not have the same name", expected.getName(), actual.getName());
      if (Double.doubleToLongBits(expected.getMass()) != Double.doubleToLongBits(expected.getMass()))
         throwNotEqualAssertionError("Rigid-bodies (%s) do not have the same mass".formatted(expected.getName()),
                                     Double.toString(expected.getMass()),
                                     Double.toString(actual.getMass()));
      if (!EuclidCoreTools.epsilonEquals(expected.getMomentOfInertia(), actual.getMomentOfInertia(), epsilon))
         throwNotEqualAssertionError("Rigid-bodies (%s) do not have the same MoI".formatted(expected.getName()),
                                     Objects.toString(expected.getMass()),
                                     Objects.toString(actual.getMass()));
      if (!EuclidCoreTools.epsilonEquals(expected.getInertiaPose(), actual.getInertiaPose(), epsilon))
         throwNotEqualAssertionError("Rigid-bodies (%s) do not have the same inertia pose".formatted(expected.getName()),
                                     Objects.toString(expected.getInertiaPose()),
                                     Objects.toString(actual.getInertiaPose()));
      if (!Objects.equals(expected.getChildrenJoints().size(), actual.getChildrenJoints().size()))
         throwNotEqualAssertionError("Rigid-bodies (%s) do not have the same number of children".formatted(expected.getName()),
                                     Integer.toString(expected.getChildrenJoints().size()),
                                     Integer.toString(actual.getChildrenJoints().size()));
      if (!Objects.equals(expected.getVisualDefinitions().size(), actual.getVisualDefinitions().size()))
         throwNotEqualAssertionError("Rigid-bodies (%s) do not have the same number of visuals".formatted(expected.getName()),
                                     Integer.toString(expected.getVisualDefinitions().size()),
                                     Integer.toString(actual.getVisualDefinitions().size()));

      for (int i = 0; i < expected.getVisualDefinitions().size(); i++)
      {
         VisualDefinition expectedVisual = expected.getVisualDefinitions().get(i);
         VisualDefinition actualVisual = actual.getVisualDefinitions().get(i);
         assertVisualEquals("In rigid-body (%s)".formatted(expected.getName()), expectedVisual, actualVisual, epsilon);
      }

      if (!Objects.equals(expected.getCollisionShapeDefinitions().size(), actual.getCollisionShapeDefinitions().size()))
         throwNotEqualAssertionError("Rigid-bodies (%s) do not have the same number of collisions".formatted(expected.getName()),
                                     Integer.toString(expected.getCollisionShapeDefinitions().size()),
                                     Integer.toString(actual.getCollisionShapeDefinitions().size()));

      for (int i = 0; i < expected.getCollisionShapeDefinitions().size(); i++)
      {
         CollisionShapeDefinition expectedCollisionShape = expected.getCollisionShapeDefinitions().get(i);
         CollisionShapeDefinition actualCollisionShape = actual.getCollisionShapeDefinitions().get(i);
         assertCollisionShapeEquals("In rigid-body (%s)".formatted(expected.getName()), expectedCollisionShape, actualCollisionShape, epsilon);
      }
   }

   public static void assertVisualEquals(String messagePrefix, VisualDefinition expected, VisualDefinition actual, double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Visuals do not have the same name"), expected.getName(), actual.getName());
      if (!EuclidCoreTools.epsilonEquals(expected.getOriginPose(), actual.getOriginPose(), epsilon))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Visuals (%s) do not have the same pose".formatted(expected.getName())),
                                     Objects.toString(expected.getOriginPose()),
                                     Objects.toString(actual.getOriginPose()));
      if (!Objects.equals(expected.getGeometryDefinition(), actual.getGeometryDefinition()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Visuals (%s) do not have the same geometry".formatted(expected.getName())),
                                     Objects.toString(expected.getGeometryDefinition()),
                                     Objects.toString(actual.getGeometryDefinition()));
      if (!Objects.equals(expected.getMaterialDefinition(), actual.getMaterialDefinition()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Visuals (%s) do not have the same material".formatted(expected.getName())),
                                     Objects.toString(expected.getMaterialDefinition()),
                                     Objects.toString(actual.getMaterialDefinition()));
   }

   public static void assertCollisionShapeEquals(String messagePrefix, CollisionShapeDefinition expected, CollisionShapeDefinition actual, double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Collisions do not have the same name"), expected.getName(), actual.getName());
      if (!EuclidCoreTools.epsilonEquals(expected.getOriginPose(), actual.getOriginPose(), epsilon))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Collisions (%s) do not have the same pose".formatted(expected.getName())),
                                     Objects.toString(expected.getOriginPose()),
                                     Objects.toString(actual.getOriginPose()));
      if (!Objects.equals(expected.getGeometryDefinition(), actual.getGeometryDefinition()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Collisions (%s) do not have the same geometry".formatted(expected.getName())),
                                     Objects.toString(expected.getGeometryDefinition()),
                                     Objects.toString(actual.getGeometryDefinition()));
      if (expected.isConcave() != actual.isConcave())
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Collisions (%s) do not have the concave property".formatted(expected.getName())),
                                     Objects.toString(expected.isConcave()),
                                     Objects.toString(actual.isConcave()));

      if (expected.getCollisionMask() != actual.getCollisionMask())
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Collisions (%s) do not have the collision mask".formatted(expected.getName())),
                                     Objects.toString(expected.getCollisionMask()),
                                     Objects.toString(actual.getCollisionMask()));

      if (expected.getCollisionGroup() != actual.getCollisionGroup())
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Collisions (%s) do not have the collision group".formatted(expected.getName())),
                                     Objects.toString(expected.getCollisionGroup()),
                                     Objects.toString(actual.getCollisionGroup()));
   }

   public static void assertJointEquals(JointDefinition expected, JointDefinition actual, double epsilon)
   {

      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError("Joints do not have the same name", expected.getName(), actual.getName());
      if (!EuclidCoreTools.epsilonEquals(expected.getTransformToParent(), actual.getTransformToParent(), epsilon))
         throwNotEqualAssertionError("Joints (%s) do not have the same transform".formatted(expected.getName()),
                                     Objects.toString(expected.getTransformToParent()),
                                     Objects.toString(actual.getTransformToParent()));

      if (expected instanceof OneDoFJointDefinition expectedOneDoFJoint)
      {
         OneDoFJointDefinition actualOneDoFJoint = (OneDoFJointDefinition) actual;

         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getAxis(), actualOneDoFJoint.getAxis(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same axis".formatted(expected.getName()),
                                        Objects.toString(expectedOneDoFJoint.getAxis()),
                                        Objects.toString(actualOneDoFJoint.getAxis()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getPositionLowerLimit(), actualOneDoFJoint.getPositionLowerLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same position lower limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getPositionLowerLimit()),
                                        Double.toString(actualOneDoFJoint.getPositionLowerLimit()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getPositionUpperLimit(), actualOneDoFJoint.getPositionUpperLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same position upper limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getPositionUpperLimit()),
                                        Double.toString(actualOneDoFJoint.getPositionUpperLimit()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getVelocityLowerLimit(), actualOneDoFJoint.getVelocityLowerLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same velocity lower limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getVelocityLowerLimit()),
                                        Double.toString(actualOneDoFJoint.getVelocityLowerLimit()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getVelocityUpperLimit(), actualOneDoFJoint.getVelocityUpperLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same velocity upper limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getVelocityUpperLimit()),
                                        Double.toString(actualOneDoFJoint.getVelocityUpperLimit()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getEffortLowerLimit(), actualOneDoFJoint.getEffortLowerLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same effort lower limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getEffortLowerLimit()),
                                        Double.toString(actualOneDoFJoint.getEffortLowerLimit()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getEffortUpperLimit(), actualOneDoFJoint.getEffortUpperLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same effort upper limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getEffortUpperLimit()),
                                        Double.toString(actualOneDoFJoint.getEffortUpperLimit()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getDamping(), actualOneDoFJoint.getDamping(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same damping".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getDamping()),
                                        Double.toString(actualOneDoFJoint.getDamping()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getStiction(), actualOneDoFJoint.getStiction(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same stiction".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getStiction()),
                                        Double.toString(actualOneDoFJoint.getStiction()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getKpSoftLimitStop(), actualOneDoFJoint.getKpSoftLimitStop(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same kp soft limit stop".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getKpSoftLimitStop()),
                                        Double.toString(actualOneDoFJoint.getKpSoftLimitStop()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getKdSoftLimitStop(), actualOneDoFJoint.getKdSoftLimitStop(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same kd soft limit stop".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getKdSoftLimitStop()),
                                        Double.toString(actualOneDoFJoint.getKdSoftLimitStop()));
         if (!EuclidCoreTools.epsilonEquals(expectedOneDoFJoint.getDampingVelocitySoftLimit(), actualOneDoFJoint.getDampingVelocitySoftLimit(), epsilon))
            throwNotEqualAssertionError("Joints (%s) do not have the same damping velocity soft limit".formatted(expected.getName()),
                                        Double.toString(expectedOneDoFJoint.getDampingVelocitySoftLimit()),
                                        Double.toString(actualOneDoFJoint.getDampingVelocitySoftLimit()));
      }

      if (!Objects.equals(expected.getSensorDefinitions().size(), actual.getSensorDefinitions().size()))
         throwNotEqualAssertionError("Joints (%s) do not have the same number of sensors".formatted(expected.getName()),
                                     Objects.toString(expected.getSensorDefinitions().size()),
                                     Objects.toString(actual.getSensorDefinitions().size()));

      Collections.sort(expected.getSensorDefinitions(), (s1, s2) -> s1.getName().compareTo(s2.getName()));
      Collections.sort(actual.getSensorDefinitions(), (s1, s2) -> s1.getName().compareTo(s2.getName()));

      for (int i = 0; i < expected.getSensorDefinitions().size(); i++)
      {
         SensorDefinition expectedSensor = expected.getSensorDefinitions().get(i);
         SensorDefinition actualSensor = actual.getSensorDefinitions().get(i);
         assertSensorEquals("In rigid-body (%s)".formatted(expected.getName()), expectedSensor, actualSensor, epsilon);
      }

      if (!Objects.equals(expected.getKinematicPointDefinitions().size(), actual.getKinematicPointDefinitions().size()))
         throwNotEqualAssertionError("Joints (%s) do not have the same number of kinematic points".formatted(expected.getName()),
                                     Objects.toString(expected.getKinematicPointDefinitions().size()),
                                     Objects.toString(actual.getKinematicPointDefinitions().size()));

      for (int i = 0; i < expected.getKinematicPointDefinitions().size(); i++)
      {
         KinematicPointDefinition expectedKinematicPoint = expected.getKinematicPointDefinitions().get(i);
         KinematicPointDefinition actualKinematicPoint = actual.getKinematicPointDefinitions().get(i);
         assertKinematicPointEquals("In rigid-body (%s)".formatted(expected.getName()), expectedKinematicPoint, actualKinematicPoint, epsilon);
      }

      if (!Objects.equals(expected.getExternalWrenchPointDefinitions().size(), actual.getExternalWrenchPointDefinitions().size()))
         throwNotEqualAssertionError("Joints (%s) do not have the same number of external wrench points".formatted(expected.getName()),
                                     Objects.toString(expected.getExternalWrenchPointDefinitions().size()),
                                     Objects.toString(actual.getExternalWrenchPointDefinitions().size()));

      for (int i = 0; i < expected.getExternalWrenchPointDefinitions().size(); i++)
      {
         ExternalWrenchPointDefinition expectedExternalWrenchPoint = expected.getExternalWrenchPointDefinitions().get(i);
         ExternalWrenchPointDefinition actualExternalWrenchPoint = actual.getExternalWrenchPointDefinitions().get(i);
         assertExternalWrenchPointEquals("In rigid-body (%s)".formatted(expected.getName()), expectedExternalWrenchPoint, actualExternalWrenchPoint, epsilon);
      }

      if (!Objects.equals(expected.getGroundContactPointDefinitions().size(), actual.getGroundContactPointDefinitions().size()))
         throwNotEqualAssertionError("Joints (%s) do not have the same number of ground contact points".formatted(expected.getName()),
                                     Objects.toString(expected.getGroundContactPointDefinitions().size()),
                                     Objects.toString(actual.getGroundContactPointDefinitions().size()));

      for (int i = 0; i < expected.getGroundContactPointDefinitions().size(); i++)
      {
         GroundContactPointDefinition expectedGroundContactPoint = expected.getGroundContactPointDefinitions().get(i);
         GroundContactPointDefinition actualGroundContactPoint = actual.getGroundContactPointDefinitions().get(i);
         assertGroundContactPointEquals("In rigid-body (%s)".formatted(expected.getName()), expectedGroundContactPoint, actualGroundContactPoint, epsilon);
      }

      // TODO Skipped the loop closure 
   }

   public static void assertSensorEquals(String messagePrefix, SensorDefinition expected, SensorDefinition actual, double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Sensors do not have the same name"), expected.getName(), actual.getName());

      if (!EuclidCoreTools.epsilonEquals(expected.getTransformToJoint(), actual.getTransformToJoint(), epsilon))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Sensors (%s) do not have the same pose".formatted(expected.getName())),
                                     Objects.toString(expected.getTransformToJoint()),
                                     Objects.toString(actual.getTransformToJoint()));

      if (expected.getUpdatePeriod() != actual.getUpdatePeriod())
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Sensors (%s) do not have the same udpate period".formatted(expected.getName())),
                                     Objects.toString(expected.getUpdatePeriod()),
                                     Objects.toString(actual.getUpdatePeriod()));

      // TODO Not bothering testing separately all the fields of all the sub-classes
   }

   public static void assertKinematicPointEquals(String messagePrefix, KinematicPointDefinition expected, KinematicPointDefinition actual, double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Kinematic points do not have the same name"), expected.getName(), actual.getName());

      if (!EuclidCoreTools.epsilonEquals(expected.getTransformToParent(), actual.getTransformToParent(), epsilon))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Kinematic points (%s) do not have the same pose".formatted(expected.getName())),
                                     Objects.toString(expected.getTransformToParent()),
                                     Objects.toString(actual.getTransformToParent()));
   }

   public static void assertExternalWrenchPointEquals(String messagePrefix,
                                                      ExternalWrenchPointDefinition expected,
                                                      ExternalWrenchPointDefinition actual,
                                                      double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "External wrench points do not have the same name"),
                                     expected.getName(),
                                     actual.getName());

      if (!EuclidCoreTools.epsilonEquals(expected.getTransformToParent(), actual.getTransformToParent(), epsilon))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "External wrench points (%s) do not have the same pose".formatted(expected.getName())),
                                     Objects.toString(expected.getTransformToParent()),
                                     Objects.toString(actual.getTransformToParent()));
   }

   public static void assertGroundContactPointEquals(String messagePrefix,
                                                     GroundContactPointDefinition expected,
                                                     GroundContactPointDefinition actual,
                                                     double epsilon)
   {
      if (!Objects.equals(expected.getName(), actual.getName()))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Ground contact points do not have the same name"),
                                     expected.getName(),
                                     actual.getName());

      if (!EuclidCoreTools.epsilonEquals(expected.getTransformToParent(), actual.getTransformToParent(), epsilon))
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix, "Ground contact points (%s) do not have the same pose".formatted(expected.getName())),
                                     Objects.toString(expected.getTransformToParent()),
                                     Objects.toString(actual.getTransformToParent()));

      if (expected.getGroupIdentifier() != actual.getGroupIdentifier())
         throwNotEqualAssertionError(addPrefixToMessage(messagePrefix,
                                                        "Ground contact points (%s) do not have the same group identifier".formatted(expected.getName())),
                                     Integer.toString(expected.getGroupIdentifier()),
                                     Integer.toString(actual.getGroupIdentifier()));
   }

   @Test
   public void testSaveLoadRobotDefinitionXML() throws JAXBException, FileNotFoundException, IOException
   {
      InputStream resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/valkyrie/valkyrie_sim.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition exportedRobotDefinition = URDFTools.toFloatingRobotDefinition(urdfModel);
      exportedRobotDefinition.simplifyKinematics();
      exportedRobotDefinition.transformsAllFrameToZUp();
      File testFile = new File("test.xml");
      DefinitionIOTools.saveRobotDefinition(new FileOutputStream(testFile), exportedRobotDefinition);
      RobotDefinition importedRobotDefinition = DefinitionIOTools.loadRobotDefinition(new FileInputStream(testFile));
      performAssertionsOnRobotDefinition(importedRobotDefinition);
      testFile.delete();
   }

   private void performAssertionsOnRobotDefinition(RobotDefinition robotDefinition)
   {
      for (String jointName : AllJointNames)
      {
         assertNotNull(robotDefinition.getJointDefinition(jointName), jointName);
         if (!jointName.equals(PelvisName))
            assertEquals(RevoluteJointDefinition.class, robotDefinition.getJointDefinition(jointName).getClass());
      }

      for (String linkName : AllLinkNames)
      {
         assertNotNull(robotDefinition.getRigidBodyDefinition(linkName), "Link: " + linkName);
      }

      assertEquals(1, robotDefinition.getRootJointDefinitions().size());
      assertEquals(SixDoFJointDefinition.class, robotDefinition.getRootJointDefinitions().get(0).getClass());
      assertEquals(PelvisName, robotDefinition.getRootJointDefinitions().get(0).getName());

      assertKinematicsContinuity(robotDefinition.getRootJointDefinitions().get(0), LeftLegJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getRootJointDefinitions().get(0), RightLegJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getRootJointDefinitions().get(0), TorsoJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(TorsoRollName), NeckJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(TorsoRollName), LeftArmJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(TorsoRollName), RightArmJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(LeftWristPitchName), LeftIndexFingerJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(LeftWristPitchName), LeftMiddleFingerJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(LeftWristPitchName), LeftPinkyJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(LeftWristPitchName), LeftThumbJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(RightWristPitchName), RightIndexFingerJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(RightWristPitchName), RightMiddleFingerJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(RightWristPitchName), RightPinkyJointNames, robotDefinition);
      assertKinematicsContinuity(robotDefinition.getJointDefinition(RightWristPitchName), RightThumbJointNames, robotDefinition);

      assertPhysicalProperties(robotDefinition, valkyrieProperties(), subtract(AllJointNames, HokuyoJointName), AllLinkNames);
      assertSensorsProperties(robotDefinition, valkyrieSensorProperties(), AllJointNames);
   }

   public static void assertPhysicalProperties(RobotDefinition robotDefinition,
                                               Map<String, Map<String, Object>> robotProperties,
                                               String[] allJointNames,
                                               String[] allLinkNames)
   {
      for (String jointName : allJointNames)
      {
         Map<String, Object> jointProperties = robotProperties.get(jointName);
         JointDefinition jointDefinition = robotDefinition.getJointDefinition(jointName);
         String messagePrefix = "Joint: " + jointName;
         Orientation3DBasics rotation = jointDefinition.getTransformToParent().getRotation();
         Vector3D translation = jointDefinition.getTransformToParent().getTranslation();
         assertTrue(rotation.isZeroOrientation(EPSILON), "Expected zero rotation, was: " + rotation);
         EuclidCoreTestTools.assertEquals(messagePrefix, (Tuple3DReadOnly) jointProperties.get("offsetFromParentJoint"), translation, EPSILON);
         if (jointDefinition instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDoFJointDefinition = (OneDoFJointDefinition) jointDefinition;
            assertEquals((double) jointProperties.get("positionLowerLimit"), oneDoFJointDefinition.getPositionLowerLimit(), messagePrefix);
            assertEquals((double) jointProperties.get("positionUpperLimit"), oneDoFJointDefinition.getPositionUpperLimit(), messagePrefix);
            assertEquals((double) jointProperties.get("velocityLowerLimit"), oneDoFJointDefinition.getVelocityLowerLimit(), messagePrefix);
            assertEquals((double) jointProperties.get("velocityUpperLimit"), oneDoFJointDefinition.getVelocityUpperLimit(), messagePrefix);
            assertEquals((double) jointProperties.get("effortLowerLimit"), oneDoFJointDefinition.getEffortLowerLimit(), messagePrefix);
            assertEquals((double) jointProperties.get("effortUpperLimit"), oneDoFJointDefinition.getEffortUpperLimit(), messagePrefix);
            EuclidCoreTestTools.assertEquals(messagePrefix, (Tuple3DReadOnly) jointProperties.get("axis"), oneDoFJointDefinition.getAxis(), EPSILON);
            assertEquals((double) jointProperties.get("damping"), oneDoFJointDefinition.getDamping(), messagePrefix);
            assertEquals((double) jointProperties.get("stiction"), oneDoFJointDefinition.getStiction(), messagePrefix);
         }
      }

      for (String linkName : allLinkNames)
      {
         String messagePrefix = "Link: " + linkName;
         Map<String, Object> linkProperties = robotProperties.get(linkName);
         RigidBodyDefinition rigidBodyDefinition = robotDefinition.getRigidBodyDefinition(linkName);
         assertEquals((double) linkProperties.get("mass"), rigidBodyDefinition.getMass(), EPSILON, messagePrefix);
         EuclidCoreTestTools.assertEquals(messagePrefix,
                                          (Tuple3DReadOnly) linkProperties.get("centerOfMass"),
                                          rigidBodyDefinition.getCenterOfMassOffset(),
                                          EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(messagePrefix,
                                                  (Matrix3DReadOnly) linkProperties.get("inertia"),
                                                  rigidBodyDefinition.getMomentOfInertia(),
                                                  EPSILON);
      }
   }

   public static void assertSensorsProperties(RobotDefinition robotDefinition, Map<String, Map<String, Object>> robotSensorProperties, String[] allJointNames)
   {
      int actualNumberOfCameras = 0;
      int actualNumberOfIMUs = 0;
      int actualNumberOfLidars = 0;

      for (String jointName : allJointNames)
      {
         JointDefinition jointDefinition = robotDefinition.getJointDefinition(jointName);

         actualNumberOfCameras += jointDefinition.getSensorDefinitions(CameraSensorDefinition.class).size();
         actualNumberOfIMUs += jointDefinition.getSensorDefinitions(IMUSensorDefinition.class).size();
         actualNumberOfLidars += jointDefinition.getSensorDefinitions(LidarSensorDefinition.class).size();

         for (SensorDefinition sensorDefinition : jointDefinition.getSensorDefinitions())
         {
            Map<String, Object> sensorProperties = robotSensorProperties.get(sensorDefinition.getName());
            assertSensorProperties(sensorDefinition, sensorProperties);
         }
      }

      int expectedNumberOfCameras = (int) robotSensorProperties.values()
                                                               .stream()
                                                               .flatMap(sensorProperties -> sensorProperties.entrySet().stream())
                                                               .filter(entry -> entry.getKey().equals("imageWidth"))
                                                               .count();
      int expectedNumberOfIMUs = (int) robotSensorProperties.values()
                                                            .stream()
                                                            .flatMap(sensorProperties -> sensorProperties.entrySet().stream())
                                                            .filter(entry -> entry.getKey().equals("accelerationNoiseMean"))
                                                            .count();
      int expectedNumberOfLidars = (int) robotSensorProperties.values()
                                                              .stream()
                                                              .flatMap(sensorProperties -> sensorProperties.entrySet().stream())
                                                              .filter(entry -> entry.getKey().equals("sweepYawMin"))
                                                              .count();
      assertEquals(expectedNumberOfCameras, actualNumberOfCameras);
      assertEquals(expectedNumberOfIMUs, actualNumberOfIMUs);
      assertEquals(expectedNumberOfLidars, actualNumberOfLidars);
   }

   public static void assertSensorProperties(SensorDefinition sensorDefinition, Map<String, Object> sensorProperties)
   {
      if (sensorProperties == null)
         return;

      EuclidCoreTestTools.assertEquals("Sensor: " + sensorDefinition.getName(),
                                       (RigidBodyTransform) sensorProperties.get("transformToJoint"),
                                       new RigidBodyTransform(sensorDefinition.getTransformToJoint()),
                                       EPSILON);

      if (sensorDefinition instanceof CameraSensorDefinition)
      {
         CameraSensorDefinition cameraSensorDefinition = (CameraSensorDefinition) sensorDefinition;
         assertEquals((double) sensorProperties.get("fieldOfView"), cameraSensorDefinition.getFieldOfView());
         assertEquals((double) sensorProperties.get("clipNear"), cameraSensorDefinition.getClipNear());
         assertEquals((double) sensorProperties.get("clipFar"), cameraSensorDefinition.getClipFar());
         assertEquals((int) sensorProperties.get("imageWidth"), cameraSensorDefinition.getImageWidth());
         assertEquals((int) sensorProperties.get("imageHeight"), cameraSensorDefinition.getImageHeight());
      }
      else if (sensorDefinition instanceof IMUSensorDefinition)
      {
         IMUSensorDefinition imuSensorDefinition = (IMUSensorDefinition) sensorDefinition;
         assertEquals((double) sensorProperties.get("accelerationNoiseMean"), imuSensorDefinition.getAccelerationNoiseMean());
         assertEquals((double) sensorProperties.get("accelerationNoiseStandardDeviation"), imuSensorDefinition.getAccelerationNoiseStandardDeviation());
         assertEquals((double) sensorProperties.get("accelerationBiasMean"), imuSensorDefinition.getAccelerationBiasMean());
         assertEquals((double) sensorProperties.get("accelerationBiasStandardDeviation"), imuSensorDefinition.getAccelerationBiasStandardDeviation());
         assertEquals((double) sensorProperties.get("angularVelocityNoiseMean"), imuSensorDefinition.getAngularVelocityNoiseMean());
         assertEquals((double) sensorProperties.get("angularVelocityNoiseStandardDeviation"), imuSensorDefinition.getAngularVelocityNoiseStandardDeviation());
         assertEquals((double) sensorProperties.get("angularVelocityBiasMean"), imuSensorDefinition.getAngularVelocityBiasMean());
         assertEquals((double) sensorProperties.get("angularVelocityBiasStandardDeviation"), imuSensorDefinition.getAngularVelocityBiasStandardDeviation());
      }
      else if (sensorDefinition instanceof LidarSensorDefinition)
      {
         LidarSensorDefinition lidarSensorDefinition = (LidarSensorDefinition) sensorDefinition;
         assertEquals((double) sensorProperties.get("sweepYawMin"), lidarSensorDefinition.getSweepYawMin());
         assertEquals((double) sensorProperties.get("sweepYawMax"), lidarSensorDefinition.getSweepYawMax());
         assertEquals((double) sensorProperties.get("heightPitchMin"), lidarSensorDefinition.getHeightPitchMin());
         assertEquals((double) sensorProperties.get("heightPitchMax"), lidarSensorDefinition.getHeightPitchMax());
         assertEquals((double) sensorProperties.get("minRange"), lidarSensorDefinition.getMinRange());
         assertEquals((double) sensorProperties.get("maxRange"), lidarSensorDefinition.getMaxRange());
         assertEquals((int) sensorProperties.get("pointsPerSweep"), lidarSensorDefinition.getPointsPerSweep());
         assertEquals((int) sensorProperties.get("scanHeight"), lidarSensorDefinition.getScanHeight());
      }
   }

   public static void assertKinematicsContinuity(JointDefinition expectedParentJoint, String[] jointNames, RobotDefinition robotDefinition)
   {
      for (String jointName : jointNames)
      {
         JointDefinition joint = robotDefinition.getJointDefinition(jointName);
         assertNotNull(joint.getParentJoint(), jointName);
         assertTrue(expectedParentJoint == joint.getParentJoint());
         assertTrue(expectedParentJoint.getSuccessor().getChildrenJoints().contains(joint));
         expectedParentJoint = joint;
      }
   }

   @SuppressWarnings({"unchecked"})
   public static <T> T[] concatenate(T[]... arrays)
   {
      int length = Stream.of(arrays).mapToInt(array -> array.length).sum();
      T[] ret = (T[]) Array.newInstance(arrays[0].getClass().getComponentType(), length);

      int currentIndex = 0;

      for (T[] array : arrays)
      {
         for (T element : array)
         {
            ret[currentIndex++] = element;
         }
      }
      return ret;
   }

   @SuppressWarnings("unchecked")
   @SafeVarargs
   public static <T> T[] subtract(T[] source, T... elementsToSubtract)
   {
      HashSet<T> sourceSet = new LinkedHashSet<>(Arrays.asList(source));
      for (T elementToSubtract : elementsToSubtract)
      {
         sourceSet.remove(elementToSubtract);
      }
      return sourceSet.toArray((T[]) Array.newInstance(source.getClass().getComponentType(), sourceSet.size()));
   }

   private static final double Infinity = Double.POSITIVE_INFINITY;

   private static Map<String, Map<String, Object>> valkyrieProperties()
   {
      // Generated from working parser.
      Map<String, Map<String, Object>> properties = new HashMap<>();
      properties.put("pelvis", new HashMap<>());
      properties.put("pelvis", new HashMap<>());
      properties.get("pelvis").put("mass", 8.22);
      properties.get("pelvis").put("centerOfMass", new Vector3D(-0.00532, -0.003512, -0.0036));
      properties.get("pelvis")
                .put("inertia", new Matrix3D(0.118664, -1.43482E-4, 0.00327129, -1.43482E-4, 0.0979634, 0.00215955, 0.00327129, 0.00215955, 0.0838546));
      properties.get("pelvis").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.put("leftHipYaw", new HashMap<>());
      properties.put("leftHipYawLink", new HashMap<>());
      properties.get("leftHipYawLink").put("mass", 2.39);
      properties.get("leftHipYawLink").put("centerOfMass", new Vector3D(0.02176, -0.00131, 0.03867));
      properties.get("leftHipYawLink").put("inertia", new Matrix3D(0.017261, 0.0, 0.0, 0.0, 0.0148662, 0.0, 0.0, 0.0, 0.0112382));
      properties.get("leftHipYaw").put("offsetFromParentJoint", new Vector3D(0.0, 0.1016, -0.1853));
      properties.get("leftHipYaw").put("positionLowerLimit", -0.4141);
      properties.get("leftHipYaw").put("positionUpperLimit", 1.1);
      properties.get("leftHipYaw").put("velocityLowerLimit", -5.89);
      properties.get("leftHipYaw").put("velocityUpperLimit", 5.89);
      properties.get("leftHipYaw").put("effortLowerLimit", -190.0);
      properties.get("leftHipYaw").put("effortUpperLimit", 190.0);
      properties.get("leftHipYaw").put("kpPositionLimit", 100.0);
      properties.get("leftHipYaw").put("kdPositionLimit", 20.0);
      properties.get("leftHipYaw").put("kpVelocityLimit", 500.0);
      properties.get("leftHipYaw").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("leftHipYaw").put("damping", 0.1);
      properties.get("leftHipYaw").put("stiction", 0.0);
      properties.put("leftHipRoll", new HashMap<>());
      properties.put("leftHipRollLink", new HashMap<>());
      properties.get("leftHipRollLink").put("mass", 3.665);
      properties.get("leftHipRollLink").put("centerOfMass", new Vector3D(0.012959, 0.00755, -0.01595));
      properties.get("leftHipRollLink")
                .put("inertia", new Matrix3D(0.00597896, -2.34823E-4, 5.53962E-4, -2.34823E-4, 0.00937265, 7.78956E-4, 5.53962E-4, 7.78956E-4, 0.00819312));
      properties.get("leftHipRoll").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("leftHipRoll").put("positionLowerLimit", -0.467);
      properties.get("leftHipRoll").put("positionUpperLimit", 0.5515);
      properties.get("leftHipRoll").put("velocityLowerLimit", -7.0);
      properties.get("leftHipRoll").put("velocityUpperLimit", 7.0);
      properties.get("leftHipRoll").put("effortLowerLimit", -350.0);
      properties.get("leftHipRoll").put("effortUpperLimit", 350.0);
      properties.get("leftHipRoll").put("kpPositionLimit", 100.0);
      properties.get("leftHipRoll").put("kdPositionLimit", 20.0);
      properties.get("leftHipRoll").put("kpVelocityLimit", 500.0);
      properties.get("leftHipRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftHipRoll").put("damping", 0.1);
      properties.get("leftHipRoll").put("stiction", 0.0);
      properties.put("leftHipPitch", new HashMap<>());
      properties.put("leftHipPitchLink", new HashMap<>());
      properties.get("leftHipPitchLink").put("mass", 10.2);
      properties.get("leftHipPitchLink").put("centerOfMass", new Vector3D(0.016691, 0.091397, -0.207875));
      properties.get("leftHipPitchLink")
                .put("inertia", new Matrix3D(0.240834, 3.5915E-5, 0.00369938, 3.5915E-5, 0.256897, -0.001333, 0.00369938, -0.001333, 0.0232764));
      properties.get("leftHipPitch").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, -0.06090000000000001));
      properties.get("leftHipPitch").put("positionLowerLimit", -2.42);
      properties.get("leftHipPitch").put("positionUpperLimit", 1.619);
      properties.get("leftHipPitch").put("velocityLowerLimit", -6.11);
      properties.get("leftHipPitch").put("velocityUpperLimit", 6.11);
      properties.get("leftHipPitch").put("effortLowerLimit", -350.0);
      properties.get("leftHipPitch").put("effortUpperLimit", 350.0);
      properties.get("leftHipPitch").put("kpPositionLimit", 100.0);
      properties.get("leftHipPitch").put("kdPositionLimit", 20.0);
      properties.get("leftHipPitch").put("kpVelocityLimit", 500.0);
      properties.get("leftHipPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("leftHipPitch").put("damping", 0.1);
      properties.get("leftHipPitch").put("stiction", 0.0);
      properties.put("leftKneePitch", new HashMap<>());
      properties.put("leftKneePitchLink", new HashMap<>());
      properties.get("leftKneePitchLink").put("mass", 6.2);
      properties.get("leftKneePitchLink").put("centerOfMass", new Vector3D(-0.022183, 0.001703, -0.189418));
      properties.get("leftKneePitchLink")
                .put("inertia", new Matrix3D(0.0869357, 9.929E-5, 5.73207E-4, 9.929E-5, 0.0915841, 3.13745E-4, 5.73207E-4, 3.13745E-4, 0.0140173));
      properties.get("leftKneePitch").put("offsetFromParentJoint", new Vector3D(1.12225E-4, 0.036105, -0.430959));
      properties.get("leftKneePitch").put("positionLowerLimit", -0.083);
      properties.get("leftKneePitch").put("positionUpperLimit", 2.057);
      properties.get("leftKneePitch").put("velocityLowerLimit", -6.11);
      properties.get("leftKneePitch").put("velocityUpperLimit", 6.11);
      properties.get("leftKneePitch").put("effortLowerLimit", -350.0);
      properties.get("leftKneePitch").put("effortUpperLimit", 350.0);
      properties.get("leftKneePitch").put("kpPositionLimit", 100.0);
      properties.get("leftKneePitch").put("kdPositionLimit", 20.0);
      properties.get("leftKneePitch").put("kpVelocityLimit", 500.0);
      properties.get("leftKneePitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("leftKneePitch").put("damping", 0.1);
      properties.get("leftKneePitch").put("stiction", 0.0);
      properties.put("leftAnklePitch", new HashMap<>());
      properties.put("leftAnklePitchLink", new HashMap<>());
      properties.get("leftAnklePitchLink").put("mass", 0.03);
      properties.get("leftAnklePitchLink").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("leftAnklePitchLink").put("inertia", new Matrix3D(4.377E-6, 0.0, 0.0, 0.0, 4.322E-6, 0.0, 0.0, 0.0, 7.015E-6));
      properties.get("leftAnklePitch").put("offsetFromParentJoint", new Vector3D(-0.010238125, 0.0, -0.40627099999999994));
      properties.get("leftAnklePitch").put("positionLowerLimit", -0.8644);
      properties.get("leftAnklePitch").put("positionUpperLimit", 0.875);
      properties.get("leftAnklePitch").put("velocityLowerLimit", -11.0);
      properties.get("leftAnklePitch").put("velocityUpperLimit", 11.0);
      properties.get("leftAnklePitch").put("effortLowerLimit", -205.0);
      properties.get("leftAnklePitch").put("effortUpperLimit", 205.0);
      properties.get("leftAnklePitch").put("kpPositionLimit", 100.0);
      properties.get("leftAnklePitch").put("kdPositionLimit", 20.0);
      properties.get("leftAnklePitch").put("kpVelocityLimit", 500.0);
      properties.get("leftAnklePitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("leftAnklePitch").put("damping", 0.1);
      properties.get("leftAnklePitch").put("stiction", 0.0);
      properties.put("leftAnkleRoll", new HashMap<>());
      properties.put("leftFoot", new HashMap<>());
      properties.get("leftFoot").put("mass", 2.37);
      properties.get("leftFoot").put("centerOfMass", new Vector3D(0.0369087, 0.00494324, -0.0489279));
      properties.get("leftFoot")
                .put("inertia", new Matrix3D(0.00641532, 2.0788E-4, 0.00128536, 2.0788E-4, 0.0179943, -2.02908E-4, 0.00128536, -2.02908E-4, 0.0209358));
      properties.get("leftAnkleRoll").put("offsetFromParentJoint", new Vector3D(0.0101259, 0.0, 0.0));
      properties.get("leftAnkleRoll").put("positionLowerLimit", -0.349);
      properties.get("leftAnkleRoll").put("positionUpperLimit", 0.348);
      properties.get("leftAnkleRoll").put("velocityLowerLimit", -11.0);
      properties.get("leftAnkleRoll").put("velocityUpperLimit", 11.0);
      properties.get("leftAnkleRoll").put("effortLowerLimit", -205.0);
      properties.get("leftAnkleRoll").put("effortUpperLimit", 205.0);
      properties.get("leftAnkleRoll").put("kpPositionLimit", 100.0);
      properties.get("leftAnkleRoll").put("kdPositionLimit", 20.0);
      properties.get("leftAnkleRoll").put("kpVelocityLimit", 500.0);
      properties.get("leftAnkleRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftAnkleRoll").put("damping", 0.3);
      properties.get("leftAnkleRoll").put("stiction", 0.0);
      properties.put("rightHipYaw", new HashMap<>());
      properties.put("rightHipYawLink", new HashMap<>());
      properties.get("rightHipYawLink").put("mass", 2.39);
      properties.get("rightHipYawLink").put("centerOfMass", new Vector3D(0.02176, 0.00131, 0.03867));
      properties.get("rightHipYawLink").put("inertia", new Matrix3D(0.017261, 0.0, 0.0, 0.0, 0.0148662, 0.0, 0.0, 0.0, 0.0112382));
      properties.get("rightHipYaw").put("offsetFromParentJoint", new Vector3D(0.0, -0.1016, -0.1853));
      properties.get("rightHipYaw").put("positionLowerLimit", -1.1);
      properties.get("rightHipYaw").put("positionUpperLimit", 0.4141);
      properties.get("rightHipYaw").put("velocityLowerLimit", -5.89);
      properties.get("rightHipYaw").put("velocityUpperLimit", 5.89);
      properties.get("rightHipYaw").put("effortLowerLimit", -190.0);
      properties.get("rightHipYaw").put("effortUpperLimit", 190.0);
      properties.get("rightHipYaw").put("kpPositionLimit", 100.0);
      properties.get("rightHipYaw").put("kdPositionLimit", 20.0);
      properties.get("rightHipYaw").put("kpVelocityLimit", 500.0);
      properties.get("rightHipYaw").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("rightHipYaw").put("damping", 0.1);
      properties.get("rightHipYaw").put("stiction", 0.0);
      properties.put("rightHipRoll", new HashMap<>());
      properties.put("rightHipRollLink", new HashMap<>());
      properties.get("rightHipRollLink").put("mass", 3.665);
      properties.get("rightHipRollLink").put("centerOfMass", new Vector3D(0.012959, -0.00755, -0.01595));
      properties.get("rightHipRollLink")
                .put("inertia", new Matrix3D(0.00597896, 2.34823E-4, 5.53962E-4, 2.34823E-4, 0.00937265, -7.78956E-4, 5.53962E-4, -7.78956E-4, 0.00819312));
      properties.get("rightHipRoll").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("rightHipRoll").put("positionLowerLimit", -0.5515);
      properties.get("rightHipRoll").put("positionUpperLimit", 0.467);
      properties.get("rightHipRoll").put("velocityLowerLimit", -7.0);
      properties.get("rightHipRoll").put("velocityUpperLimit", 7.0);
      properties.get("rightHipRoll").put("effortLowerLimit", -350.0);
      properties.get("rightHipRoll").put("effortUpperLimit", 350.0);
      properties.get("rightHipRoll").put("kpPositionLimit", 100.0);
      properties.get("rightHipRoll").put("kdPositionLimit", 20.0);
      properties.get("rightHipRoll").put("kpVelocityLimit", 500.0);
      properties.get("rightHipRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightHipRoll").put("damping", 0.1);
      properties.get("rightHipRoll").put("stiction", 0.0);
      properties.put("rightHipPitch", new HashMap<>());
      properties.put("rightHipPitchLink", new HashMap<>());
      properties.get("rightHipPitchLink").put("mass", 10.2);
      properties.get("rightHipPitchLink").put("centerOfMass", new Vector3D(0.016691, -0.091397, -0.207875));
      properties.get("rightHipPitchLink")
                .put("inertia", new Matrix3D(0.240834, -3.5915E-5, 0.00369938, -3.5915E-5, 0.256897, 0.001333, 0.00369938, 0.001333, 0.0232764));
      properties.get("rightHipPitch").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, -0.06090000000000001));
      properties.get("rightHipPitch").put("positionLowerLimit", -2.42);
      properties.get("rightHipPitch").put("positionUpperLimit", 1.619);
      properties.get("rightHipPitch").put("velocityLowerLimit", -6.11);
      properties.get("rightHipPitch").put("velocityUpperLimit", 6.11);
      properties.get("rightHipPitch").put("effortLowerLimit", -350.0);
      properties.get("rightHipPitch").put("effortUpperLimit", 350.0);
      properties.get("rightHipPitch").put("kpPositionLimit", 100.0);
      properties.get("rightHipPitch").put("kdPositionLimit", 20.0);
      properties.get("rightHipPitch").put("kpVelocityLimit", 500.0);
      properties.get("rightHipPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("rightHipPitch").put("damping", 0.1);
      properties.get("rightHipPitch").put("stiction", 0.0);
      properties.put("rightKneePitch", new HashMap<>());
      properties.put("rightKneePitchLink", new HashMap<>());
      properties.get("rightKneePitchLink").put("mass", 6.2);
      properties.get("rightKneePitchLink").put("centerOfMass", new Vector3D(-0.022183, 0.001703, -0.189418));
      properties.get("rightKneePitchLink")
                .put("inertia", new Matrix3D(0.0869357, 9.929E-5, 5.73207E-4, 9.929E-5, 0.0915841, 3.13745E-4, 5.73207E-4, 3.13745E-4, 0.0140173));
      properties.get("rightKneePitch").put("offsetFromParentJoint", new Vector3D(1.12225E-4, -0.036105, -0.430959));
      properties.get("rightKneePitch").put("positionLowerLimit", -0.083);
      properties.get("rightKneePitch").put("positionUpperLimit", 2.057);
      properties.get("rightKneePitch").put("velocityLowerLimit", -6.11);
      properties.get("rightKneePitch").put("velocityUpperLimit", 6.11);
      properties.get("rightKneePitch").put("effortLowerLimit", -350.0);
      properties.get("rightKneePitch").put("effortUpperLimit", 350.0);
      properties.get("rightKneePitch").put("kpPositionLimit", 100.0);
      properties.get("rightKneePitch").put("kdPositionLimit", 20.0);
      properties.get("rightKneePitch").put("kpVelocityLimit", 500.0);
      properties.get("rightKneePitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("rightKneePitch").put("damping", 0.1);
      properties.get("rightKneePitch").put("stiction", 0.0);
      properties.put("rightAnklePitch", new HashMap<>());
      properties.put("rightAnklePitchLink", new HashMap<>());
      properties.get("rightAnklePitchLink").put("mass", 0.03);
      properties.get("rightAnklePitchLink").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("rightAnklePitchLink").put("inertia", new Matrix3D(4.377E-6, 0.0, 0.0, 0.0, 4.322E-6, 0.0, 0.0, 0.0, 7.015E-6));
      properties.get("rightAnklePitch").put("offsetFromParentJoint", new Vector3D(-0.010238125, 0.0, -0.40627099999999994));
      properties.get("rightAnklePitch").put("positionLowerLimit", -0.8644);
      properties.get("rightAnklePitch").put("positionUpperLimit", 0.875);
      properties.get("rightAnklePitch").put("velocityLowerLimit", -11.0);
      properties.get("rightAnklePitch").put("velocityUpperLimit", 11.0);
      properties.get("rightAnklePitch").put("effortLowerLimit", -205.0);
      properties.get("rightAnklePitch").put("effortUpperLimit", 205.0);
      properties.get("rightAnklePitch").put("kpPositionLimit", 100.0);
      properties.get("rightAnklePitch").put("kdPositionLimit", 20.0);
      properties.get("rightAnklePitch").put("kpVelocityLimit", 500.0);
      properties.get("rightAnklePitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("rightAnklePitch").put("damping", 0.1);
      properties.get("rightAnklePitch").put("stiction", 0.0);
      properties.put("rightAnkleRoll", new HashMap<>());
      properties.put("rightFoot", new HashMap<>());
      properties.get("rightFoot").put("mass", 2.37);
      properties.get("rightFoot").put("centerOfMass", new Vector3D(0.0369087, 0.00494324, -0.0489279));
      properties.get("rightFoot")
                .put("inertia", new Matrix3D(0.00641532, 2.0788E-4, 0.00128536, 2.0788E-4, 0.0179943, -2.02908E-4, 0.00128536, -2.02908E-4, 0.0209358));
      properties.get("rightAnkleRoll").put("offsetFromParentJoint", new Vector3D(0.0101259, 0.0, 0.0));
      properties.get("rightAnkleRoll").put("positionLowerLimit", -0.349);
      properties.get("rightAnkleRoll").put("positionUpperLimit", 0.348);
      properties.get("rightAnkleRoll").put("velocityLowerLimit", -11.0);
      properties.get("rightAnkleRoll").put("velocityUpperLimit", 11.0);
      properties.get("rightAnkleRoll").put("effortLowerLimit", -205.0);
      properties.get("rightAnkleRoll").put("effortUpperLimit", 205.0);
      properties.get("rightAnkleRoll").put("kpPositionLimit", 100.0);
      properties.get("rightAnkleRoll").put("kdPositionLimit", 20.0);
      properties.get("rightAnkleRoll").put("kpVelocityLimit", 500.0);
      properties.get("rightAnkleRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightAnkleRoll").put("damping", 0.3);
      properties.get("rightAnkleRoll").put("stiction", 0.0);
      properties.put("torsoYaw", new HashMap<>());
      properties.put("torsoYawLink", new HashMap<>());
      properties.get("torsoYawLink").put("mass", 0.5);
      properties.get("torsoYawLink").put("centerOfMass", new Vector3D(0.0, 0.0, -0.01));
      properties.get("torsoYawLink")
                .put("inertia", new Matrix3D(6.08427E-4, -1.172E-6, 1.647E-6, -1.172E-6, 6.20328E-4, -2.33E-7, 1.647E-6, -2.33E-7, 0.00107811));
      properties.get("torsoYaw").put("offsetFromParentJoint", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("torsoYaw").put("positionLowerLimit", -1.329);
      properties.get("torsoYaw").put("positionUpperLimit", 1.181);
      properties.get("torsoYaw").put("velocityLowerLimit", -5.89);
      properties.get("torsoYaw").put("velocityUpperLimit", 5.89);
      properties.get("torsoYaw").put("effortLowerLimit", -190.0);
      properties.get("torsoYaw").put("effortUpperLimit", 190.0);
      properties.get("torsoYaw").put("kpPositionLimit", 100.0);
      properties.get("torsoYaw").put("kdPositionLimit", 20.0);
      properties.get("torsoYaw").put("kpVelocityLimit", 500.0);
      properties.get("torsoYaw").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("torsoYaw").put("damping", 0.1);
      properties.get("torsoYaw").put("stiction", 0.0);
      properties.put("torsoPitch", new HashMap<>());
      properties.put("torsoPitchLink", new HashMap<>());
      properties.get("torsoPitchLink").put("mass", 0.1);
      properties.get("torsoPitchLink").put("centerOfMass", new Vector3D(0.0, 0.0, 0.005));
      properties.get("torsoPitchLink").put("inertia", new Matrix3D(3.032E-5, 0.0, -1.145E-6, 0.0, 2.1274E-5, 0.0, -1.145E-6, 0.0, 2.8285E-5));
      properties.get("torsoPitch").put("offsetFromParentJoint", new Vector3D(0.04191, 0.0, 0.0));
      properties.get("torsoPitch").put("positionLowerLimit", -0.13);
      properties.get("torsoPitch").put("positionUpperLimit", 0.666);
      properties.get("torsoPitch").put("velocityLowerLimit", -9.0);
      properties.get("torsoPitch").put("velocityUpperLimit", 9.0);
      properties.get("torsoPitch").put("effortLowerLimit", -150.0);
      properties.get("torsoPitch").put("effortUpperLimit", 150.0);
      properties.get("torsoPitch").put("kpPositionLimit", 100.0);
      properties.get("torsoPitch").put("kdPositionLimit", 20.0);
      properties.get("torsoPitch").put("kpVelocityLimit", 500.0);
      properties.get("torsoPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("torsoPitch").put("damping", 0.1);
      properties.get("torsoPitch").put("stiction", 0.0);
      properties.put("torsoRoll", new HashMap<>());
      properties.put("torso", new HashMap<>());
      properties.get("torso").put("mass", 39.47);
      properties.get("torso").put("centerOfMass", new Vector3D(-0.095548, -0.003337, 0.243098));
      properties.get("torso").put("inertia", new Matrix3D(0.873269, 9.95625E-5, 0.0613452, 9.95625E-5, 1.01085, 0.00181849, 0.0613452, 0.00181849, 0.778398));
      properties.get("torsoRoll").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0203));
      properties.get("torsoRoll").put("positionLowerLimit", -0.23);
      properties.get("torsoRoll").put("positionUpperLimit", 0.255);
      properties.get("torsoRoll").put("velocityLowerLimit", -9.0);
      properties.get("torsoRoll").put("velocityUpperLimit", 9.0);
      properties.get("torsoRoll").put("effortLowerLimit", -150.0);
      properties.get("torsoRoll").put("effortUpperLimit", 150.0);
      properties.get("torsoRoll").put("kpPositionLimit", 100.0);
      properties.get("torsoRoll").put("kdPositionLimit", 20.0);
      properties.get("torsoRoll").put("kpVelocityLimit", 500.0);
      properties.get("torsoRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("torsoRoll").put("damping", 0.1);
      properties.get("torsoRoll").put("stiction", 0.0);
      properties.put("leftShoulderPitch", new HashMap<>());
      properties.put("leftShoulderPitchLink", new HashMap<>());
      properties.get("leftShoulderPitchLink").put("mass", 2.65);
      properties.get("leftShoulderPitchLink").put("centerOfMass", new Vector3D(-0.012, 0.251, 0.0));
      properties.get("leftShoulderPitchLink").put("inertia", new Matrix3D(0.0137182, 0.0, 0.0, 0.0, 0.0105028, 0.0, 0.0, 0.0, 0.0148064));
      properties.get("leftShoulderPitch").put("offsetFromParentJoint", new Vector3D(-0.0316, 0.0, 0.2984));
      properties.get("leftShoulderPitch").put("positionLowerLimit", -2.85);
      properties.get("leftShoulderPitch").put("positionUpperLimit", 2.0);
      properties.get("leftShoulderPitch").put("velocityLowerLimit", -3.0);
      properties.get("leftShoulderPitch").put("velocityUpperLimit", 3.0);
      properties.get("leftShoulderPitch").put("effortLowerLimit", -190.0);
      properties.get("leftShoulderPitch").put("effortUpperLimit", 190.0);
      properties.get("leftShoulderPitch").put("kpPositionLimit", 100.0);
      properties.get("leftShoulderPitch").put("kdPositionLimit", 20.0);
      properties.get("leftShoulderPitch").put("kpVelocityLimit", 500.0);
      properties.get("leftShoulderPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("leftShoulderPitch").put("damping", 0.1);
      properties.get("leftShoulderPitch").put("stiction", 0.0);
      properties.put("leftShoulderRoll", new HashMap<>());
      properties.put("leftShoulderRollLink", new HashMap<>());
      properties.get("leftShoulderRollLink").put("mass", 3.97);
      properties.get("leftShoulderRollLink").put("centerOfMass", new Vector3D(-0.008513, 0.02068, -0.001088));
      properties.get("leftShoulderRollLink")
                .put("inertia", new Matrix3D(0.0145988, -6.6764E-4, -3.629E-5, -6.6764E-4, 0.00645214, -8.283E-5, -3.629E-5, -8.283E-5, 0.0168483));
      properties.get("leftShoulderRoll").put("offsetFromParentJoint", new Vector3D(0.0, 0.2499, 0.0));
      properties.get("leftShoulderRoll").put("positionLowerLimit", -1.519);
      properties.get("leftShoulderRoll").put("positionUpperLimit", 1.266);
      properties.get("leftShoulderRoll").put("velocityLowerLimit", -3.5);
      properties.get("leftShoulderRoll").put("velocityUpperLimit", 3.5);
      properties.get("leftShoulderRoll").put("effortLowerLimit", -350.0);
      properties.get("leftShoulderRoll").put("effortUpperLimit", 350.0);
      properties.get("leftShoulderRoll").put("kpPositionLimit", 100.0);
      properties.get("leftShoulderRoll").put("kdPositionLimit", 20.0);
      properties.get("leftShoulderRoll").put("kpVelocityLimit", 500.0);
      properties.get("leftShoulderRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftShoulderRoll").put("damping", 0.1);
      properties.get("leftShoulderRoll").put("stiction", 0.0);
      properties.put("leftShoulderYaw", new HashMap<>());
      properties.put("leftShoulderYawLink", new HashMap<>());
      properties.get("leftShoulderYawLink").put("mass", 3.085);
      properties.get("leftShoulderYawLink").put("centerOfMass", new Vector3D(-0.004304, 0.209832, 0.007295));
      properties.get("leftShoulderYawLink")
                .put("inertia", new Matrix3D(0.0393552, -0.00782708, -7.53947E-4, -0.00782708, 0.00490577, 0.00272387, -7.53947E-4, 0.00272387, 0.0418795));
      properties.get("leftShoulderYaw").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("leftShoulderYaw").put("positionLowerLimit", -3.1);
      properties.get("leftShoulderYaw").put("positionUpperLimit", 2.18);
      properties.get("leftShoulderYaw").put("velocityLowerLimit", -1.5);
      properties.get("leftShoulderYaw").put("velocityUpperLimit", 1.5);
      properties.get("leftShoulderYaw").put("effortLowerLimit", -65.0);
      properties.get("leftShoulderYaw").put("effortUpperLimit", 65.0);
      properties.get("leftShoulderYaw").put("kpPositionLimit", 100.0);
      properties.get("leftShoulderYaw").put("kdPositionLimit", 20.0);
      properties.get("leftShoulderYaw").put("kpVelocityLimit", 500.0);
      properties.get("leftShoulderYaw").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("leftShoulderYaw").put("damping", 0.1);
      properties.get("leftShoulderYaw").put("stiction", 0.0);
      properties.put("leftElbowPitch", new HashMap<>());
      properties.put("leftElbowPitchLink", new HashMap<>());
      properties.get("leftElbowPitchLink").put("mass", 1.83);
      properties.get("leftElbowPitchLink").put("centerOfMass", new Vector3D(-0.020344, 0.014722, 0.0223));
      properties.get("leftElbowPitchLink")
                .put("inertia", new Matrix3D(0.00331452, 5.35099E-4, 7.28077E-4, 5.35099E-4, 0.00350567, -4.23865E-4, 7.28077E-4, -4.23865E-4, 0.00301128));
      properties.get("leftElbowPitch").put("offsetFromParentJoint", new Vector3D(0.0254, 0.32999999999999996, 0.0));
      properties.get("leftElbowPitch").put("positionLowerLimit", -2.174);
      properties.get("leftElbowPitch").put("positionUpperLimit", 0.12);
      properties.get("leftElbowPitch").put("velocityLowerLimit", -3.5);
      properties.get("leftElbowPitch").put("velocityUpperLimit", 3.5);
      properties.get("leftElbowPitch").put("effortLowerLimit", -65.0);
      properties.get("leftElbowPitch").put("effortUpperLimit", 65.0);
      properties.get("leftElbowPitch").put("kpPositionLimit", 100.0);
      properties.get("leftElbowPitch").put("kdPositionLimit", 20.0);
      properties.get("leftElbowPitch").put("kpVelocityLimit", 500.0);
      properties.get("leftElbowPitch").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("leftElbowPitch").put("damping", 0.1);
      properties.get("leftElbowPitch").put("stiction", 0.0);
      properties.put("leftForearmYaw", new HashMap<>());
      properties.put("leftForearmLink", new HashMap<>());
      properties.get("leftForearmLink").put("mass", 2.476);
      properties.get("leftForearmLink").put("centerOfMass", new Vector3D(0.015, 0.13, 0.019564));
      properties.get("leftForearmLink")
                .put("inertia", new Matrix3D(0.0117554, 0.00130085, -7.27141E-4, 0.00130085, 0.00507157, 0.00169542, -7.27141E-4, 0.00169542, 0.0113657));
      properties.get("leftForearmYaw").put("offsetFromParentJoint", new Vector3D(-0.0254, 0.0, 0.0));
      properties.get("leftForearmYaw").put("positionLowerLimit", -2.019);
      properties.get("leftForearmYaw").put("positionUpperLimit", 3.14);
      properties.get("leftForearmYaw").put("velocityLowerLimit", -0.8);
      properties.get("leftForearmYaw").put("velocityUpperLimit", 0.8);
      properties.get("leftForearmYaw").put("effortLowerLimit", -26.0);
      properties.get("leftForearmYaw").put("effortUpperLimit", 26.0);
      properties.get("leftForearmYaw").put("kpPositionLimit", 100.0);
      properties.get("leftForearmYaw").put("kdPositionLimit", 20.0);
      properties.get("leftForearmYaw").put("kpVelocityLimit", 500.0);
      properties.get("leftForearmYaw").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("leftForearmYaw").put("damping", 0.1);
      properties.get("leftForearmYaw").put("stiction", 0.0);
      properties.put("leftWristRoll", new HashMap<>());
      properties.put("leftWristRollLink", new HashMap<>());
      properties.get("leftWristRollLink").put("mass", 0.14);
      properties.get("leftWristRollLink").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("leftWristRollLink").put("inertia", new Matrix3D(3.0251E-5, 1.25E-7, 3.6E-8, 1.25E-7, 3.772E-5, 0.0, 3.6E-8, 0.0, 9.395E-6));
      properties.get("leftWristRoll").put("offsetFromParentJoint", new Vector3D(0.0, 0.2871, 0.0));
      properties.get("leftWristRoll").put("positionLowerLimit", -0.35);
      properties.get("leftWristRoll").put("positionUpperLimit", 0.35);
      properties.get("leftWristRoll").put("velocityLowerLimit", -1.0);
      properties.get("leftWristRoll").put("velocityUpperLimit", 1.0);
      properties.get("leftWristRoll").put("effortLowerLimit", -14.0);
      properties.get("leftWristRoll").put("effortUpperLimit", 14.0);
      properties.get("leftWristRoll").put("kpPositionLimit", 100.0);
      properties.get("leftWristRoll").put("kdPositionLimit", 20.0);
      properties.get("leftWristRoll").put("kpVelocityLimit", 500.0);
      properties.get("leftWristRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftWristRoll").put("damping", 0.1);
      properties.get("leftWristRoll").put("stiction", 0.0);
      properties.put("leftWristPitch", new HashMap<>());
      properties.put("leftPalm", new HashMap<>());
      properties.get("leftPalm").put("mass", 0.712);
      properties.get("leftPalm").put("centerOfMass", new Vector3D(0.002954, 0.052034, -2.36E-4));
      properties.get("leftPalm")
                .put("inertia", new Matrix3D(9.43493E-4, 3.4393E-5, 3.8828E-5, 3.4393E-5, 7.11024E-4, -2.3429E-5, 3.8828E-5, -2.3429E-5, 6.10199E-4));
      properties.get("leftWristPitch").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("leftWristPitch").put("positionLowerLimit", -0.6);
      properties.get("leftWristPitch").put("positionUpperLimit", 0.6);
      properties.get("leftWristPitch").put("velocityLowerLimit", -1.0);
      properties.get("leftWristPitch").put("velocityUpperLimit", 1.0);
      properties.get("leftWristPitch").put("effortLowerLimit", -14.0);
      properties.get("leftWristPitch").put("effortUpperLimit", 14.0);
      properties.get("leftWristPitch").put("kpPositionLimit", 100.0);
      properties.get("leftWristPitch").put("kdPositionLimit", 20.0);
      properties.get("leftWristPitch").put("kpVelocityLimit", 500.0);
      properties.get("leftWristPitch").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("leftWristPitch").put("damping", 0.1);
      properties.get("leftWristPitch").put("stiction", 0.0);
      properties.put("leftIndexFingerPitch1", new HashMap<>());
      properties.put("leftIndexFingerPitch1Link", new HashMap<>());
      properties.get("leftIndexFingerPitch1Link").put("mass", 0.02);
      properties.get("leftIndexFingerPitch1Link").put("centerOfMass", new Vector3D(0.0, 0.019696269397585765, 0.0034723150516228376));
      properties.get("leftIndexFingerPitch1Link")
                .put("inertia",
                     new Matrix3D(4.232E-6,
                                  5.769988217210815E-8,
                                  1.8295452913902818E-8,
                                  5.769988217210815E-8,
                                  1.2980849917342867E-6,
                                  -4.642891416145748E-7,
                                  1.8295452913902818E-8,
                                  -4.642891416145748E-7,
                                  3.8089150082657133E-6));
      properties.get("leftIndexFingerPitch1").put("offsetFromParentJoint", new Vector3D(0.0022000000000000006, 0.09760000000000002, 0.02350000000000002));
      properties.get("leftIndexFingerPitch1").put("positionLowerLimit", -1.57);
      properties.get("leftIndexFingerPitch1").put("positionUpperLimit", -0.0);
      properties.get("leftIndexFingerPitch1").put("velocityLowerLimit", -1.0);
      properties.get("leftIndexFingerPitch1").put("velocityUpperLimit", 1.0);
      properties.get("leftIndexFingerPitch1").put("effortLowerLimit", -10.0);
      properties.get("leftIndexFingerPitch1").put("effortUpperLimit", 10.0);
      properties.get("leftIndexFingerPitch1").put("kpPositionLimit", 100.0);
      properties.get("leftIndexFingerPitch1").put("kdPositionLimit", 20.0);
      properties.get("leftIndexFingerPitch1").put("kpVelocityLimit", 500.0);
      properties.get("leftIndexFingerPitch1").put("axis", new Vector3D(0.0, -0.17361607288187247, 0.9848134134124475));
      properties.get("leftIndexFingerPitch1").put("damping", 1.0);
      properties.get("leftIndexFingerPitch1").put("stiction", 0.0);
      properties.put("leftIndexFingerPitch2", new HashMap<>());
      properties.put("leftIndexFingerPitch2Link", new HashMap<>());
      properties.get("leftIndexFingerPitch2Link").put("mass", 0.018);
      properties.get("leftIndexFingerPitch2Link").put("centerOfMass", new Vector3D(0.0, 0.012802575108430745, 0.0022570047835548442));
      properties.get("leftIndexFingerPitch2Link")
                .put("inertia",
                     new Matrix3D(1.562E-6,
                                  1.4484824204393054E-8,
                                  1.575404290231016E-8,
                                  1.4484824204393054E-8,
                                  7.274652087801619E-7,
                                  -6.200070784124636E-8,
                                  1.575404290231016E-8,
                                  -6.200070784124639E-8,
                                  1.249534791219838E-6));
      properties.get("leftIndexFingerPitch2").put("offsetFromParentJoint", new Vector3D(0.0, 0.037519999999999935, 0.006614999999999957));
      properties.get("leftIndexFingerPitch2").put("positionLowerLimit", -1.658);
      properties.get("leftIndexFingerPitch2").put("positionUpperLimit", -0.0);
      properties.get("leftIndexFingerPitch2").put("velocityLowerLimit", -1.0);
      properties.get("leftIndexFingerPitch2").put("velocityUpperLimit", 1.0);
      properties.get("leftIndexFingerPitch2").put("effortLowerLimit", -10.0);
      properties.get("leftIndexFingerPitch2").put("effortUpperLimit", 10.0);
      properties.get("leftIndexFingerPitch2").put("kpPositionLimit", 100.0);
      properties.get("leftIndexFingerPitch2").put("kdPositionLimit", 20.0);
      properties.get("leftIndexFingerPitch2").put("kpVelocityLimit", 500.0);
      properties.get("leftIndexFingerPitch2").put("axis", new Vector3D(0.0, -0.17361607288187247, 0.9848134134124475));
      properties.get("leftIndexFingerPitch2").put("damping", 1.0);
      properties.get("leftIndexFingerPitch2").put("stiction", 0.0);
      properties.put("leftIndexFingerPitch3", new HashMap<>());
      properties.put("leftIndexFingerPitch3Link", new HashMap<>());
      properties.get("leftIndexFingerPitch3Link").put("mass", 0.01);
      properties.get("leftIndexFingerPitch3Link").put("centerOfMass", new Vector3D(0.0, 0.009848134698792883, 0.0017361575258114188));
      properties.get("leftIndexFingerPitch3Link")
                .put("inertia",
                     new Matrix3D(6.02E-7,
                                  2.3635523277102915E-8,
                                  4.1667780619474044E-9,
                                  2.3635523277102915E-8,
                                  2.1201844137782995E-7,
                                  -3.4121728199972275E-8,
                                  4.1667780619474044E-9,
                                  -3.412172819997229E-8,
                                  4.989815586221701E-7));
      properties.get("leftIndexFingerPitch3").put("offsetFromParentJoint", new Vector3D(0.0, 0.022550000000000042, 0.003976000000000067));
      properties.get("leftIndexFingerPitch3").put("positionLowerLimit", -1.92);
      properties.get("leftIndexFingerPitch3").put("positionUpperLimit", -0.0);
      properties.get("leftIndexFingerPitch3").put("velocityLowerLimit", -1.0);
      properties.get("leftIndexFingerPitch3").put("velocityUpperLimit", 1.0);
      properties.get("leftIndexFingerPitch3").put("effortLowerLimit", -10.0);
      properties.get("leftIndexFingerPitch3").put("effortUpperLimit", 10.0);
      properties.get("leftIndexFingerPitch3").put("kpPositionLimit", 100.0);
      properties.get("leftIndexFingerPitch3").put("kdPositionLimit", 20.0);
      properties.get("leftIndexFingerPitch3").put("kpVelocityLimit", 500.0);
      properties.get("leftIndexFingerPitch3").put("axis", new Vector3D(0.0, -0.17361607288187247, 0.9848134134124475));
      properties.get("leftIndexFingerPitch3").put("damping", 1.0);
      properties.get("leftIndexFingerPitch3").put("stiction", 0.0);
      properties.put("leftMiddleFingerPitch1", new HashMap<>());
      properties.put("leftMiddleFingerPitch1Link", new HashMap<>());
      properties.get("leftMiddleFingerPitch1Link").put("mass", 0.02);
      properties.get("leftMiddleFingerPitch1Link").put("centerOfMass", new Vector3D(0.0, 0.019850857332287003, -0.002437921896449382));
      properties.get("leftMiddleFingerPitch1Link")
                .put("inertia",
                     new Matrix3D(5.148E-6,
                                  4.3027657440849864E-8,
                                  5.798335550214148E-9,
                                  4.3027657440849864E-8,
                                  1.369619914645733E-6,
                                  3.91746684249759E-7,
                                  5.798335550214148E-9,
                                  3.9174668424975893E-7,
                                  4.544380085354267E-6));
      properties.get("leftMiddleFingerPitch1").put("offsetFromParentJoint", new Vector3D(0.0022000000000000006, 0.09699999999999998, -0.011899999999999966));
      properties.get("leftMiddleFingerPitch1").put("positionLowerLimit", -1.658);
      properties.get("leftMiddleFingerPitch1").put("positionUpperLimit", -0.0);
      properties.get("leftMiddleFingerPitch1").put("velocityLowerLimit", -1.0);
      properties.get("leftMiddleFingerPitch1").put("velocityUpperLimit", 1.0);
      properties.get("leftMiddleFingerPitch1").put("effortLowerLimit", -10.0);
      properties.get("leftMiddleFingerPitch1").put("effortUpperLimit", 10.0);
      properties.get("leftMiddleFingerPitch1").put("kpPositionLimit", 100.0);
      properties.get("leftMiddleFingerPitch1").put("kdPositionLimit", 20.0);
      properties.get("leftMiddleFingerPitch1").put("kpVelocityLimit", 500.0);
      properties.get("leftMiddleFingerPitch1").put("axis", new Vector3D(0.0, 0.12189598527100426, 0.9925428800685697));
      properties.get("leftMiddleFingerPitch1").put("damping", 1.0);
      properties.get("leftMiddleFingerPitch1").put("stiction", 0.0);
      properties.put("leftMiddleFingerPitch2", new HashMap<>());
      properties.put("leftMiddleFingerPitch2Link", new HashMap<>());
      properties.get("leftMiddleFingerPitch2Link").put("mass", 0.011);
      properties.get("leftMiddleFingerPitch2Link").put("centerOfMass", new Vector3D(0.0, 0.012903057265986551, -0.0015846492326920985));
      properties.get("leftMiddleFingerPitch2Link")
                .put("inertia",
                     new Matrix3D(1.97E-6,
                                  -3.240560042184074E-8,
                                  1.50624387567218E-8,
                                  -3.240560042184074E-8,
                                  5.606531868249063E-7,
                                  1.3217378737957825E-7,
                                  1.50624387567218E-8,
                                  1.321737873795783E-7,
                                  2.265346813175094E-6));
      properties.get("leftMiddleFingerPitch2").put("offsetFromParentJoint", new Vector3D(0.0, 0.037820000000000006, -0.004644000000000044));
      properties.get("leftMiddleFingerPitch2").put("positionLowerLimit", -1.92);
      properties.get("leftMiddleFingerPitch2").put("positionUpperLimit", -0.0);
      properties.get("leftMiddleFingerPitch2").put("velocityLowerLimit", -1.0);
      properties.get("leftMiddleFingerPitch2").put("velocityUpperLimit", 1.0);
      properties.get("leftMiddleFingerPitch2").put("effortLowerLimit", -10.0);
      properties.get("leftMiddleFingerPitch2").put("effortUpperLimit", 10.0);
      properties.get("leftMiddleFingerPitch2").put("kpPositionLimit", 100.0);
      properties.get("leftMiddleFingerPitch2").put("kdPositionLimit", 20.0);
      properties.get("leftMiddleFingerPitch2").put("kpVelocityLimit", 500.0);
      properties.get("leftMiddleFingerPitch2").put("axis", new Vector3D(0.0, 0.12189598527100426, 0.9925428800685697));
      properties.get("leftMiddleFingerPitch2").put("damping", 1.0);
      properties.get("leftMiddleFingerPitch2").put("stiction", 0.0);
      properties.put("leftMiddleFingerPitch3", new HashMap<>());
      properties.put("leftMiddleFingerPitch3Link", new HashMap<>());
      properties.get("leftMiddleFingerPitch3Link").put("mass", 0.006);
      properties.get("leftMiddleFingerPitch3Link").put("centerOfMass", new Vector3D(0.0, 0.009925428666143501, -0.001218960948224691));
      properties.get("leftMiddleFingerPitch3Link")
                .put("inertia",
                     new Matrix3D(3.96E-7,
                                  9.05478189435162E-9,
                                  -1.0452198678787192E-10,
                                  9.05478189435162E-9,
                                  1.659963752647085E-7,
                                  3.2825678158083905E-8,
                                  -1.0452198678787192E-10,
                                  3.282567815808391E-8,
                                  2.9700362473529145E-7));
      properties.get("leftMiddleFingerPitch3").put("offsetFromParentJoint", new Vector3D(0.0, 0.022730000000000167, -0.002791999999999949));
      properties.get("leftMiddleFingerPitch3").put("positionLowerLimit", -1.57);
      properties.get("leftMiddleFingerPitch3").put("positionUpperLimit", -0.0);
      properties.get("leftMiddleFingerPitch3").put("velocityLowerLimit", -1.0);
      properties.get("leftMiddleFingerPitch3").put("velocityUpperLimit", 1.0);
      properties.get("leftMiddleFingerPitch3").put("effortLowerLimit", -10.0);
      properties.get("leftMiddleFingerPitch3").put("effortUpperLimit", 10.0);
      properties.get("leftMiddleFingerPitch3").put("kpPositionLimit", 100.0);
      properties.get("leftMiddleFingerPitch3").put("kdPositionLimit", 20.0);
      properties.get("leftMiddleFingerPitch3").put("kpVelocityLimit", 500.0);
      properties.get("leftMiddleFingerPitch3").put("axis", new Vector3D(0.0, 0.12189598527100426, 0.9925428800685697));
      properties.get("leftMiddleFingerPitch3").put("damping", 1.0);
      properties.get("leftMiddleFingerPitch3").put("stiction", 0.0);
      properties.put("leftPinkyPitch1", new HashMap<>());
      properties.put("leftPinkyPitch1Link", new HashMap<>());
      properties.get("leftPinkyPitch1Link").put("mass", 0.02);
      properties.get("leftPinkyPitch1Link").put("centerOfMass", new Vector3D(0.0, 0.019850857332287003, -0.002437921896449382));
      properties.get("leftPinkyPitch1Link")
                .put("inertia",
                     new Matrix3D(5.148E-6,
                                  4.3027657440849864E-8,
                                  5.798335550214148E-9,
                                  4.3027657440849864E-8,
                                  1.369619914645733E-6,
                                  3.91746684249759E-7,
                                  5.798335550214148E-9,
                                  3.9174668424975893E-7,
                                  4.544380085354267E-6));
      properties.get("leftPinkyPitch1").put("offsetFromParentJoint", new Vector3D(0.0022000000000000006, 0.08379999999999999, -0.04099999999999998));
      properties.get("leftPinkyPitch1").put("positionLowerLimit", -1.57);
      properties.get("leftPinkyPitch1").put("positionUpperLimit", -0.0);
      properties.get("leftPinkyPitch1").put("velocityLowerLimit", -1.0);
      properties.get("leftPinkyPitch1").put("velocityUpperLimit", 1.0);
      properties.get("leftPinkyPitch1").put("effortLowerLimit", -10.0);
      properties.get("leftPinkyPitch1").put("effortUpperLimit", 10.0);
      properties.get("leftPinkyPitch1").put("kpPositionLimit", 100.0);
      properties.get("leftPinkyPitch1").put("kdPositionLimit", 20.0);
      properties.get("leftPinkyPitch1").put("kpVelocityLimit", 500.0);
      properties.get("leftPinkyPitch1").put("axis", new Vector3D(0.0, 0.12189598527100426, 0.9925428800685697));
      properties.get("leftPinkyPitch1").put("damping", 1.0);
      properties.get("leftPinkyPitch1").put("stiction", 0.0);
      properties.put("leftPinkyPitch2", new HashMap<>());
      properties.put("leftPinkyPitch2Link", new HashMap<>());
      properties.get("leftPinkyPitch2Link").put("mass", 0.011);
      properties.get("leftPinkyPitch2Link").put("centerOfMass", new Vector3D(0.0, 0.012903057265986551, -0.0015846492326920985));
      properties.get("leftPinkyPitch2Link")
                .put("inertia",
                     new Matrix3D(1.97E-6,
                                  -3.240560042184074E-8,
                                  1.50624387567218E-8,
                                  -3.240560042184074E-8,
                                  5.606531868249063E-7,
                                  1.3217378737957825E-7,
                                  1.50624387567218E-8,
                                  1.321737873795783E-7,
                                  2.265346813175094E-6));
      properties.get("leftPinkyPitch2").put("offsetFromParentJoint", new Vector3D(0.0, 0.037816000000000016, -0.0046439999999999025));
      properties.get("leftPinkyPitch2").put("positionLowerLimit", -1.658);
      properties.get("leftPinkyPitch2").put("positionUpperLimit", -0.0);
      properties.get("leftPinkyPitch2").put("velocityLowerLimit", -1.0);
      properties.get("leftPinkyPitch2").put("velocityUpperLimit", 1.0);
      properties.get("leftPinkyPitch2").put("effortLowerLimit", -10.0);
      properties.get("leftPinkyPitch2").put("effortUpperLimit", 10.0);
      properties.get("leftPinkyPitch2").put("kpPositionLimit", 100.0);
      properties.get("leftPinkyPitch2").put("kdPositionLimit", 20.0);
      properties.get("leftPinkyPitch2").put("kpVelocityLimit", 500.0);
      properties.get("leftPinkyPitch2").put("axis", new Vector3D(0.0, 0.12189598527100426, 0.9925428800685697));
      properties.get("leftPinkyPitch2").put("damping", 1.0);
      properties.get("leftPinkyPitch2").put("stiction", 0.0);
      properties.put("leftPinkyPitch3", new HashMap<>());
      properties.put("leftPinkyPitch3Link", new HashMap<>());
      properties.get("leftPinkyPitch3Link").put("mass", 0.006);
      properties.get("leftPinkyPitch3Link").put("centerOfMass", new Vector3D(0.0, 0.009925428666143501, -0.001218960948224691));
      properties.get("leftPinkyPitch3Link")
                .put("inertia",
                     new Matrix3D(3.96E-7,
                                  9.05478189435162E-9,
                                  -1.0452198678787192E-10,
                                  9.05478189435162E-9,
                                  1.659963752647085E-7,
                                  3.2825678158083905E-8,
                                  -1.0452198678787192E-10,
                                  3.282567815808391E-8,
                                  2.9700362473529145E-7));
      properties.get("leftPinkyPitch3").put("offsetFromParentJoint", new Vector3D(0.0, 0.022734000000000056, -0.0027920000000000765));
      properties.get("leftPinkyPitch3").put("positionLowerLimit", -1.92);
      properties.get("leftPinkyPitch3").put("positionUpperLimit", -0.0);
      properties.get("leftPinkyPitch3").put("velocityLowerLimit", -1.0);
      properties.get("leftPinkyPitch3").put("velocityUpperLimit", 1.0);
      properties.get("leftPinkyPitch3").put("effortLowerLimit", -10.0);
      properties.get("leftPinkyPitch3").put("effortUpperLimit", 10.0);
      properties.get("leftPinkyPitch3").put("kpPositionLimit", 100.0);
      properties.get("leftPinkyPitch3").put("kdPositionLimit", 20.0);
      properties.get("leftPinkyPitch3").put("kpVelocityLimit", 500.0);
      properties.get("leftPinkyPitch3").put("axis", new Vector3D(0.0, 0.12189598527100426, 0.9925428800685697));
      properties.get("leftPinkyPitch3").put("damping", 1.0);
      properties.get("leftPinkyPitch3").put("stiction", 0.0);
      properties.put("leftThumbRoll", new HashMap<>());
      properties.put("leftThumbRollLink", new HashMap<>());
      properties.get("leftThumbRollLink").put("mass", 0.017);
      properties.get("leftThumbRollLink").put("centerOfMass", new Vector3D(0.0, 0.003420522332544199, 0.009396809403865038));
      properties.get("leftThumbRollLink")
                .put("inertia",
                     new Matrix3D(2.77788E-6,
                                  3.800365322933979E-8,
                                  8.7821261445544E-8,
                                  3.800365322933979E-8,
                                  3.2663699259248467E-6,
                                  -7.322727571414368E-7,
                                  8.7821261445544E-8,
                                  -7.322727571414366E-7,
                                  1.6352200740751536E-6));
      properties.get("leftThumbRoll").put("offsetFromParentJoint", new Vector3D(0.0049, 0.03510000000000002, 0.022800000000000042));
      properties.get("leftThumbRoll").put("positionLowerLimit", 0.0);
      properties.get("leftThumbRoll").put("positionUpperLimit", 2.356);
      properties.get("leftThumbRoll").put("velocityLowerLimit", -1.0);
      properties.get("leftThumbRoll").put("velocityUpperLimit", 1.0);
      properties.get("leftThumbRoll").put("effortLowerLimit", -10.0);
      properties.get("leftThumbRoll").put("effortUpperLimit", 10.0);
      properties.get("leftThumbRoll").put("kpPositionLimit", 100.0);
      properties.get("leftThumbRoll").put("kdPositionLimit", 20.0);
      properties.get("leftThumbRoll").put("kpVelocityLimit", 500.0);
      properties.get("leftThumbRoll").put("axis", new Vector3D(0.0, 0.939681022333869, -0.34205200812972125));
      properties.get("leftThumbRoll").put("damping", 1.0);
      properties.get("leftThumbRoll").put("stiction", 0.0);
      properties.put("leftThumbPitch1", new HashMap<>());
      properties.put("leftThumbPitch1Link", new HashMap<>());
      properties.get("leftThumbPitch1Link").put("mass", 0.02);
      properties.get("leftThumbPitch1Link").put("centerOfMass", new Vector3D(0.0, 0.005, 0.02));
      properties.get("leftThumbPitch1Link").put("inertia", new Matrix3D(4.239E-6, 0.0, 0.0, 0.0, 4.582E-6, -0.0, 0.0, -0.0, 1.47E-6));
      properties.get("leftThumbPitch1").put("offsetFromParentJoint", new Vector3D(0.0, 0.007832999999999965, 0.021518999999999993));
      properties.get("leftThumbPitch1").put("positionLowerLimit", -1.658);
      properties.get("leftThumbPitch1").put("positionUpperLimit", -0.0);
      properties.get("leftThumbPitch1").put("velocityLowerLimit", -1.0);
      properties.get("leftThumbPitch1").put("velocityUpperLimit", 1.0);
      properties.get("leftThumbPitch1").put("effortLowerLimit", -10.0);
      properties.get("leftThumbPitch1").put("effortUpperLimit", 10.0);
      properties.get("leftThumbPitch1").put("kpPositionLimit", 100.0);
      properties.get("leftThumbPitch1").put("kdPositionLimit", 20.0);
      properties.get("leftThumbPitch1").put("kpVelocityLimit", 500.0);
      properties.get("leftThumbPitch1").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftThumbPitch1").put("damping", 1.0);
      properties.get("leftThumbPitch1").put("stiction", 0.0);
      properties.put("leftThumbPitch2", new HashMap<>());
      properties.put("leftThumbPitch2Link", new HashMap<>());
      properties.get("leftThumbPitch2Link").put("mass", 0.013);
      properties.get("leftThumbPitch2Link").put("centerOfMass", new Vector3D(0.0, 0.004, 0.017));
      properties.get("leftThumbPitch2Link").put("inertia", new Matrix3D(1.266E-6, 0.0, 0.0, 0.0, 1.503E-6, 0.0, 0.0, 0.0, 6.99E-7));
      properties.get("leftThumbPitch2").put("offsetFromParentJoint", new Vector3D(0.0, 0.00660000000000005, 0.03750000000000003));
      properties.get("leftThumbPitch2").put("positionLowerLimit", -1.92);
      properties.get("leftThumbPitch2").put("positionUpperLimit", -0.0);
      properties.get("leftThumbPitch2").put("velocityLowerLimit", -1.0);
      properties.get("leftThumbPitch2").put("velocityUpperLimit", 1.0);
      properties.get("leftThumbPitch2").put("effortLowerLimit", -10.0);
      properties.get("leftThumbPitch2").put("effortUpperLimit", 10.0);
      properties.get("leftThumbPitch2").put("kpPositionLimit", 100.0);
      properties.get("leftThumbPitch2").put("kdPositionLimit", 20.0);
      properties.get("leftThumbPitch2").put("kpVelocityLimit", 500.0);
      properties.get("leftThumbPitch2").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftThumbPitch2").put("damping", 1.0);
      properties.get("leftThumbPitch2").put("stiction", 0.0);
      properties.put("leftThumbPitch3", new HashMap<>());
      properties.put("leftThumbPitch3Link", new HashMap<>());
      properties.get("leftThumbPitch3Link").put("mass", 0.006);
      properties.get("leftThumbPitch3Link").put("centerOfMass", new Vector3D(0.0, 0.004, 0.01));
      properties.get("leftThumbPitch3Link").put("inertia", new Matrix3D(3.22E-7, 0.0, 0.0, 0.0, 4.06E-7, 0.0, 0.0, 0.0, 2.1E-7));
      properties.get("leftThumbPitch3").put("offsetFromParentJoint", new Vector3D(0.0, 0.004899999999999904, 0.02749999999999997));
      properties.get("leftThumbPitch3").put("positionLowerLimit", -1.57);
      properties.get("leftThumbPitch3").put("positionUpperLimit", -0.0);
      properties.get("leftThumbPitch3").put("velocityLowerLimit", -1.0);
      properties.get("leftThumbPitch3").put("velocityUpperLimit", 1.0);
      properties.get("leftThumbPitch3").put("effortLowerLimit", -10.0);
      properties.get("leftThumbPitch3").put("effortUpperLimit", 10.0);
      properties.get("leftThumbPitch3").put("kpPositionLimit", 100.0);
      properties.get("leftThumbPitch3").put("kdPositionLimit", 20.0);
      properties.get("leftThumbPitch3").put("kpVelocityLimit", 500.0);
      properties.get("leftThumbPitch3").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("leftThumbPitch3").put("damping", 1.0);
      properties.get("leftThumbPitch3").put("stiction", 0.0);
      properties.put("lowerNeckPitch", new HashMap<>());
      properties.put("lowerNeckPitchLink", new HashMap<>());
      properties.get("lowerNeckPitchLink").put("mass", 1.05);
      properties.get("lowerNeckPitchLink").put("centerOfMass", new Vector3D(-0.02, 0.0, 0.04));
      properties.get("lowerNeckPitchLink")
                .put("inertia", new Matrix3D(0.00147072, -1.26021E-4, 6.32521E-4, -1.26021E-4, 0.00185192, 2.43012E-4, 6.32521E-4, 2.43012E-4, 8.32117E-4));
      properties.get("lowerNeckPitch").put("offsetFromParentJoint", new Vector3D(0.020351799999999996, 0.0, 0.33845000000000003));
      properties.get("lowerNeckPitch").put("positionLowerLimit", 0.0);
      properties.get("lowerNeckPitch").put("positionUpperLimit", 1.162);
      properties.get("lowerNeckPitch").put("velocityLowerLimit", -5.0);
      properties.get("lowerNeckPitch").put("velocityUpperLimit", 5.0);
      properties.get("lowerNeckPitch").put("effortLowerLimit", -26.0);
      properties.get("lowerNeckPitch").put("effortUpperLimit", 26.0);
      properties.get("lowerNeckPitch").put("kpPositionLimit", 100.0);
      properties.get("lowerNeckPitch").put("kdPositionLimit", 20.0);
      properties.get("lowerNeckPitch").put("kpVelocityLimit", 500.0);
      properties.get("lowerNeckPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("lowerNeckPitch").put("damping", 0.1);
      properties.get("lowerNeckPitch").put("stiction", 0.0);
      properties.put("neckYaw", new HashMap<>());
      properties.put("neckYawLink", new HashMap<>());
      properties.get("neckYawLink").put("mass", 1.4);
      properties.get("neckYawLink").put("centerOfMass", new Vector3D(-0.03, -0.01, 0.15));
      properties.get("neckYawLink")
                .put("inertia", new Matrix3D(0.0019977, -1.80062E-4, 7.23677E-4, -1.80062E-4, 0.00291993, 2.46467E-4, 7.23677E-4, 2.46467E-4, 0.00211975));
      properties.get("neckYaw").put("offsetFromParentJoint", new Vector3D(-0.051924, 0.0, 0.0));
      properties.get("neckYaw").put("positionLowerLimit", -1.0472);
      properties.get("neckYaw").put("positionUpperLimit", 1.0472);
      properties.get("neckYaw").put("velocityLowerLimit", -5.0);
      properties.get("neckYaw").put("velocityUpperLimit", 5.0);
      properties.get("neckYaw").put("effortLowerLimit", -26.0);
      properties.get("neckYaw").put("effortUpperLimit", 26.0);
      properties.get("neckYaw").put("kpPositionLimit", 100.0);
      properties.get("neckYaw").put("kdPositionLimit", 20.0);
      properties.get("neckYaw").put("kpVelocityLimit", 500.0);
      properties.get("neckYaw").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("neckYaw").put("damping", 0.1);
      properties.get("neckYaw").put("stiction", 0.0);
      properties.put("upperNeckPitch", new HashMap<>());
      properties.put("upperNeckPitchLink", new HashMap<>());
      properties.get("upperNeckPitchLink").put("mass", 5.30991);
      properties.get("upperNeckPitchLink").put("centerOfMass", new Vector3D(0.102036, 0.00422754, 0.0409152));
      properties.get("upperNeckPitchLink")
                .put("inertia", new Matrix3D(0.0446288, -4.43568E-4, -0.0021252, -4.43568E-4, 0.0461084, -6.44772E-4, -0.0021252, -6.44772E-4, 0.0402031));
      properties.get("upperNeckPitch").put("offsetFromParentJoint", new Vector3D(-0.06, 0.0, 0.19599699999999998));
      properties.get("upperNeckPitch").put("positionLowerLimit", -0.872);
      properties.get("upperNeckPitch").put("positionUpperLimit", 0.0);
      properties.get("upperNeckPitch").put("velocityLowerLimit", -5.0);
      properties.get("upperNeckPitch").put("velocityUpperLimit", 5.0);
      properties.get("upperNeckPitch").put("effortLowerLimit", -26.0);
      properties.get("upperNeckPitch").put("effortUpperLimit", 26.0);
      properties.get("upperNeckPitch").put("kpPositionLimit", 100.0);
      properties.get("upperNeckPitch").put("kdPositionLimit", 20.0);
      properties.get("upperNeckPitch").put("kpVelocityLimit", 500.0);
      properties.get("upperNeckPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("upperNeckPitch").put("damping", 0.1);
      properties.get("upperNeckPitch").put("stiction", 0.0);
      properties.put("hokuyo_joint", new HashMap<>());
      properties.put("hokuyo_link", new HashMap<>());
      properties.get("hokuyo_link").put("mass", 0.057664);
      properties.get("hokuyo_link").put("centerOfMass", new Vector3D(0.03270279235379761, -4.084110874927957E-4, -9.106101256784079E-5));
      properties.get("hokuyo_link")
                .put("inertia",
                     new Matrix3D(3.541192586390836E-5,
                                  -5.075197116557559E-8,
                                  -1.004534784646333E-5,
                                  -5.075197116557559E-8,
                                  4.343700005209264E-5,
                                  -3.2175569823047584E-9,
                                  -1.0045347846463328E-5,
                                  -3.217556982304757E-9,
                                  4.545707408399899E-5));
      properties.get("hokuyo_joint").put("offsetFromParentJoint", new Vector3D(0.1278812, 2.33516E-7, -0.006071999999999966));
      properties.get("hokuyo_joint").put("positionLowerLimit", -1.0E16);
      properties.get("hokuyo_joint").put("positionUpperLimit", 1.0E16);
      properties.get("hokuyo_joint").put("velocityLowerLimit", -Infinity);
      properties.get("hokuyo_joint").put("velocityUpperLimit", Infinity);
      properties.get("hokuyo_joint").put("effortLowerLimit", -Infinity);
      properties.get("hokuyo_joint").put("effortUpperLimit", Infinity);
      properties.get("hokuyo_joint").put("kpPositionLimit", 100.0);
      properties.get("hokuyo_joint").put("kdPositionLimit", 20.0);
      properties.get("hokuyo_joint").put("kpVelocityLimit", 0.0);
      properties.get("hokuyo_joint").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("hokuyo_joint").put("damping", 0.01);
      properties.get("hokuyo_joint").put("stiction", 0.0);
      properties.put("rightShoulderPitch", new HashMap<>());
      properties.put("rightShoulderPitchLink", new HashMap<>());
      properties.get("rightShoulderPitchLink").put("mass", 2.65);
      properties.get("rightShoulderPitchLink").put("centerOfMass", new Vector3D(0.012, -0.251, 0.0));
      properties.get("rightShoulderPitchLink").put("inertia", new Matrix3D(0.0137182, 0.0, 0.0, 0.0, 0.0105028, 0.0, 0.0, 0.0, 0.0148064));
      properties.get("rightShoulderPitch").put("offsetFromParentJoint", new Vector3D(-0.0316, 0.0, 0.2984));
      properties.get("rightShoulderPitch").put("positionLowerLimit", -2.85);
      properties.get("rightShoulderPitch").put("positionUpperLimit", 2.0);
      properties.get("rightShoulderPitch").put("velocityLowerLimit", -3.0);
      properties.get("rightShoulderPitch").put("velocityUpperLimit", 3.0);
      properties.get("rightShoulderPitch").put("effortLowerLimit", -190.0);
      properties.get("rightShoulderPitch").put("effortUpperLimit", 190.0);
      properties.get("rightShoulderPitch").put("kpPositionLimit", 100.0);
      properties.get("rightShoulderPitch").put("kdPositionLimit", 20.0);
      properties.get("rightShoulderPitch").put("kpVelocityLimit", 500.0);
      properties.get("rightShoulderPitch").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("rightShoulderPitch").put("damping", 0.1);
      properties.get("rightShoulderPitch").put("stiction", 0.0);
      properties.put("rightShoulderRoll", new HashMap<>());
      properties.put("rightShoulderRollLink", new HashMap<>());
      properties.get("rightShoulderRollLink").put("mass", 3.97);
      properties.get("rightShoulderRollLink").put("centerOfMass", new Vector3D(0.008513, -0.02068, -0.001088));
      properties.get("rightShoulderRollLink")
                .put("inertia", new Matrix3D(0.0145988, -6.6764E-4, 3.629E-5, -6.6764E-4, 0.00645214, 8.283E-5, 3.629E-5, 8.283E-5, 0.0168483));
      properties.get("rightShoulderRoll").put("offsetFromParentJoint", new Vector3D(0.0, -0.2499, 0.0));
      properties.get("rightShoulderRoll").put("positionLowerLimit", -1.266);
      properties.get("rightShoulderRoll").put("positionUpperLimit", 1.519);
      properties.get("rightShoulderRoll").put("velocityLowerLimit", -3.5);
      properties.get("rightShoulderRoll").put("velocityUpperLimit", 3.5);
      properties.get("rightShoulderRoll").put("effortLowerLimit", -350.0);
      properties.get("rightShoulderRoll").put("effortUpperLimit", 350.0);
      properties.get("rightShoulderRoll").put("kpPositionLimit", 100.0);
      properties.get("rightShoulderRoll").put("kdPositionLimit", 20.0);
      properties.get("rightShoulderRoll").put("kpVelocityLimit", 500.0);
      properties.get("rightShoulderRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightShoulderRoll").put("damping", 0.1);
      properties.get("rightShoulderRoll").put("stiction", 0.0);
      properties.put("rightShoulderYaw", new HashMap<>());
      properties.put("rightShoulderYawLink", new HashMap<>());
      properties.get("rightShoulderYawLink").put("mass", 3.085);
      properties.get("rightShoulderYawLink").put("centerOfMass", new Vector3D(-0.004304, -0.209832, -0.007295));
      properties.get("rightShoulderYawLink")
                .put("inertia", new Matrix3D(0.0393552, 0.00782708, 7.53947E-4, 0.00782708, 0.00490577, -0.00272387, 7.53947E-4, -0.00272387, 0.0418795));
      properties.get("rightShoulderYaw").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("rightShoulderYaw").put("positionLowerLimit", -3.1);
      properties.get("rightShoulderYaw").put("positionUpperLimit", 2.18);
      properties.get("rightShoulderYaw").put("velocityLowerLimit", -1.5);
      properties.get("rightShoulderYaw").put("velocityUpperLimit", 1.5);
      properties.get("rightShoulderYaw").put("effortLowerLimit", -65.0);
      properties.get("rightShoulderYaw").put("effortUpperLimit", 65.0);
      properties.get("rightShoulderYaw").put("kpPositionLimit", 100.0);
      properties.get("rightShoulderYaw").put("kdPositionLimit", 20.0);
      properties.get("rightShoulderYaw").put("kpVelocityLimit", 500.0);
      properties.get("rightShoulderYaw").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("rightShoulderYaw").put("damping", 0.1);
      properties.get("rightShoulderYaw").put("stiction", 0.0);
      properties.put("rightElbowPitch", new HashMap<>());
      properties.put("rightElbowPitchLink", new HashMap<>());
      properties.get("rightElbowPitchLink").put("mass", 1.83);
      properties.get("rightElbowPitchLink").put("centerOfMass", new Vector3D(-0.020344, -0.014722, -0.0223));
      properties.get("rightElbowPitchLink")
                .put("inertia", new Matrix3D(0.00331452, -5.35099E-4, -7.28077E-4, -5.35099E-4, 0.00350567, -4.23865E-4, -7.28077E-4, -4.23865E-4, 0.00301128));
      properties.get("rightElbowPitch").put("offsetFromParentJoint", new Vector3D(0.0254, -0.32999999999999996, 0.0));
      properties.get("rightElbowPitch").put("positionLowerLimit", -0.12);
      properties.get("rightElbowPitch").put("positionUpperLimit", 2.174);
      properties.get("rightElbowPitch").put("velocityLowerLimit", -3.5);
      properties.get("rightElbowPitch").put("velocityUpperLimit", 3.5);
      properties.get("rightElbowPitch").put("effortLowerLimit", -65.0);
      properties.get("rightElbowPitch").put("effortUpperLimit", 65.0);
      properties.get("rightElbowPitch").put("kpPositionLimit", 100.0);
      properties.get("rightElbowPitch").put("kdPositionLimit", 20.0);
      properties.get("rightElbowPitch").put("kpVelocityLimit", 500.0);
      properties.get("rightElbowPitch").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("rightElbowPitch").put("damping", 0.1);
      properties.get("rightElbowPitch").put("stiction", 0.0);
      properties.put("rightForearmYaw", new HashMap<>());
      properties.put("rightForearmLink", new HashMap<>());
      properties.get("rightForearmLink").put("mass", 2.476);
      properties.get("rightForearmLink").put("centerOfMass", new Vector3D(0.015, -0.13, 0.019564));
      properties.get("rightForearmLink")
                .put("inertia", new Matrix3D(0.0117554, -0.00130085, -7.27141E-4, -0.00130085, 0.00507157, -0.00169542, -7.27141E-4, -0.00169542, 0.0113657));
      properties.get("rightForearmYaw").put("offsetFromParentJoint", new Vector3D(-0.0254, 0.0, 0.0));
      properties.get("rightForearmYaw").put("positionLowerLimit", -2.019);
      properties.get("rightForearmYaw").put("positionUpperLimit", 3.14);
      properties.get("rightForearmYaw").put("velocityLowerLimit", -0.8);
      properties.get("rightForearmYaw").put("velocityUpperLimit", 0.8);
      properties.get("rightForearmYaw").put("effortLowerLimit", -26.0);
      properties.get("rightForearmYaw").put("effortUpperLimit", 26.0);
      properties.get("rightForearmYaw").put("kpPositionLimit", 100.0);
      properties.get("rightForearmYaw").put("kdPositionLimit", 20.0);
      properties.get("rightForearmYaw").put("kpVelocityLimit", 500.0);
      properties.get("rightForearmYaw").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("rightForearmYaw").put("damping", 0.1);
      properties.get("rightForearmYaw").put("stiction", 0.0);
      properties.put("rightWristRoll", new HashMap<>());
      properties.put("rightWristRollLink", new HashMap<>());
      properties.get("rightWristRollLink").put("mass", 0.14);
      properties.get("rightWristRollLink").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("rightWristRollLink").put("inertia", new Matrix3D(3.0251E-5, -1.25E-7, -3.6E-8, -1.25E-7, 3.772E-5, 0.0, -3.6E-8, 0.0, 9.395E-6));
      properties.get("rightWristRoll").put("offsetFromParentJoint", new Vector3D(0.0, -0.2871, 0.0));
      properties.get("rightWristRoll").put("positionLowerLimit", -0.35);
      properties.get("rightWristRoll").put("positionUpperLimit", 0.35);
      properties.get("rightWristRoll").put("velocityLowerLimit", -1.0);
      properties.get("rightWristRoll").put("velocityUpperLimit", 1.0);
      properties.get("rightWristRoll").put("effortLowerLimit", -14.0);
      properties.get("rightWristRoll").put("effortUpperLimit", 14.0);
      properties.get("rightWristRoll").put("kpPositionLimit", 100.0);
      properties.get("rightWristRoll").put("kdPositionLimit", 20.0);
      properties.get("rightWristRoll").put("kpVelocityLimit", 500.0);
      properties.get("rightWristRoll").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightWristRoll").put("damping", 0.1);
      properties.get("rightWristRoll").put("stiction", 0.0);
      properties.put("rightWristPitch", new HashMap<>());
      properties.put("rightPalm", new HashMap<>());
      properties.get("rightPalm").put("mass", 0.712);
      properties.get("rightPalm").put("centerOfMass", new Vector3D(0.002954, -0.052034, -2.36E-4));
      properties.get("rightPalm")
                .put("inertia", new Matrix3D(9.43493E-4, 3.4393E-5, -3.8828E-5, 3.4393E-5, 7.11024E-4, 2.3429E-5, -3.8828E-5, 2.3429E-5, 6.10199E-4));
      properties.get("rightWristPitch").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("rightWristPitch").put("positionLowerLimit", -0.6);
      properties.get("rightWristPitch").put("positionUpperLimit", 0.6);
      properties.get("rightWristPitch").put("velocityLowerLimit", -1.0);
      properties.get("rightWristPitch").put("velocityUpperLimit", 1.0);
      properties.get("rightWristPitch").put("effortLowerLimit", -14.0);
      properties.get("rightWristPitch").put("effortUpperLimit", 14.0);
      properties.get("rightWristPitch").put("kpPositionLimit", 100.0);
      properties.get("rightWristPitch").put("kdPositionLimit", 20.0);
      properties.get("rightWristPitch").put("kpVelocityLimit", 500.0);
      properties.get("rightWristPitch").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("rightWristPitch").put("damping", 0.1);
      properties.get("rightWristPitch").put("stiction", 0.0);
      properties.put("rightIndexFingerPitch1", new HashMap<>());
      properties.put("rightIndexFingerPitch1Link", new HashMap<>());
      properties.get("rightIndexFingerPitch1Link").put("mass", 0.02);
      properties.get("rightIndexFingerPitch1Link").put("centerOfMass", new Vector3D(0.0, -0.019696269397585765, 0.0034723150516228376));
      properties.get("rightIndexFingerPitch1Link")
                .put("inertia",
                     new Matrix3D(4.232E-6,
                                  -6.047773421340643E-8,
                                  2.5384373958342057E-9,
                                  -6.047773421340643E-8,
                                  1.2932975760459825E-6,
                                  4.511331296418157E-7,
                                  2.5384373958342057E-9,
                                  4.511331296418157E-7,
                                  3.813702423954017E-6));
      properties.get("rightIndexFingerPitch1").put("offsetFromParentJoint", new Vector3D(0.0022000000000000006, -0.09760000000000002, 0.02350000000000002));
      properties.get("rightIndexFingerPitch1").put("positionLowerLimit", 0.0);
      properties.get("rightIndexFingerPitch1").put("positionUpperLimit", 1.57);
      properties.get("rightIndexFingerPitch1").put("velocityLowerLimit", -1.0);
      properties.get("rightIndexFingerPitch1").put("velocityUpperLimit", 1.0);
      properties.get("rightIndexFingerPitch1").put("effortLowerLimit", -10.0);
      properties.get("rightIndexFingerPitch1").put("effortUpperLimit", 10.0);
      properties.get("rightIndexFingerPitch1").put("kpPositionLimit", 100.0);
      properties.get("rightIndexFingerPitch1").put("kdPositionLimit", 20.0);
      properties.get("rightIndexFingerPitch1").put("kpVelocityLimit", 500.0);
      properties.get("rightIndexFingerPitch1").put("axis", new Vector3D(0.0, 0.17361607288187247, 0.9848134134124475));
      properties.get("rightIndexFingerPitch1").put("damping", 1.0);
      properties.get("rightIndexFingerPitch1").put("stiction", 0.0);
      properties.put("rightIndexFingerPitch2", new HashMap<>());
      properties.put("rightIndexFingerPitch2Link", new HashMap<>());
      properties.get("rightIndexFingerPitch2Link").put("mass", 0.018);
      properties.get("rightIndexFingerPitch2Link").put("centerOfMass", new Vector3D(0.0, -0.012802575108430745, 0.0022570047835548442));
      properties.get("rightIndexFingerPitch2Link")
                .put("inertia",
                     new Matrix3D(1.562E-6,
                                  -1.8998833771502745E-8,
                                  -9.851107314551334E-9,
                                  -1.8998833771502745E-8,
                                  7.486666211140791E-7,
                                  1.2026304657775112E-7,
                                  -9.851107314551334E-9,
                                  1.2026304657775112E-7,
                                  1.2283333788859208E-6));
      properties.get("rightIndexFingerPitch2").put("offsetFromParentJoint", new Vector3D(0.0, -0.037519999999999935, 0.006614999999999957));
      properties.get("rightIndexFingerPitch2").put("positionLowerLimit", 0.0);
      properties.get("rightIndexFingerPitch2").put("positionUpperLimit", 1.658);
      properties.get("rightIndexFingerPitch2").put("velocityLowerLimit", -1.0);
      properties.get("rightIndexFingerPitch2").put("velocityUpperLimit", 1.0);
      properties.get("rightIndexFingerPitch2").put("effortLowerLimit", -10.0);
      properties.get("rightIndexFingerPitch2").put("effortUpperLimit", 10.0);
      properties.get("rightIndexFingerPitch2").put("kpPositionLimit", 100.0);
      properties.get("rightIndexFingerPitch2").put("kdPositionLimit", 20.0);
      properties.get("rightIndexFingerPitch2").put("kpVelocityLimit", 500.0);
      properties.get("rightIndexFingerPitch2").put("axis", new Vector3D(0.0, 0.17361607288187247, 0.9848134134124475));
      properties.get("rightIndexFingerPitch2").put("damping", 1.0);
      properties.get("rightIndexFingerPitch2").put("stiction", 0.0);
      properties.put("rightIndexFingerPitch3", new HashMap<>());
      properties.put("rightIndexFingerPitch3Link", new HashMap<>());
      properties.get("rightIndexFingerPitch3Link").put("mass", 0.01);
      properties.get("rightIndexFingerPitch3Link").put("centerOfMass", new Vector3D(0.0, -0.009848134698792883, 0.0017361575258114188));
      properties.get("rightIndexFingerPitch3Link")
                .put("inertia",
                     new Matrix3D(6.02E-7,
                                  -2.3635523277102915E-8,
                                  4.1667780619474044E-9,
                                  -2.3635523277102915E-8,
                                  2.2364502233513937E-7,
                                  6.607204299095878E-8,
                                  4.1667780619474044E-9,
                                  6.607204299095878E-8,
                                  4.873549776648605E-7));
      properties.get("rightIndexFingerPitch3").put("offsetFromParentJoint", new Vector3D(0.0, -0.022550000000000042, 0.003976000000000067));
      properties.get("rightIndexFingerPitch3").put("positionLowerLimit", 0.0);
      properties.get("rightIndexFingerPitch3").put("positionUpperLimit", 1.92);
      properties.get("rightIndexFingerPitch3").put("velocityLowerLimit", -1.0);
      properties.get("rightIndexFingerPitch3").put("velocityUpperLimit", 1.0);
      properties.get("rightIndexFingerPitch3").put("effortLowerLimit", -10.0);
      properties.get("rightIndexFingerPitch3").put("effortUpperLimit", 10.0);
      properties.get("rightIndexFingerPitch3").put("kpPositionLimit", 100.0);
      properties.get("rightIndexFingerPitch3").put("kdPositionLimit", 20.0);
      properties.get("rightIndexFingerPitch3").put("kpVelocityLimit", 500.0);
      properties.get("rightIndexFingerPitch3").put("axis", new Vector3D(0.0, 0.17361607288187247, 0.9848134134124475));
      properties.get("rightIndexFingerPitch3").put("damping", 1.0);
      properties.get("rightIndexFingerPitch3").put("stiction", 0.0);
      properties.put("rightMiddleFingerPitch1", new HashMap<>());
      properties.put("rightMiddleFingerPitch1Link", new HashMap<>());
      properties.get("rightMiddleFingerPitch1Link").put("mass", 0.02);
      properties.get("rightMiddleFingerPitch1Link").put("centerOfMass", new Vector3D(0.0, -0.019850857332287003, -0.002437921896449382));
      properties.get("rightMiddleFingerPitch1Link")
                .put("inertia",
                     new Matrix3D(5.148E-6,
                                  -4.034594335475554E-8,
                                  -1.6037607515301554E-8,
                                  -4.034594335475554E-8,
                                  1.37155570823588E-6,
                                  -3.995089457228315E-7,
                                  -1.6037607515301554E-8,
                                  -3.995089457228316E-7,
                                  4.542444291764121E-6));
      properties.get("rightMiddleFingerPitch1").put("offsetFromParentJoint", new Vector3D(0.0022000000000000006, -0.09699999999999998, -0.011899999999999966));
      properties.get("rightMiddleFingerPitch1").put("positionLowerLimit", 0.0);
      properties.get("rightMiddleFingerPitch1").put("positionUpperLimit", 1.658);
      properties.get("rightMiddleFingerPitch1").put("velocityLowerLimit", -1.0);
      properties.get("rightMiddleFingerPitch1").put("velocityUpperLimit", 1.0);
      properties.get("rightMiddleFingerPitch1").put("effortLowerLimit", -10.0);
      properties.get("rightMiddleFingerPitch1").put("effortUpperLimit", 10.0);
      properties.get("rightMiddleFingerPitch1").put("kpPositionLimit", 100.0);
      properties.get("rightMiddleFingerPitch1").put("kdPositionLimit", 20.0);
      properties.get("rightMiddleFingerPitch1").put("kpVelocityLimit", 500.0);
      properties.get("rightMiddleFingerPitch1").put("axis", new Vector3D(0.0, -0.12189598527100426, 0.9925428800685697));
      properties.get("rightMiddleFingerPitch1").put("damping", 1.0);
      properties.get("rightMiddleFingerPitch1").put("stiction", 0.0);
      properties.put("rightMiddleFingerPitch2", new HashMap<>());
      properties.put("rightMiddleFingerPitch2Link", new HashMap<>());
      properties.get("rightMiddleFingerPitch2Link").put("mass", 0.011);
      properties.get("rightMiddleFingerPitch2Link").put("centerOfMass", new Vector3D(0.0, -0.012903057265986551, -0.0015846492326920985));
      properties.get("rightMiddleFingerPitch2Link")
                .put("inertia",
                     new Matrix3D(1.97E-6,
                                  3.508731450793507E-8,
                                  -6.773504308793901E-9,
                                  3.508731450793507E-8,
                                  5.98401161832773E-7,
                                  -2.835378861044921E-7,
                                  -6.773504308793901E-9,
                                  -2.835378861044921E-7,
                                  2.2275988381672266E-6));
      properties.get("rightMiddleFingerPitch2").put("offsetFromParentJoint", new Vector3D(0.0, -0.037820000000000006, -0.004644000000000044));
      properties.get("rightMiddleFingerPitch2").put("positionLowerLimit", 0.0);
      properties.get("rightMiddleFingerPitch2").put("positionUpperLimit", 1.92);
      properties.get("rightMiddleFingerPitch2").put("velocityLowerLimit", -1.0);
      properties.get("rightMiddleFingerPitch2").put("velocityUpperLimit", 1.0);
      properties.get("rightMiddleFingerPitch2").put("effortLowerLimit", -10.0);
      properties.get("rightMiddleFingerPitch2").put("effortUpperLimit", 10.0);
      properties.get("rightMiddleFingerPitch2").put("kpPositionLimit", 100.0);
      properties.get("rightMiddleFingerPitch2").put("kdPositionLimit", 20.0);
      properties.get("rightMiddleFingerPitch2").put("kpVelocityLimit", 500.0);
      properties.get("rightMiddleFingerPitch2").put("axis", new Vector3D(0.0, -0.12189598527100426, 0.9925428800685697));
      properties.get("rightMiddleFingerPitch2").put("damping", 1.0);
      properties.get("rightMiddleFingerPitch2").put("stiction", 0.0);
      properties.put("rightMiddleFingerPitch3", new HashMap<>());
      properties.put("rightMiddleFingerPitch3Link", new HashMap<>());
      properties.get("rightMiddleFingerPitch3Link").put("mass", 0.006);
      properties.get("rightMiddleFingerPitch3Link").put("centerOfMass", new Vector3D(0.0, -0.009925428666143501, -0.001218960948224691));
      properties.get("rightMiddleFingerPitch3Link")
                .put("inertia",
                     new Matrix3D(3.96E-7,
                                  -8.810989704706681E-9,
                                  -2.089607720016572E-9,
                                  -8.810989704706681E-9,
                                  1.5825320090412047E-7,
                                  -1.7766322657938948E-9,
                                  -2.089607720016572E-9,
                                  -1.7766322657938948E-9,
                                  3.0474679909587953E-7));
      properties.get("rightMiddleFingerPitch3").put("offsetFromParentJoint", new Vector3D(0.0, -0.022730000000000167, -0.002791999999999949));
      properties.get("rightMiddleFingerPitch3").put("positionLowerLimit", 0.0);
      properties.get("rightMiddleFingerPitch3").put("positionUpperLimit", 1.57);
      properties.get("rightMiddleFingerPitch3").put("velocityLowerLimit", -1.0);
      properties.get("rightMiddleFingerPitch3").put("velocityUpperLimit", 1.0);
      properties.get("rightMiddleFingerPitch3").put("effortLowerLimit", -10.0);
      properties.get("rightMiddleFingerPitch3").put("effortUpperLimit", 10.0);
      properties.get("rightMiddleFingerPitch3").put("kpPositionLimit", 100.0);
      properties.get("rightMiddleFingerPitch3").put("kdPositionLimit", 20.0);
      properties.get("rightMiddleFingerPitch3").put("kpVelocityLimit", 500.0);
      properties.get("rightMiddleFingerPitch3").put("axis", new Vector3D(0.0, -0.12189598527100426, 0.9925428800685697));
      properties.get("rightMiddleFingerPitch3").put("damping", 1.0);
      properties.get("rightMiddleFingerPitch3").put("stiction", 0.0);
      properties.put("rightPinkyPitch1", new HashMap<>());
      properties.put("rightPinkyPitch1Link", new HashMap<>());
      properties.get("rightPinkyPitch1Link").put("mass", 0.02);
      properties.get("rightPinkyPitch1Link").put("centerOfMass", new Vector3D(0.0, -0.019850857332287003, -0.002437921896449382));
      properties.get("rightPinkyPitch1Link")
                .put("inertia",
                     new Matrix3D(5.148E-6,
                                  -4.034594335475554E-8,
                                  -1.6037607515301554E-8,
                                  -4.034594335475554E-8,
                                  1.37155570823588E-6,
                                  -3.995089457228315E-7,
                                  -1.6037607515301554E-8,
                                  -3.995089457228316E-7,
                                  4.542444291764121E-6));
      properties.get("rightPinkyPitch1").put("offsetFromParentJoint", new Vector3D(0.0022000000000000006, -0.08379999999999999, -0.04099999999999998));
      properties.get("rightPinkyPitch1").put("positionLowerLimit", 0.0);
      properties.get("rightPinkyPitch1").put("positionUpperLimit", 1.57);
      properties.get("rightPinkyPitch1").put("velocityLowerLimit", -1.0);
      properties.get("rightPinkyPitch1").put("velocityUpperLimit", 1.0);
      properties.get("rightPinkyPitch1").put("effortLowerLimit", -10.0);
      properties.get("rightPinkyPitch1").put("effortUpperLimit", 10.0);
      properties.get("rightPinkyPitch1").put("kpPositionLimit", 100.0);
      properties.get("rightPinkyPitch1").put("kdPositionLimit", 20.0);
      properties.get("rightPinkyPitch1").put("kpVelocityLimit", 500.0);
      properties.get("rightPinkyPitch1").put("axis", new Vector3D(0.0, -0.12189598527100426, 0.9925428800685697));
      properties.get("rightPinkyPitch1").put("damping", 1.0);
      properties.get("rightPinkyPitch1").put("stiction", 0.0);
      properties.put("rightPinkyPitch2", new HashMap<>());
      properties.put("rightPinkyPitch2Link", new HashMap<>());
      properties.get("rightPinkyPitch2Link").put("mass", 0.011);
      properties.get("rightPinkyPitch2Link").put("centerOfMass", new Vector3D(0.0, -0.012903057265986551, -0.0015846492326920985));
      properties.get("rightPinkyPitch2Link")
                .put("inertia",
                     new Matrix3D(1.97E-6,
                                  3.508731450793507E-8,
                                  -6.773504308793901E-9,
                                  3.508731450793507E-8,
                                  5.98401161832773E-7,
                                  -2.835378861044921E-7,
                                  -6.773504308793901E-9,
                                  -2.835378861044921E-7,
                                  2.2275988381672266E-6));
      properties.get("rightPinkyPitch2").put("offsetFromParentJoint", new Vector3D(0.0, -0.037816000000000016, -0.0046439999999999025));
      properties.get("rightPinkyPitch2").put("positionLowerLimit", 0.0);
      properties.get("rightPinkyPitch2").put("positionUpperLimit", 1.658);
      properties.get("rightPinkyPitch2").put("velocityLowerLimit", -1.0);
      properties.get("rightPinkyPitch2").put("velocityUpperLimit", 1.0);
      properties.get("rightPinkyPitch2").put("effortLowerLimit", -10.0);
      properties.get("rightPinkyPitch2").put("effortUpperLimit", 10.0);
      properties.get("rightPinkyPitch2").put("kpPositionLimit", 100.0);
      properties.get("rightPinkyPitch2").put("kdPositionLimit", 20.0);
      properties.get("rightPinkyPitch2").put("kpVelocityLimit", 500.0);
      properties.get("rightPinkyPitch2").put("axis", new Vector3D(0.0, -0.12189598527100426, 0.9925428800685697));
      properties.get("rightPinkyPitch2").put("damping", 1.0);
      properties.get("rightPinkyPitch2").put("stiction", 0.0);
      properties.put("rightPinkyPitch3", new HashMap<>());
      properties.put("rightPinkyPitch3Link", new HashMap<>());
      properties.get("rightPinkyPitch3Link").put("mass", 0.006);
      properties.get("rightPinkyPitch3Link").put("centerOfMass", new Vector3D(0.0, -0.009925428666143501, -0.001218960948224691));
      properties.get("rightPinkyPitch3Link")
                .put("inertia",
                     new Matrix3D(3.96E-7,
                                  -8.810989704706681E-9,
                                  -2.089607720016572E-9,
                                  -8.810989704706681E-9,
                                  1.5825320090412047E-7,
                                  -1.7766322657938948E-9,
                                  -2.089607720016572E-9,
                                  -1.7766322657938948E-9,
                                  3.0474679909587953E-7));
      properties.get("rightPinkyPitch3").put("offsetFromParentJoint", new Vector3D(0.0, -0.022734000000000056, -0.0027920000000000765));
      properties.get("rightPinkyPitch3").put("positionLowerLimit", 0.0);
      properties.get("rightPinkyPitch3").put("positionUpperLimit", 1.92);
      properties.get("rightPinkyPitch3").put("velocityLowerLimit", -1.0);
      properties.get("rightPinkyPitch3").put("velocityUpperLimit", 1.0);
      properties.get("rightPinkyPitch3").put("effortLowerLimit", -10.0);
      properties.get("rightPinkyPitch3").put("effortUpperLimit", 10.0);
      properties.get("rightPinkyPitch3").put("kpPositionLimit", 100.0);
      properties.get("rightPinkyPitch3").put("kdPositionLimit", 20.0);
      properties.get("rightPinkyPitch3").put("kpVelocityLimit", 500.0);
      properties.get("rightPinkyPitch3").put("axis", new Vector3D(0.0, -0.12189598527100426, 0.9925428800685697));
      properties.get("rightPinkyPitch3").put("damping", 1.0);
      properties.get("rightPinkyPitch3").put("stiction", 0.0);
      properties.put("rightThumbRoll", new HashMap<>());
      properties.put("rightThumbRollLink", new HashMap<>());
      properties.get("rightThumbRollLink").put("mass", 0.017);
      properties.get("rightThumbRollLink").put("centerOfMass", new Vector3D(0.0, -0.003420522332544199, 0.009396809403865038));
      properties.get("rightThumbRollLink")
                .put("inertia",
                     new Matrix3D(2.77788E-6,
                                  3.800365322933979E-8,
                                  -8.7821261445544E-8,
                                  3.800365322933979E-8,
                                  3.2663699259248467E-6,
                                  7.322727571414368E-7,
                                  -8.7821261445544E-8,
                                  7.322727571414366E-7,
                                  1.6352200740751536E-6));
      properties.get("rightThumbRoll").put("offsetFromParentJoint", new Vector3D(0.0049, -0.03510000000000002, 0.022800000000000042));
      properties.get("rightThumbRoll").put("positionLowerLimit", 0.0);
      properties.get("rightThumbRoll").put("positionUpperLimit", 2.356);
      properties.get("rightThumbRoll").put("velocityLowerLimit", -1.0);
      properties.get("rightThumbRoll").put("velocityUpperLimit", 1.0);
      properties.get("rightThumbRoll").put("effortLowerLimit", -10.0);
      properties.get("rightThumbRoll").put("effortUpperLimit", 10.0);
      properties.get("rightThumbRoll").put("kpPositionLimit", 100.0);
      properties.get("rightThumbRoll").put("kdPositionLimit", 20.0);
      properties.get("rightThumbRoll").put("kpVelocityLimit", 500.0);
      properties.get("rightThumbRoll").put("axis", new Vector3D(0.0, 0.939681022333869, 0.34205200812972125));
      properties.get("rightThumbRoll").put("damping", 1.0);
      properties.get("rightThumbRoll").put("stiction", 0.0);
      properties.put("rightThumbPitch1", new HashMap<>());
      properties.put("rightThumbPitch1Link", new HashMap<>());
      properties.get("rightThumbPitch1Link").put("mass", 0.02);
      properties.get("rightThumbPitch1Link").put("centerOfMass", new Vector3D(0.0, -0.005, 0.02));
      properties.get("rightThumbPitch1Link").put("inertia", new Matrix3D(4.239E-6, 0.0, -0.0, 0.0, 4.582E-6, 0.0, -0.0, 0.0, 1.47E-6));
      properties.get("rightThumbPitch1").put("offsetFromParentJoint", new Vector3D(0.0, -0.007832999999999965, 0.021518999999999993));
      properties.get("rightThumbPitch1").put("positionLowerLimit", 0.0);
      properties.get("rightThumbPitch1").put("positionUpperLimit", 1.658);
      properties.get("rightThumbPitch1").put("velocityLowerLimit", -1.0);
      properties.get("rightThumbPitch1").put("velocityUpperLimit", 1.0);
      properties.get("rightThumbPitch1").put("effortLowerLimit", -10.0);
      properties.get("rightThumbPitch1").put("effortUpperLimit", 10.0);
      properties.get("rightThumbPitch1").put("kpPositionLimit", 100.0);
      properties.get("rightThumbPitch1").put("kdPositionLimit", 20.0);
      properties.get("rightThumbPitch1").put("kpVelocityLimit", 500.0);
      properties.get("rightThumbPitch1").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightThumbPitch1").put("damping", 1.0);
      properties.get("rightThumbPitch1").put("stiction", 0.0);
      properties.put("rightThumbPitch2", new HashMap<>());
      properties.put("rightThumbPitch2Link", new HashMap<>());
      properties.get("rightThumbPitch2Link").put("mass", 0.013);
      properties.get("rightThumbPitch2Link").put("centerOfMass", new Vector3D(0.0, -0.004, 0.017));
      properties.get("rightThumbPitch2Link").put("inertia", new Matrix3D(1.266E-6, 0.0, -0.0, 0.0, 1.503E-6, -0.0, -0.0, -0.0, 6.99E-7));
      properties.get("rightThumbPitch2").put("offsetFromParentJoint", new Vector3D(0.0, -0.00660000000000005, 0.03750000000000003));
      properties.get("rightThumbPitch2").put("positionLowerLimit", 0.0);
      properties.get("rightThumbPitch2").put("positionUpperLimit", 1.92);
      properties.get("rightThumbPitch2").put("velocityLowerLimit", -1.0);
      properties.get("rightThumbPitch2").put("velocityUpperLimit", 1.0);
      properties.get("rightThumbPitch2").put("effortLowerLimit", -10.0);
      properties.get("rightThumbPitch2").put("effortUpperLimit", 10.0);
      properties.get("rightThumbPitch2").put("kpPositionLimit", 100.0);
      properties.get("rightThumbPitch2").put("kdPositionLimit", 20.0);
      properties.get("rightThumbPitch2").put("kpVelocityLimit", 500.0);
      properties.get("rightThumbPitch2").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightThumbPitch2").put("damping", 1.0);
      properties.get("rightThumbPitch2").put("stiction", 0.0);
      properties.put("rightThumbPitch3", new HashMap<>());
      properties.put("rightThumbPitch3Link", new HashMap<>());
      properties.get("rightThumbPitch3Link").put("mass", 0.006);
      properties.get("rightThumbPitch3Link").put("centerOfMass", new Vector3D(0.0, -0.004, 0.01));
      properties.get("rightThumbPitch3Link").put("inertia", new Matrix3D(3.22E-7, 0.0, 0.0, 0.0, 4.06E-7, 0.0, 0.0, 0.0, 2.1E-7));
      properties.get("rightThumbPitch3").put("offsetFromParentJoint", new Vector3D(0.0, -0.004899999999999904, 0.02749999999999997));
      properties.get("rightThumbPitch3").put("positionLowerLimit", 0.0);
      properties.get("rightThumbPitch3").put("positionUpperLimit", 1.57);
      properties.get("rightThumbPitch3").put("velocityLowerLimit", -1.0);
      properties.get("rightThumbPitch3").put("velocityUpperLimit", 1.0);
      properties.get("rightThumbPitch3").put("effortLowerLimit", -10.0);
      properties.get("rightThumbPitch3").put("effortUpperLimit", 10.0);
      properties.get("rightThumbPitch3").put("kpPositionLimit", 100.0);
      properties.get("rightThumbPitch3").put("kdPositionLimit", 20.0);
      properties.get("rightThumbPitch3").put("kpVelocityLimit", 500.0);
      properties.get("rightThumbPitch3").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("rightThumbPitch3").put("damping", 1.0);
      properties.get("rightThumbPitch3").put("stiction", 0.0);

      return properties;
   }

   public static Map<String, Map<String, Object>> valkyrieSensorProperties()
   {
      Map<String, Map<String, Object>> sensorProperties = new HashMap<>();
      sensorProperties.put("pelvisMiddleImu", new HashMap<>());
      sensorProperties.get("pelvisMiddleImu")
                      .put("transformToJoint",
                           new RigidBodyTransform(1.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  -0.9999999999964793,
                                                  -2.65358979335273E-6,
                                                  0.0,
                                                  0.0,
                                                  2.65358979335273E-6,
                                                  -0.9999999999964793,
                                                  -0.108196));
      sensorProperties.get("pelvisMiddleImu").put("accelerationNoiseMean", 0.0);
      sensorProperties.get("pelvisMiddleImu").put("accelerationNoiseStandardDeviation", 0.017);
      sensorProperties.get("pelvisMiddleImu").put("accelerationBiasMean", 0.1);
      sensorProperties.get("pelvisMiddleImu").put("accelerationBiasStandardDeviation", 0.001);
      sensorProperties.get("pelvisMiddleImu").put("angularVelocityNoiseMean", 7.5E-6);
      sensorProperties.get("pelvisMiddleImu").put("angularVelocityNoiseStandardDeviation", 8.0E-7);
      sensorProperties.get("pelvisMiddleImu").put("angularVelocityBiasMean", 0.0);
      sensorProperties.get("pelvisMiddleImu").put("angularVelocityBiasStandardDeviation", 0.0);
      sensorProperties.put("pelvisRearImu", new HashMap<>());
      sensorProperties.get("pelvisRearImu")
                      .put("transformToJoint",
                           new RigidBodyTransform(-0.8886208494403026,
                                                  -1.4365406117855013E-6,
                                                  0.4586425470210231,
                                                  -0.0758449,
                                                  -2.358035216243522E-6,
                                                  0.999999999996188,
                                                  -1.4365406117855013E-6,
                                                  0.0,
                                                  -0.4586425470172111,
                                                  -2.358035216243522E-6,
                                                  -0.8886208494403026,
                                                  -0.111056));
      sensorProperties.get("pelvisRearImu").put("accelerationNoiseMean", 0.0);
      sensorProperties.get("pelvisRearImu").put("accelerationNoiseStandardDeviation", 0.017);
      sensorProperties.get("pelvisRearImu").put("accelerationBiasMean", 0.1);
      sensorProperties.get("pelvisRearImu").put("accelerationBiasStandardDeviation", 0.001);
      sensorProperties.get("pelvisRearImu").put("angularVelocityNoiseMean", 7.5E-6);
      sensorProperties.get("pelvisRearImu").put("angularVelocityNoiseStandardDeviation", 8.0E-7);
      sensorProperties.get("pelvisRearImu").put("angularVelocityBiasMean", 0.0);
      sensorProperties.get("pelvisRearImu").put("angularVelocityBiasStandardDeviation", 0.0);
      sensorProperties.put("leftHazardCamera___default__", new HashMap<>());
      sensorProperties.get("leftHazardCamera___default__")
                      .put("transformToJoint",
                           new RigidBodyTransform(7.963267107332633E-4,
                                                  -7.963264582434141E-4,
                                                  0.9999993658637698,
                                                  0.0345,
                                                  0.9999996829318346,
                                                  6.341362301717473E-7,
                                                  -7.963264582434141E-4,
                                                  0.0406,
                                                  5.55112E-17,
                                                  0.9999996829318346,
                                                  7.963267107332633E-4,
                                                  0.1135));
      sensorProperties.get("leftHazardCamera___default__").put("fieldOfView", 1.378);
      sensorProperties.get("leftHazardCamera___default__").put("clipNear", 0.1);
      sensorProperties.get("leftHazardCamera___default__").put("clipFar", 100.0);
      sensorProperties.get("leftHazardCamera___default__").put("imageWidth", 1280);
      sensorProperties.get("leftHazardCamera___default__").put("imageHeight", 1024);
      sensorProperties.put("rightHazardCamera___default__", new HashMap<>());
      sensorProperties.get("rightHazardCamera___default__")
                      .put("transformToJoint",
                           new RigidBodyTransform(7.963267107332633E-4,
                                                  -7.963264582434141E-4,
                                                  0.9999993658637698,
                                                  0.0345,
                                                  0.9999996829318346,
                                                  6.341362301717473E-7,
                                                  -7.963264582434141E-4,
                                                  -0.0406,
                                                  5.55112E-17,
                                                  0.9999996829318346,
                                                  7.963267107332633E-4,
                                                  0.1135));
      sensorProperties.get("rightHazardCamera___default__").put("fieldOfView", 1.378);
      sensorProperties.get("rightHazardCamera___default__").put("clipNear", 0.1);
      sensorProperties.get("rightHazardCamera___default__").put("clipFar", 100.0);
      sensorProperties.get("rightHazardCamera___default__").put("imageWidth", 1280);
      sensorProperties.get("rightHazardCamera___default__").put("imageHeight", 1024);
      sensorProperties.put("leftTorsoImu", new HashMap<>());
      sensorProperties.get("leftTorsoImu")
                      .put("transformToJoint",
                           new RigidBodyTransform(1.0,
                                                  -0.0,
                                                  0.0,
                                                  -0.0627634,
                                                  0.0,
                                                  9.632679474766714E-5,
                                                  0.9999999953605743,
                                                  0.134239,
                                                  -0.0,
                                                  -0.9999999953605743,
                                                  9.632679474766714E-5,
                                                  0.363068));
      sensorProperties.get("leftTorsoImu").put("accelerationNoiseMean", 0.0);
      sensorProperties.get("leftTorsoImu").put("accelerationNoiseStandardDeviation", 0.017);
      sensorProperties.get("leftTorsoImu").put("accelerationBiasMean", 0.1);
      sensorProperties.get("leftTorsoImu").put("accelerationBiasStandardDeviation", 0.001);
      sensorProperties.get("leftTorsoImu").put("angularVelocityNoiseMean", 7.5E-6);
      sensorProperties.get("leftTorsoImu").put("angularVelocityNoiseStandardDeviation", 8.0E-7);
      sensorProperties.get("leftTorsoImu").put("angularVelocityBiasMean", 0.0);
      sensorProperties.get("leftTorsoImu").put("angularVelocityBiasStandardDeviation", 0.0);
      sensorProperties.put("stereo_camera_left", new HashMap<>());
      sensorProperties.get("stereo_camera_left")
                      .put("transformToJoint",
                           new RigidBodyTransform(0.991444821419641,
                                                  -3.46363776756234E-7,
                                                  -0.13052649570127964,
                                                  0.183847,
                                                  5.293958426245172E-23,
                                                  -0.9999999999964793,
                                                  2.65358979335273E-6,
                                                  -0.035,
                                                  -0.1305264957017392,
                                                  -2.6308878587915793E-6,
                                                  -0.9914448214161503,
                                                  0.0773366));
      sensorProperties.get("stereo_camera_left").put("fieldOfView", 1.39626);
      sensorProperties.get("stereo_camera_left").put("clipNear", 0.02);
      sensorProperties.get("stereo_camera_left").put("clipFar", 300.0);
      sensorProperties.get("stereo_camera_left").put("imageWidth", 1024);
      sensorProperties.get("stereo_camera_left").put("imageHeight", 544);
      sensorProperties.put("stereo_camera_right", new HashMap<>());
      sensorProperties.get("stereo_camera_right")
                      .put("transformToJoint",
                           new RigidBodyTransform(0.991444821419641,
                                                  -3.46363776756234E-7,
                                                  -0.13052649570127964,
                                                  0.1838470242454644,
                                                  5.293958426245172E-23,
                                                  -0.9999999999964793,
                                                  2.65358979335273E-6,
                                                  0.03499999999975355,
                                                  -0.1305264957017392,
                                                  -2.6308878587915793E-6,
                                                  -0.9914448214161503,
                                                  0.07733678416215012));
      sensorProperties.get("stereo_camera_right").put("fieldOfView", 1.39626);
      sensorProperties.get("stereo_camera_right").put("clipNear", 0.02);
      sensorProperties.get("stereo_camera_right").put("clipFar", 300.0);
      sensorProperties.get("stereo_camera_right").put("imageWidth", 1024);
      sensorProperties.get("stereo_camera_right").put("imageHeight", 544);
      sensorProperties.put("head_imu_sensor", new HashMap<>());
      sensorProperties.get("head_imu_sensor")
                      .put("transformToJoint",
                           new RigidBodyTransform(0.991444821419641,
                                                  -3.46363776756234E-7,
                                                  -0.13052649570127964,
                                                  0.136492,
                                                  5.293958426245172E-23,
                                                  -0.9999999999964793,
                                                  2.65358979335273E-6,
                                                  -0.035,
                                                  -0.1305264957017392,
                                                  -2.6308878587915793E-6,
                                                  -0.9914448214161503,
                                                  0.0815537));
      sensorProperties.get("head_imu_sensor").put("accelerationNoiseMean", 0.0);
      sensorProperties.get("head_imu_sensor").put("accelerationNoiseStandardDeviation", 0.017);
      sensorProperties.get("head_imu_sensor").put("accelerationBiasMean", 0.1);
      sensorProperties.get("head_imu_sensor").put("accelerationBiasStandardDeviation", 0.001);
      sensorProperties.get("head_imu_sensor").put("angularVelocityNoiseMean", 7.5E-6);
      sensorProperties.get("head_imu_sensor").put("angularVelocityNoiseStandardDeviation", 8.0E-7);
      sensorProperties.get("head_imu_sensor").put("angularVelocityBiasMean", 0.0);
      sensorProperties.get("head_imu_sensor").put("angularVelocityBiasStandardDeviation", 0.0);
      sensorProperties.put("head_hokuyo_sensor", new HashMap<>());
      sensorProperties.get("head_hokuyo_sensor")
                      .put("transformToJoint",
                           new RigidBodyTransform(0.991444821419641,
                                                  -3.46363776756234E-7,
                                                  -0.13052649570127964,
                                                  0.027785447207070033,
                                                  5.293958426245172E-23,
                                                  -0.9999999999964793,
                                                  2.65358979335273E-6,
                                                  3.9803846900290945E-8,
                                                  -0.1305264957017392,
                                                  -2.6308878587915793E-6,
                                                  -0.9914448214161503,
                                                  -0.01878746719229443));
      sensorProperties.get("head_hokuyo_sensor").put("sweepYawMin", -2.35619);
      sensorProperties.get("head_hokuyo_sensor").put("sweepYawMax", 2.35619);
      sensorProperties.get("head_hokuyo_sensor").put("heightPitchMin", 0.0);
      sensorProperties.get("head_hokuyo_sensor").put("heightPitchMax", 0.0);
      sensorProperties.get("head_hokuyo_sensor").put("minRange", 0.1);
      sensorProperties.get("head_hokuyo_sensor").put("maxRange", 30.0);
      sensorProperties.get("head_hokuyo_sensor").put("pointsPerSweep", 1080);
      sensorProperties.get("head_hokuyo_sensor").put("scanHeight", 1);
      return sensorProperties;
   }
}
