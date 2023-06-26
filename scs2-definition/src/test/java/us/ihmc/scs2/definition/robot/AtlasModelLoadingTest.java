package us.ihmc.scs2.definition.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static us.ihmc.scs2.definition.robot.ValkyrieModelLoadingTest.assertKinematicsContinuity;
import static us.ihmc.scs2.definition.robot.ValkyrieModelLoadingTest.assertPhysicalProperties;
import static us.ihmc.scs2.definition.robot.ValkyrieModelLoadingTest.assertSensorsProperties;
import static us.ihmc.scs2.definition.robot.ValkyrieModelLoadingTest.subtract;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import javax.xml.bind.JAXBException;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

public class AtlasModelLoadingTest
{
   private static final String LeftHipYawName = "l_leg_hpz";
   private static final String LeftHipRollName = "l_leg_hpx";
   private static final String LeftHipPitchName = "l_leg_hpy";
   private static final String LeftKneePitchName = "l_leg_kny";
   private static final String LeftAnklePitchName = "l_leg_aky";
   private static final String LeftAnkleRollName = "l_leg_akx";

   private static final String RightHipYawName = "r_leg_hpz";
   private static final String RightHipRollName = "r_leg_hpx";
   private static final String RightHipPitchName = "r_leg_hpy";
   private static final String RightKneePitchName = "r_leg_kny";
   private static final String RightAnklePitchName = "r_leg_aky";
   private static final String RightAnkleRollName = "r_leg_akx";

   private static final String TorsoYawName = "back_bkz";
   private static final String TorsoPitchName = "back_bky";
   private static final String TorsoRollName = "back_bkx";

   private static final String NeckPitchName = "neck_ry";

   private static final String HokuyoJointName = "hokuyo_joint";

   private static final String LeftShoulderYawName = "l_arm_shz";
   private static final String LeftShoulderRollName = "l_arm_shx";
   private static final String LeftElbowPitchName = "l_arm_ely";
   private static final String LeftElbowRollName = "l_arm_elx";
   private static final String LeftWristPitchName = "l_arm_wry";
   private static final String LeftWristRollName = "l_arm_wrx";
   private static final String LeftWristSecondPitchName = "l_arm_wry2";

   private static final String RightShoulderYawName = "r_arm_shz";
   private static final String RightShoulderRollName = "r_arm_shx";
   private static final String RightElbowPitchName = "r_arm_ely";
   private static final String RightElbowRollName = "r_arm_elx";
   private static final String RightWristPitchName = "r_arm_wry";
   private static final String RightWristRollName = "r_arm_wrx";
   private static final String RightWristSecondPitchName = "r_arm_wry2";

   private static final String PelvisName = "pelvis";

   private static final String LeftHipYawLinkName = "l_uglut";
   private static final String LeftHipRollLinkName = "l_lglut";
   private static final String LeftHipPitchLinkName = "l_uleg";
   private static final String LeftKneePitchLinkName = "l_lleg";
   private static final String LeftAnklePitchLinkName = "l_talus";
   private static final String LeftFootName = "l_foot";

   private static final String RightHipYawLinkName = "r_uglut";
   private static final String RightHipRollLinkName = "r_lglut";
   private static final String RightHipPitchLinkName = "r_uleg";
   private static final String RightKneePitchLinkName = "r_lleg";
   private static final String RightAnklePitchLinkName = "r_talus";
   private static final String RightFootName = "r_foot";

   private static final String TorsoYawLinkName = "ltorso";
   private static final String TorsoPitchLinkName = "mtorso";
   private static final String TorsoRollLinkName = "utorso";

   private static final String NeckPitchLinkName = "head";

   private static final String HokuyoLinkName = "hokuyo_link";

   private static final String LeftShoulderYawLinkName = "l_clav";
   private static final String LeftShoulderRollLinkName = "l_scap";
   private static final String LeftElbowPitchLinkName = "l_uarm";
   private static final String LeftElbowRollLinkName = "l_larm";
   private static final String LeftWristPitchLinkName = "l_ufarm";
   private static final String LeftWristRollLinkName = "l_lfarm";
   private static final String LeftWristSecondPitchLinkName = "l_hand";

   private static final String RightShoulderYawLinkName = "r_clav";
   private static final String RightShoulderRollLinkName = "r_scap";
   private static final String RightElbowPitchLinkName = "r_uarm";
   private static final String RightElbowRollLinkName = "r_larm";
   private static final String RightWristPitchLinkName = "r_ufarm";
   private static final String RightWristRollLinkName = "r_lfarm";
   private static final String RightWristSecondPitchLinkName = "r_hand";

   private static final String[] LeftLegJointNames = {LeftHipYawName, LeftHipRollName, LeftHipPitchName, LeftKneePitchName, LeftAnklePitchName,
         LeftAnkleRollName};
   private static final String[] RightLegJointNames = {RightHipYawName, RightHipRollName, RightHipPitchName, RightKneePitchName, RightAnklePitchName,
         RightAnkleRollName};
   private static final String[] TorsoJointNames = {TorsoYawName, TorsoPitchName, TorsoRollName};
   private static final String[] NeckJointNames = {NeckPitchName};
   private static final String[] LeftArmJointNames = {LeftShoulderYawName, LeftShoulderRollName, LeftElbowPitchName, LeftElbowRollName, LeftWristPitchName,
         LeftWristRollName, LeftWristSecondPitchName};
   private static final String[] LeftFinger1JointNames = {"l_palm_finger_1_joint", "l_finger_1_joint_1", "l_finger_1_joint_2", "l_finger_1_joint_3"};
   private static final String[] LeftFinger2JointNames = {"l_palm_finger_2_joint", "l_finger_2_joint_1", "l_finger_2_joint_2", "l_finger_2_joint_3"};
   private static final String[] LeftMiddleFingerJointNames = {"l_palm_finger_middle_joint", "l_finger_middle_joint_1", "l_finger_middle_joint_2",
         "l_finger_middle_joint_3"};
   private static final String[] RightFinger1JointNames = {"r_palm_finger_1_joint", "r_finger_1_joint_1", "r_finger_1_joint_2", "r_finger_1_joint_3"};
   private static final String[] RightFinger2JointNames = {"r_palm_finger_2_joint", "r_finger_2_joint_1", "r_finger_2_joint_2", "r_finger_2_joint_3"};
   private static final String[] RightMiddleFingerJointNames = {"r_palm_finger_middle_joint", "r_finger_middle_joint_1", "r_finger_middle_joint_2",
         "r_finger_middle_joint_3"};
   private static final String[] RightArmJointNames = {RightShoulderYawName, RightShoulderRollName, RightElbowPitchName, RightElbowRollName,
         RightWristPitchName, RightWristRollName, RightWristSecondPitchName};
   private static final String[] AllJointNames = ValkyrieModelLoadingTest.concatenate(new String[] {PelvisName, HokuyoJointName},
                                                                                      LeftLegJointNames,
                                                                                      RightLegJointNames,
                                                                                      TorsoJointNames,
                                                                                      NeckJointNames,
                                                                                      LeftArmJointNames,
                                                                                      LeftFinger1JointNames,
                                                                                      LeftFinger2JointNames,
                                                                                      LeftMiddleFingerJointNames,
                                                                                      RightArmJointNames,
                                                                                      RightFinger1JointNames,
                                                                                      RightFinger2JointNames,
                                                                                      RightMiddleFingerJointNames);

   private static final String[] LeftLegLinkNames = {LeftHipYawLinkName, LeftHipRollLinkName, LeftHipPitchLinkName, LeftKneePitchLinkName,
         LeftAnklePitchLinkName, LeftFootName};
   private static final String[] RightLegLinkNames = {RightHipYawLinkName, RightHipRollLinkName, RightHipPitchLinkName, RightKneePitchLinkName,
         RightAnklePitchLinkName, RightFootName};
   private static final String[] TorsoLinkNames = {TorsoYawLinkName, TorsoPitchLinkName, TorsoRollLinkName};
   private static final String[] NeckLinkNames = {NeckPitchLinkName};
   private static final String[] LeftArmLinkNames = {LeftShoulderYawLinkName, LeftShoulderRollLinkName, LeftElbowPitchLinkName, LeftElbowRollLinkName,
         LeftWristPitchLinkName, LeftWristRollLinkName, LeftWristSecondPitchLinkName};
   private static final String[] LeftFinger1LinkNames = {"l_finger_1_link_0", "l_finger_1_link_1", "l_finger_1_link_2", "l_finger_1_link_3"};
   private static final String[] LeftFinger2LinkNames = {"l_finger_2_link_0", "l_finger_2_link_1", "l_finger_2_link_2", "l_finger_2_link_3"};
   private static final String[] LeftMiddleFingerLinkNames = {"l_finger_middle_link_0", "l_finger_middle_link_1", "l_finger_middle_link_2",
         "l_finger_middle_link_3"};
   private static final String[] RightFinger1LinkNames = {"r_finger_1_link_0", "r_finger_1_link_1", "r_finger_1_link_2", "r_finger_1_link_3"};
   private static final String[] RightFinger2LinkNames = {"r_finger_2_link_0", "r_finger_2_link_1", "r_finger_2_link_2", "r_finger_2_link_3"};
   private static final String[] RightMiddleFingerLinkNames = {"r_finger_middle_link_0", "r_finger_middle_link_1", "r_finger_middle_link_2",
         "r_finger_middle_link_3"};
   private static final String[] RightArmLinkNames = {RightShoulderYawLinkName, RightShoulderRollLinkName, RightElbowPitchLinkName, RightElbowRollLinkName,
         RightWristPitchLinkName, RightWristRollLinkName, RightWristSecondPitchLinkName};
   private static final String[] AllLinkNames = ValkyrieModelLoadingTest.concatenate(new String[] {PelvisName, HokuyoLinkName},
                                                                                     LeftLegLinkNames,
                                                                                     RightLegLinkNames,
                                                                                     TorsoLinkNames,
                                                                                     NeckLinkNames,
                                                                                     LeftArmLinkNames,
                                                                                     LeftFinger1LinkNames,
                                                                                     LeftFinger2LinkNames,
                                                                                     LeftMiddleFingerLinkNames,
                                                                                     RightArmLinkNames,
                                                                                     RightFinger1LinkNames,
                                                                                     RightFinger2LinkNames,
                                                                                     RightMiddleFingerLinkNames);

   private static final double Infinity = Double.POSITIVE_INFINITY;

   @Test
   public void testSDFTools() throws Exception
   {
      InputStream resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/atlas/atlas_unplugged_v5_dual_robotiq.sdf");
      SDFRoot sdfRoot = SDFTools.loadSDFRoot(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition robotDefinition = SDFTools.toFloatingRobotDefinition(sdfRoot.getModels().get(0));
      performAssertionsOnRobotDefinition(robotDefinition);
   }

   @Test
   public void testURDFTools() throws Exception
   {
      InputStream resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/atlas/atlas_unplugged_v5_dual_robotiq_with_head.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel);
      performAssertionsOnRobotDefinition(robotDefinition);
   }

   @Test
   public void testSaveLoadRobotDefinitionXML() throws JAXBException, FileNotFoundException, IOException
   {
      InputStream resourceAsStream = this.getClass().getClassLoader().getResourceAsStream("models/atlas/atlas_unplugged_v5_dual_robotiq_with_head.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(resourceAsStream, Collections.emptyList(), this.getClass().getClassLoader());
      RobotDefinition exportedRobotDefinition = URDFTools.toRobotDefinition(urdfModel);
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
         assertNotNull(robotDefinition.getJointDefinition(jointName));
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

      assertPhysicalProperties(robotDefinition, atlasProperties(), subtract(AllJointNames, HokuyoJointName), AllLinkNames);
      assertSensorsProperties(robotDefinition, atlasSensorProperties(), AllJointNames);
   }

   private static Map<String, Map<String, Object>> atlasProperties()
   {
      // Generated from working parser.
      Map<String, Map<String, Object>> properties = new HashMap<>();
      properties.put("pelvis", new HashMap<>());
      properties.put("pelvis", new HashMap<>());
      properties.get("pelvis").put("mass", 9.609);
      properties.get("pelvis").put("centerOfMass", new Vector3D(0.011608, 0.0, 0.0266707));
      properties.get("pelvis").put("inertia", new Matrix3D(0.125568, 8.0E-4, -5.00733E-4, 8.0E-4, 0.0972042, -5.0E-4, -5.00733E-4, -5.0E-4, 0.117936));
      properties.get("pelvis").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.put("back_bkz", new HashMap<>());
      properties.put("ltorso", new HashMap<>());
      properties.get("ltorso").put("mass", 2.27);
      properties.get("ltorso").put("centerOfMass", new Vector3D(-0.0112984, -3.15366E-6, 0.0746835));
      properties.get("ltorso").put("inertia",
                                   new Matrix3D(0.0039092, -5.04491E-8, -3.42157E-4, -5.04491E-8, 0.00341694, 4.87119E-7, -3.42157E-4, 4.87119E-7, 0.00174492));
      properties.get("back_bkz").put("offsetFromParentJoint", new Vector3D(-0.0125, 0.0, 0.0));
      properties.get("back_bkz").put("positionLowerLimit", -0.663225);
      properties.get("back_bkz").put("positionUpperLimit", 0.663225);
      properties.get("back_bkz").put("velocityLowerLimit", -12.0);
      properties.get("back_bkz").put("velocityUpperLimit", 12.0);
      properties.get("back_bkz").put("effortLowerLimit", -106.0);
      properties.get("back_bkz").put("effortUpperLimit", 106.0);
      properties.get("back_bkz").put("kpPositionLimit", 100.0);
      properties.get("back_bkz").put("kdPositionLimit", 20.0);
      properties.get("back_bkz").put("kpVelocityLimit", 500.0);
      properties.get("back_bkz").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("back_bkz").put("damping", 0.1);
      properties.get("back_bkz").put("stiction", 0.0);
      properties.put("back_bky", new HashMap<>());
      properties.put("mtorso", new HashMap<>());
      properties.get("mtorso").put("mass", 0.799);
      properties.get("mtorso").put("centerOfMass", new Vector3D(-0.00816266, -0.0131245, 0.0305974));
      properties.get("mtorso").put("inertia",
                                   new Matrix3D(4.54181E-4, -6.10764E-5, 3.94009E-5, -6.10764E-5, 4.83282E-4, 5.27463E-5, 3.94009E-5, 5.27463E-5, 4.44215E-4));
      properties.get("back_bky").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.162));
      properties.get("back_bky").put("positionLowerLimit", -0.219388);
      properties.get("back_bky").put("positionUpperLimit", 0.538783);
      properties.get("back_bky").put("velocityLowerLimit", -9.0);
      properties.get("back_bky").put("velocityUpperLimit", 9.0);
      properties.get("back_bky").put("effortLowerLimit", -445.0);
      properties.get("back_bky").put("effortUpperLimit", 445.0);
      properties.get("back_bky").put("kpPositionLimit", 100.0);
      properties.get("back_bky").put("kdPositionLimit", 20.0);
      properties.get("back_bky").put("kpVelocityLimit", 500.0);
      properties.get("back_bky").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("back_bky").put("damping", 0.1);
      properties.get("back_bky").put("stiction", 0.0);
      properties.put("back_bkx", new HashMap<>());
      properties.put("utorso", new HashMap<>());
      properties.get("utorso").put("mass", 84.609);
      properties.get("utorso").put("centerOfMass", new Vector3D(-0.0616866, 0.00229456, 0.316809));
      properties.get("utorso").put("inertia", new Matrix3D(1.62389, -0.0319003, 0.0816618, -0.0319003, 1.65538, 0.0472154, 0.0816618, 0.0472154, 0.577362));
      properties.get("back_bkx").put("offsetFromParentJoint", new Vector3D(0.005705000000000001, 0.0, 0.04999999999999999));
      properties.get("back_bkx").put("positionLowerLimit", -0.4);
      properties.get("back_bkx").put("positionUpperLimit", 0.4);
      properties.get("back_bkx").put("velocityLowerLimit", -12.0);
      properties.get("back_bkx").put("velocityUpperLimit", 12.0);
      properties.get("back_bkx").put("effortLowerLimit", -300.0);
      properties.get("back_bkx").put("effortUpperLimit", 300.0);
      properties.get("back_bkx").put("kpPositionLimit", 100.0);
      properties.get("back_bkx").put("kdPositionLimit", 20.0);
      properties.get("back_bkx").put("kpVelocityLimit", 500.0);
      properties.get("back_bkx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("back_bkx").put("damping", 0.1);
      properties.get("back_bkx").put("stiction", 0.0);
      properties.put("l_arm_shz", new HashMap<>());
      properties.put("l_clav", new HashMap<>());
      properties.get("l_clav").put("mass", 4.466);
      properties.get("l_clav").put("centerOfMass", new Vector3D(0.0, 0.0, -0.084));
      properties.get("l_clav").put("inertia", new Matrix3D(0.011, 0.0, 0.0, 0.0, 0.009, -0.004, 0.0, -0.004, 0.004));
      properties.get("l_arm_shz").put("offsetFromParentJoint", new Vector3D(0.134895, 0.2256, 0.4776));
      properties.get("l_arm_shz").put("positionLowerLimit", -1.5708);
      properties.get("l_arm_shz").put("positionUpperLimit", 0.785398);
      properties.get("l_arm_shz").put("velocityLowerLimit", -12.0);
      properties.get("l_arm_shz").put("velocityUpperLimit", 12.0);
      properties.get("l_arm_shz").put("effortLowerLimit", -87.0);
      properties.get("l_arm_shz").put("effortUpperLimit", 87.0);
      properties.get("l_arm_shz").put("kpPositionLimit", 100.0);
      properties.get("l_arm_shz").put("kdPositionLimit", 20.0);
      properties.get("l_arm_shz").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_shz").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("l_arm_shz").put("damping", 0.1);
      properties.get("l_arm_shz").put("stiction", 0.0);
      properties.put("l_arm_shx", new HashMap<>());
      properties.put("l_scap", new HashMap<>());
      properties.get("l_scap").put("mass", 3.899);
      properties.get("l_scap").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("l_scap").put("inertia", new Matrix3D(0.00319, 0.0, 0.0, 0.0, 0.00583, 0.0, 0.0, 0.0, 0.00583));
      properties.get("l_arm_shx").put("offsetFromParentJoint", new Vector3D(0.0, 0.11000000000000001, -0.245));
      properties.get("l_arm_shx").put("positionLowerLimit", -1.5708);
      properties.get("l_arm_shx").put("positionUpperLimit", 1.5708);
      properties.get("l_arm_shx").put("velocityLowerLimit", -12.0);
      properties.get("l_arm_shx").put("velocityUpperLimit", 12.0);
      properties.get("l_arm_shx").put("effortLowerLimit", -99.0);
      properties.get("l_arm_shx").put("effortUpperLimit", 99.0);
      properties.get("l_arm_shx").put("kpPositionLimit", 100.0);
      properties.get("l_arm_shx").put("kdPositionLimit", 20.0);
      properties.get("l_arm_shx").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_shx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("l_arm_shx").put("damping", 0.1);
      properties.get("l_arm_shx").put("stiction", 0.0);
      properties.put("l_arm_ely", new HashMap<>());
      properties.put("l_uarm", new HashMap<>());
      properties.get("l_uarm").put("mass", 4.386);
      properties.get("l_uarm").put("centerOfMass", new Vector3D(0.0, -0.065, 0.0));
      properties.get("l_uarm").put("inertia", new Matrix3D(0.00656, 0.0, 0.0, 0.0, 0.00358, 0.0, 0.0, 0.0, 0.00656));
      properties.get("l_arm_ely").put("offsetFromParentJoint", new Vector3D(0.0, 0.18699999999999994, -0.016000000000000014));
      properties.get("l_arm_ely").put("positionLowerLimit", 0.0);
      properties.get("l_arm_ely").put("positionUpperLimit", 3.14159);
      properties.get("l_arm_ely").put("velocityLowerLimit", -12.0);
      properties.get("l_arm_ely").put("velocityUpperLimit", 12.0);
      properties.get("l_arm_ely").put("effortLowerLimit", -63.0);
      properties.get("l_arm_ely").put("effortUpperLimit", 63.0);
      properties.get("l_arm_ely").put("kpPositionLimit", 100.0);
      properties.get("l_arm_ely").put("kdPositionLimit", 20.0);
      properties.get("l_arm_ely").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_ely").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("l_arm_ely").put("damping", 0.1);
      properties.get("l_arm_ely").put("stiction", 0.0);
      properties.put("l_arm_elx", new HashMap<>());
      properties.put("l_larm", new HashMap<>());
      properties.get("l_larm").put("mass", 3.248);
      properties.get("l_larm").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("l_larm").put("inertia", new Matrix3D(0.00265, 0.0, 0.0, 0.0, 0.00446, 0.0, 0.0, 0.0, 0.00446));
      properties.get("l_arm_elx").put("offsetFromParentJoint", new Vector3D(0.0, 0.119, 0.009200000000000041));
      properties.get("l_arm_elx").put("positionLowerLimit", 0.0);
      properties.get("l_arm_elx").put("positionUpperLimit", 2.35619);
      properties.get("l_arm_elx").put("velocityLowerLimit", -12.0);
      properties.get("l_arm_elx").put("velocityUpperLimit", 12.0);
      properties.get("l_arm_elx").put("effortLowerLimit", -112.0);
      properties.get("l_arm_elx").put("effortUpperLimit", 112.0);
      properties.get("l_arm_elx").put("kpPositionLimit", 100.0);
      properties.get("l_arm_elx").put("kdPositionLimit", 20.0);
      properties.get("l_arm_elx").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_elx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("l_arm_elx").put("damping", 0.1);
      properties.get("l_arm_elx").put("stiction", 0.0);
      properties.put("l_arm_wry", new HashMap<>());
      properties.put("l_ufarm", new HashMap<>());
      properties.get("l_ufarm").put("mass", 2.4798);
      properties.get("l_ufarm").put("centerOfMass", new Vector3D(1.5E-4, -0.08296, 3.7E-4));
      properties.get("l_ufarm").put("inertia", new Matrix3D(0.012731, 0.0, 0.0, 0.0, 0.002857, 0.0, 0.0, 0.0, 0.011948));
      properties.get("l_arm_wry").put("offsetFromParentJoint", new Vector3D(0.0, 0.2995500000000001, -0.009209999999999996));
      properties.get("l_arm_wry").put("positionLowerLimit", -3.011);
      properties.get("l_arm_wry").put("positionUpperLimit", 3.011);
      properties.get("l_arm_wry").put("velocityLowerLimit", -10.0);
      properties.get("l_arm_wry").put("velocityUpperLimit", 10.0);
      properties.get("l_arm_wry").put("effortLowerLimit", -25.0);
      properties.get("l_arm_wry").put("effortUpperLimit", 25.0);
      properties.get("l_arm_wry").put("kpPositionLimit", 100.0);
      properties.get("l_arm_wry").put("kdPositionLimit", 20.0);
      properties.get("l_arm_wry").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_wry").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("l_arm_wry").put("damping", 0.1);
      properties.get("l_arm_wry").put("stiction", 0.0);
      properties.put("l_arm_wrx", new HashMap<>());
      properties.put("l_lfarm", new HashMap<>());
      properties.get("l_lfarm").put("mass", 0.648);
      properties.get("l_lfarm").put("centerOfMass", new Vector3D(1.7E-4, 0.02515, 0.00163));
      properties.get("l_lfarm").put("inertia", new Matrix3D(7.64E-4, 0.0, 0.0, 0.0, 4.29E-4, 0.0, 0.0, 0.0, 8.25E-4));
      properties.get("l_arm_wrx").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("l_arm_wrx").put("positionLowerLimit", -1.7628);
      properties.get("l_arm_wrx").put("positionUpperLimit", 1.7628);
      properties.get("l_arm_wrx").put("velocityLowerLimit", -10.0);
      properties.get("l_arm_wrx").put("velocityUpperLimit", 10.0);
      properties.get("l_arm_wrx").put("effortLowerLimit", -25.0);
      properties.get("l_arm_wrx").put("effortUpperLimit", 25.0);
      properties.get("l_arm_wrx").put("kpPositionLimit", 100.0);
      properties.get("l_arm_wrx").put("kdPositionLimit", 20.0);
      properties.get("l_arm_wrx").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_wrx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("l_arm_wrx").put("damping", 0.1);
      properties.get("l_arm_wrx").put("stiction", 0.0);
      properties.put("l_arm_wry2", new HashMap<>());
      properties.put("l_hand", new HashMap<>());
      properties.get("l_hand").put("mass", 1.8839);
      properties.get("l_hand").put("centerOfMass", new Vector3D(-0.00118561, 0.141218, 6.19884E-6));
      properties.get("l_hand").put("inertia",
                                   new Matrix3D(0.00689651, 6.78926E-5, -1.57141E-8, 6.78926E-5, 0.00397853, 6.96335E-7, -1.57141E-8, 6.96335E-7, 0.00688905));
      properties.get("l_arm_wry2").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("l_arm_wry2").put("positionLowerLimit", -2.9671);
      properties.get("l_arm_wry2").put("positionUpperLimit", 2.9671);
      properties.get("l_arm_wry2").put("velocityLowerLimit", -10.0);
      properties.get("l_arm_wry2").put("velocityUpperLimit", 10.0);
      properties.get("l_arm_wry2").put("effortLowerLimit", -25.0);
      properties.get("l_arm_wry2").put("effortUpperLimit", 25.0);
      properties.get("l_arm_wry2").put("kpPositionLimit", 100.0);
      properties.get("l_arm_wry2").put("kdPositionLimit", 20.0);
      properties.get("l_arm_wry2").put("kpVelocityLimit", 500.0);
      properties.get("l_arm_wry2").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("l_arm_wry2").put("damping", 0.1);
      properties.get("l_arm_wry2").put("stiction", 0.0);
      properties.put("l_palm_finger_1_joint", new HashMap<>());
      properties.put("l_finger_1_link_0", new HashMap<>());
      properties.get("l_finger_1_link_0").put("mass", 0.1);
      properties.get("l_finger_1_link_0").put("centerOfMass", new Vector3D(3.673203938689571E-8, -0.00999999682924925, 7.963269158366412E-6));
      properties.get("l_finger_1_link_0").put("inertia",
                                              new Matrix3D(2.499999999990555E-5,
                                                           2.5712419418057096E-11,
                                                           -2.0475498146155928E-14,
                                                           2.5712419418057096E-11,
                                                           1.800000443905034E-5,
                                                           5.574286643388569E-9,
                                                           -2.0475498146143003E-14,
                                                           5.574286643388572E-9,
                                                           2.4999995561044105E-5));
      properties.get("l_palm_finger_1_joint").put("offsetFromParentJoint", new Vector3D(-0.037790299999999985, 0.21040000000000003, -0.04550000000000004));
      properties.get("l_palm_finger_1_joint").put("positionLowerLimit", -0.2967);
      properties.get("l_palm_finger_1_joint").put("positionUpperLimit", 0.2967);
      properties.get("l_palm_finger_1_joint").put("velocityLowerLimit", -Infinity);
      properties.get("l_palm_finger_1_joint").put("velocityUpperLimit", Infinity);
      properties.get("l_palm_finger_1_joint").put("effortLowerLimit", -60.0);
      properties.get("l_palm_finger_1_joint").put("effortUpperLimit", 60.0);
      properties.get("l_palm_finger_1_joint").put("kpPositionLimit", 100.0);
      properties.get("l_palm_finger_1_joint").put("kdPositionLimit", 20.0);
      properties.get("l_palm_finger_1_joint").put("kpVelocityLimit", 0.0);
      properties.get("l_palm_finger_1_joint").put("axis", new Vector3D(6.326787993849492E-6, 7.96326747494098E-4, 0.9999996829117912));
      properties.get("l_palm_finger_1_joint").put("damping", 1.0);
      properties.get("l_palm_finger_1_joint").put("stiction", 0.0);
      properties.put("l_finger_1_joint_1", new HashMap<>());
      properties.put("l_finger_1_link_1", new HashMap<>());
      properties.get("l_finger_1_link_1").put("mass", 0.047);
      properties.get("l_finger_1_link_1").put("centerOfMass", new Vector3D(-1.6047369155512067E-7, 0.02818764799107845, -0.015522451504362931));
      properties.get("l_finger_1_link_1").put("inertia",
                                              new Matrix3D(6.0619200010434345E-5,
                                                           -8.326117546483128E-10,
                                                           3.974703474676766E-10,
                                                           -8.326117546483127E-10,
                                                           2.0475715789274972E-5,
                                                           2.6077577618842952E-5,
                                                           3.974703474676766E-10,
                                                           2.607757761884295E-5,
                                                           4.886290420029068E-5));
      properties.get("l_finger_1_joint_1").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("l_finger_1_joint_1").put("positionLowerLimit", 0.0);
      properties.get("l_finger_1_joint_1").put("positionUpperLimit", 1.2217);
      properties.get("l_finger_1_joint_1").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_1_joint_1").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_1_joint_1").put("effortLowerLimit", -60.0);
      properties.get("l_finger_1_joint_1").put("effortUpperLimit", 60.0);
      properties.get("l_finger_1_joint_1").put("kpPositionLimit", 100.0);
      properties.get("l_finger_1_joint_1").put("kdPositionLimit", 20.0);
      properties.get("l_finger_1_joint_1").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_1_joint_1").put("axis", new Vector3D(0.9999999999529641, 7.346409999654455E-6, -6.332649999702139E-6));
      properties.get("l_finger_1_joint_1").put("damping", 1.0);
      properties.get("l_finger_1_joint_1").put("stiction", 0.0);
      properties.put("l_finger_1_joint_2", new HashMap<>());
      properties.put("l_finger_1_link_2", new HashMap<>());
      properties.get("l_finger_1_link_2").put("mass", 0.03284);
      properties.get("l_finger_1_link_2").put("centerOfMass", new Vector3D(1.797414813764432E-4, 0.019035900919603402, -0.010365995812217704));
      properties.get("l_finger_1_link_2").put("inertia",
                                              new Matrix3D(1.986349688608241E-5,
                                                           1.1473033901948098E-7,
                                                           -6.247058281310054E-8,
                                                           1.1473033901948098E-7,
                                                           8.460025615260552E-6,
                                                           8.362216167123803E-6,
                                                           -6.247058281310054E-8,
                                                           8.362216167123803E-6,
                                                           1.7501257498657038E-5));
      properties.get("l_finger_1_joint_2").put("offsetFromParentJoint", new Vector3D(-4.999999999776676E-7, 0.04998000000000003, -0.02804000000000004));
      properties.get("l_finger_1_joint_2").put("positionLowerLimit", 0.0);
      properties.get("l_finger_1_joint_2").put("positionUpperLimit", 1.5708);
      properties.get("l_finger_1_joint_2").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_1_joint_2").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_1_joint_2").put("effortLowerLimit", -60.0);
      properties.get("l_finger_1_joint_2").put("effortUpperLimit", 60.0);
      properties.get("l_finger_1_joint_2").put("kpPositionLimit", 100.0);
      properties.get("l_finger_1_joint_2").put("kdPositionLimit", 20.0);
      properties.get("l_finger_1_joint_2").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_1_joint_2").put("axis", new Vector3D(0.9999999999529641, 7.346409999654455E-6, -6.332649999702139E-6));
      properties.get("l_finger_1_joint_2").put("damping", 1.0);
      properties.get("l_finger_1_joint_2").put("stiction", 0.0);
      properties.put("l_finger_1_joint_3", new HashMap<>());
      properties.put("l_finger_1_link_3", new HashMap<>());
      properties.get("l_finger_1_link_3").put("mass", 0.03354);
      properties.get("l_finger_1_link_3").put("centerOfMass", new Vector3D(1.9994207213847587E-4, 0.016260385778610654, 4.7705083434092173E-4));
      properties.get("l_finger_1_link_3").put("inertia",
                                              new Matrix3D(1.2641299894149845E-5,
                                                           2.5190379789924503E-9,
                                                           -1.1909294242819402E-8,
                                                           2.5190379789924495E-9,
                                                           3.0529472139062597E-6,
                                                           9.068601950811619E-7,
                                                           -1.1909294242819402E-8,
                                                           9.068601950811619E-7,
                                                           1.4278362891943894E-5));
      properties.get("l_finger_1_joint_3").put("offsetFromParentJoint", new Vector3D(-4.0000000001082607E-7, 0.03383000000000005, -0.019404999999999954));
      properties.get("l_finger_1_joint_3").put("positionLowerLimit", -0.6632);
      properties.get("l_finger_1_joint_3").put("positionUpperLimit", 1.0471);
      properties.get("l_finger_1_joint_3").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_1_joint_3").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_1_joint_3").put("effortLowerLimit", -60.0);
      properties.get("l_finger_1_joint_3").put("effortUpperLimit", 60.0);
      properties.get("l_finger_1_joint_3").put("kpPositionLimit", 100.0);
      properties.get("l_finger_1_joint_3").put("kdPositionLimit", 20.0);
      properties.get("l_finger_1_joint_3").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_1_joint_3").put("axis", new Vector3D(0.9999999999529641, 7.346409999654455E-6, -6.332649999702139E-6));
      properties.get("l_finger_1_joint_3").put("damping", 1.0);
      properties.get("l_finger_1_joint_3").put("stiction", 0.0);
      properties.put("l_palm_finger_2_joint", new HashMap<>());
      properties.put("l_finger_2_link_0", new HashMap<>());
      properties.get("l_finger_2_link_0").put("mass", 0.1);
      properties.get("l_finger_2_link_0").put("centerOfMass", new Vector3D(3.673203938689571E-8, -0.00999999682924925, 7.963269158366412E-6));
      properties.get("l_finger_2_link_0").put("inertia",
                                              new Matrix3D(2.499999999990555E-5,
                                                           2.5712419418057096E-11,
                                                           -2.0475498146155928E-14,
                                                           2.5712419418057096E-11,
                                                           1.800000443905034E-5,
                                                           5.574286643388569E-9,
                                                           -2.0475498146143003E-14,
                                                           5.574286643388572E-9,
                                                           2.4999995561044105E-5));
      properties.get("l_palm_finger_2_joint").put("offsetFromParentJoint", new Vector3D(0.03421000000000002, 0.21040000000000003, -0.04550000000000004));
      properties.get("l_palm_finger_2_joint").put("positionLowerLimit", -0.2967);
      properties.get("l_palm_finger_2_joint").put("positionUpperLimit", 0.2967);
      properties.get("l_palm_finger_2_joint").put("velocityLowerLimit", -Infinity);
      properties.get("l_palm_finger_2_joint").put("velocityUpperLimit", Infinity);
      properties.get("l_palm_finger_2_joint").put("effortLowerLimit", -60.0);
      properties.get("l_palm_finger_2_joint").put("effortUpperLimit", 60.0);
      properties.get("l_palm_finger_2_joint").put("kpPositionLimit", 100.0);
      properties.get("l_palm_finger_2_joint").put("kdPositionLimit", 20.0);
      properties.get("l_palm_finger_2_joint").put("kpVelocityLimit", 0.0);
      properties.get("l_palm_finger_2_joint").put("axis", new Vector3D(6.326787993849492E-6, 7.96326747494098E-4, 0.9999996829117912));
      properties.get("l_palm_finger_2_joint").put("damping", 1.0);
      properties.get("l_palm_finger_2_joint").put("stiction", 0.0);
      properties.put("l_finger_2_joint_1", new HashMap<>());
      properties.put("l_finger_2_link_1", new HashMap<>());
      properties.get("l_finger_2_link_1").put("mass", 0.047);
      properties.get("l_finger_2_link_1").put("centerOfMass", new Vector3D(-1.6047369155512067E-7, 0.02818764799107845, -0.015522451504362931));
      properties.get("l_finger_2_link_1").put("inertia",
                                              new Matrix3D(6.0619200010434345E-5,
                                                           -8.326117546483128E-10,
                                                           3.974703474676766E-10,
                                                           -8.326117546483127E-10,
                                                           2.0475715789274972E-5,
                                                           2.6077577618842952E-5,
                                                           3.974703474676766E-10,
                                                           2.607757761884295E-5,
                                                           4.886290420029068E-5));
      properties.get("l_finger_2_joint_1").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("l_finger_2_joint_1").put("positionLowerLimit", 0.0);
      properties.get("l_finger_2_joint_1").put("positionUpperLimit", 1.2217);
      properties.get("l_finger_2_joint_1").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_2_joint_1").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_2_joint_1").put("effortLowerLimit", -60.0);
      properties.get("l_finger_2_joint_1").put("effortUpperLimit", 60.0);
      properties.get("l_finger_2_joint_1").put("kpPositionLimit", 100.0);
      properties.get("l_finger_2_joint_1").put("kdPositionLimit", 20.0);
      properties.get("l_finger_2_joint_1").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_2_joint_1").put("axis", new Vector3D(0.9999999999529641, 7.346409999654455E-6, -6.332649999702139E-6));
      properties.get("l_finger_2_joint_1").put("damping", 1.0);
      properties.get("l_finger_2_joint_1").put("stiction", 0.0);
      properties.put("l_finger_2_joint_2", new HashMap<>());
      properties.put("l_finger_2_link_2", new HashMap<>());
      properties.get("l_finger_2_link_2").put("mass", 0.03284);
      properties.get("l_finger_2_link_2").put("centerOfMass", new Vector3D(1.797414813764432E-4, 0.019035900919603402, -0.010365995812217704));
      properties.get("l_finger_2_link_2").put("inertia",
                                              new Matrix3D(1.986349688608241E-5,
                                                           1.1473033901948098E-7,
                                                           -6.247058281310054E-8,
                                                           1.1473033901948098E-7,
                                                           8.460025615260552E-6,
                                                           8.362216167123803E-6,
                                                           -6.247058281310054E-8,
                                                           8.362216167123803E-6,
                                                           1.7501257498657038E-5));
      properties.get("l_finger_2_joint_2").put("offsetFromParentJoint", new Vector3D(-1.0000000000126629E-6, 0.04997999999999974, -0.028039999999999985));
      properties.get("l_finger_2_joint_2").put("positionLowerLimit", 0.0);
      properties.get("l_finger_2_joint_2").put("positionUpperLimit", 1.5708);
      properties.get("l_finger_2_joint_2").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_2_joint_2").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_2_joint_2").put("effortLowerLimit", -60.0);
      properties.get("l_finger_2_joint_2").put("effortUpperLimit", 60.0);
      properties.get("l_finger_2_joint_2").put("kpPositionLimit", 100.0);
      properties.get("l_finger_2_joint_2").put("kdPositionLimit", 20.0);
      properties.get("l_finger_2_joint_2").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_2_joint_2").put("axis", new Vector3D(0.9999999999529641, 7.346409999654455E-6, -6.332649999702139E-6));
      properties.get("l_finger_2_joint_2").put("damping", 1.0);
      properties.get("l_finger_2_joint_2").put("stiction", 0.0);
      properties.put("l_finger_2_joint_3", new HashMap<>());
      properties.put("l_finger_2_link_3", new HashMap<>());
      properties.get("l_finger_2_link_3").put("mass", 0.03354);
      properties.get("l_finger_2_link_3").put("centerOfMass", new Vector3D(1.9994207213847587E-4, 0.016260385778610654, 4.7705083434092173E-4));
      properties.get("l_finger_2_link_3").put("inertia",
                                              new Matrix3D(1.2641299894149845E-5,
                                                           2.5190379789924503E-9,
                                                           -1.1909294242819402E-8,
                                                           2.5190379789924495E-9,
                                                           3.0529472139062597E-6,
                                                           9.068601950811619E-7,
                                                           -1.1909294242819402E-8,
                                                           9.068601950811619E-7,
                                                           1.4278362891943894E-5));
      properties.get("l_finger_2_joint_3").put("offsetFromParentJoint", new Vector3D(3.014749077955665E-18, 0.03383000000000001, -0.01940499999999997));
      properties.get("l_finger_2_joint_3").put("positionLowerLimit", -0.6632);
      properties.get("l_finger_2_joint_3").put("positionUpperLimit", 1.0471);
      properties.get("l_finger_2_joint_3").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_2_joint_3").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_2_joint_3").put("effortLowerLimit", -60.0);
      properties.get("l_finger_2_joint_3").put("effortUpperLimit", 60.0);
      properties.get("l_finger_2_joint_3").put("kpPositionLimit", 100.0);
      properties.get("l_finger_2_joint_3").put("kdPositionLimit", 20.0);
      properties.get("l_finger_2_joint_3").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_2_joint_3").put("axis", new Vector3D(0.9999999999529641, 7.346409999654455E-6, -6.332649999702139E-6));
      properties.get("l_finger_2_joint_3").put("damping", 1.0);
      properties.get("l_finger_2_joint_3").put("stiction", 0.0);
      properties.put("l_palm_finger_middle_joint", new HashMap<>());
      properties.put("l_finger_middle_link_0", new HashMap<>());
      properties.get("l_finger_middle_link_0").put("mass", 0.1);
      properties.get("l_finger_middle_link_0").put("centerOfMass", new Vector3D(3.673203938689571E-8, -0.00999999682924925, -7.963269158366412E-6));
      properties.get("l_finger_middle_link_0").put("inertia",
                                                   new Matrix3D(2.499999999990555E-5,
                                                                2.571241941805711E-11,
                                                                2.0475498146117154E-14,
                                                                2.571241941805711E-11,
                                                                1.8000004439050344E-5,
                                                                -5.5742866433885716E-9,
                                                                2.0475498146168852E-14,
                                                                -5.5742866433885716E-9,
                                                                2.4999995561044105E-5));
      properties.get("l_palm_finger_middle_joint").put("offsetFromParentJoint", new Vector3D(-0.001789999999999986, 0.21040000000000003, 0.045499999999999985));
      properties.get("l_palm_finger_middle_joint").put("positionLowerLimit", -Infinity);
      properties.get("l_palm_finger_middle_joint").put("positionUpperLimit", Infinity);
      properties.get("l_palm_finger_middle_joint").put("velocityLowerLimit", -Infinity);
      properties.get("l_palm_finger_middle_joint").put("velocityUpperLimit", Infinity);
      properties.get("l_palm_finger_middle_joint").put("effortLowerLimit", -60.0);
      properties.get("l_palm_finger_middle_joint").put("effortUpperLimit", 60.0);
      properties.get("l_palm_finger_middle_joint").put("kpPositionLimit", 0.0);
      properties.get("l_palm_finger_middle_joint").put("kdPositionLimit", 0.0);
      properties.get("l_palm_finger_middle_joint").put("kpVelocityLimit", 0.0);
      properties.get("l_palm_finger_middle_joint").put("axis", new Vector3D(-6.326787993849492E-6, 7.96326747494098E-4, -0.9999996829117912));
      properties.get("l_palm_finger_middle_joint").put("damping", 1.0);
      properties.get("l_palm_finger_middle_joint").put("stiction", 0.0);
      properties.put("l_finger_middle_joint_1", new HashMap<>());
      properties.put("l_finger_middle_link_1", new HashMap<>());
      properties.get("l_finger_middle_link_1").put("mass", 0.047);
      properties.get("l_finger_middle_link_1").put("centerOfMass", new Vector3D(-5.47369155593376E-9, 0.02818764799164796, 0.015522451504157278));
      properties.get("l_finger_middle_link_1").put("inertia",
                                                   new Matrix3D(6.06191999990733E-5,
                                                                1.0584079131103385E-9,
                                                                5.579056583633519E-10,
                                                                1.0584079131103385E-9,
                                                                2.047571580317117E-5,
                                                                -2.6077577617841644E-5,
                                                                5.579056583633521E-10,
                                                                -2.607757761784164E-5,
                                                                4.886290419775551E-5));
      properties.get("l_finger_middle_joint_1").put("offsetFromParentJoint", new Vector3D(-0.0, 0.0, 0.0));
      properties.get("l_finger_middle_joint_1").put("positionLowerLimit", 0.0);
      properties.get("l_finger_middle_joint_1").put("positionUpperLimit", 1.2217);
      properties.get("l_finger_middle_joint_1").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_middle_joint_1").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_middle_joint_1").put("effortLowerLimit", -60.0);
      properties.get("l_finger_middle_joint_1").put("effortUpperLimit", 60.0);
      properties.get("l_finger_middle_joint_1").put("kpPositionLimit", 100.0);
      properties.get("l_finger_middle_joint_1").put("kdPositionLimit", 20.0);
      properties.get("l_finger_middle_joint_1").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_middle_joint_1").put("axis", new Vector3D(-0.999999999979986, 0.0, 6.326789999873376E-6));
      properties.get("l_finger_middle_joint_1").put("damping", 1.0);
      properties.get("l_finger_middle_joint_1").put("stiction", 0.0);
      properties.put("l_finger_middle_joint_2", new HashMap<>());
      properties.put("l_finger_middle_link_2", new HashMap<>());
      properties.get("l_finger_middle_link_2").put("mass", 0.03284);
      properties.get("l_finger_middle_link_2").put("centerOfMass", new Vector3D(-1.7988260063061753E-4, 0.019035899360977188, 0.010365996226544681));
      properties.get("l_finger_middle_link_2").put("inertia",
                                                   new Matrix3D(1.9863498586639554E-5,
                                                                -1.1465075589699571E-7,
                                                                -6.240354687389753E-8,
                                                                -1.1465075589699573E-7,
                                                                8.460023627654863E-6,
                                                                -8.362216444465822E-6,
                                                                -6.240354687389753E-8,
                                                                -8.362216444465819E-6,
                                                                1.750125778570558E-5));
      properties.get("l_finger_middle_joint_2").put("offsetFromParentJoint", new Vector3D(5.122938644799754E-17, 0.04997999999999988, 0.028040000000000002));
      properties.get("l_finger_middle_joint_2").put("positionLowerLimit", 0.0);
      properties.get("l_finger_middle_joint_2").put("positionUpperLimit", 1.5708);
      properties.get("l_finger_middle_joint_2").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_middle_joint_2").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_middle_joint_2").put("effortLowerLimit", -60.0);
      properties.get("l_finger_middle_joint_2").put("effortUpperLimit", 60.0);
      properties.get("l_finger_middle_joint_2").put("kpPositionLimit", 100.0);
      properties.get("l_finger_middle_joint_2").put("kdPositionLimit", 20.0);
      properties.get("l_finger_middle_joint_2").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_middle_joint_2").put("axis", new Vector3D(-0.999999999979986, -2.77555999994445E-17, 6.326789999873376E-6));
      properties.get("l_finger_middle_joint_2").put("damping", 1.0);
      properties.get("l_finger_middle_joint_2").put("stiction", 0.0);
      properties.put("l_finger_middle_joint_3", new HashMap<>());
      properties.put("l_finger_middle_link_3", new HashMap<>());
      properties.get("l_finger_middle_link_3").put("mass", 0.03354);
      properties.get("l_finger_middle_link_3").put("centerOfMass", new Vector3D(-2.0006282785347198E-4, 0.016260384308887984, -4.7705030361663007E-4));
      properties.get("l_finger_middle_link_3").put("inertia",
                                                   new Matrix3D(1.2641299867496634E-5,
                                                                -2.4509841798737985E-9,
                                                                -1.1898286066816504E-8,
                                                                -2.4509841798737985E-9,
                                                                3.0529471773839354E-6,
                                                                -9.068602759622438E-7,
                                                                -1.1898286066816505E-8,
                                                                -9.068602759622438E-7,
                                                                1.4278362955119426E-5));
      properties.get("l_finger_middle_joint_3").put("offsetFromParentJoint", new Vector3D(1.0000000000064088E-6, 0.03383000000000013, 0.01940499999999993));
      properties.get("l_finger_middle_joint_3").put("positionLowerLimit", -0.6632);
      properties.get("l_finger_middle_joint_3").put("positionUpperLimit", 1.0471);
      properties.get("l_finger_middle_joint_3").put("velocityLowerLimit", -Infinity);
      properties.get("l_finger_middle_joint_3").put("velocityUpperLimit", Infinity);
      properties.get("l_finger_middle_joint_3").put("effortLowerLimit", -60.0);
      properties.get("l_finger_middle_joint_3").put("effortUpperLimit", 60.0);
      properties.get("l_finger_middle_joint_3").put("kpPositionLimit", 100.0);
      properties.get("l_finger_middle_joint_3").put("kdPositionLimit", 20.0);
      properties.get("l_finger_middle_joint_3").put("kpVelocityLimit", 0.0);
      properties.get("l_finger_middle_joint_3").put("axis", new Vector3D(-0.999999999979986, -5.5511199998889E-17, 6.326789999873376E-6));
      properties.get("l_finger_middle_joint_3").put("damping", 1.0);
      properties.get("l_finger_middle_joint_3").put("stiction", 0.0);
      properties.put("neck_ry", new HashMap<>());
      properties.put("head", new HashMap<>());
      properties.get("head").put("mass", 1.41991);
      properties.get("head").put("centerOfMass", new Vector3D(-0.0754942, 3.38765E-5, 0.0277411));
      properties.get("head")
                .put("inertia", new Matrix3D(0.00397627, -1.51324E-6, -8.92818E-4, -1.51324E-6, 0.00412515, -6.83342E-7, -8.92818E-4, -6.83342E-7, 0.00353178));
      properties.get("neck_ry").put("offsetFromParentJoint", new Vector3D(0.288895, 0.0, 0.6215));
      properties.get("neck_ry").put("positionLowerLimit", -0.602139);
      properties.get("neck_ry").put("positionUpperLimit", 1.14319);
      properties.get("neck_ry").put("velocityLowerLimit", -6.28);
      properties.get("neck_ry").put("velocityUpperLimit", 6.28);
      properties.get("neck_ry").put("effortLowerLimit", -25.0);
      properties.get("neck_ry").put("effortUpperLimit", 25.0);
      properties.get("neck_ry").put("kpPositionLimit", 100.0);
      properties.get("neck_ry").put("kdPositionLimit", 20.0);
      properties.get("neck_ry").put("kpVelocityLimit", 500.0);
      properties.get("neck_ry").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("neck_ry").put("damping", 0.1);
      properties.get("neck_ry").put("stiction", 0.0);
      properties.put("hokuyo_joint", new HashMap<>());
      properties.put("hokuyo_link", new HashMap<>());
      properties.get("hokuyo_link").put("mass", 0.057664);
      properties.get("hokuyo_link").put("centerOfMass", new Vector3D(0.0324349, 4.084E-4, -0.0041783));
      properties.get("hokuyo_link").put("inertia",
                                        new Matrix3D(3.8183E-5, 4.9927E-8, 1.1003E-5, 4.9927E-8, 4.3437E-5, -9.8165E-9, 1.1003E-5, -9.8165E-9, 4.2686E-5));
      properties.get("hokuyo_joint").put("offsetFromParentJoint", new Vector3D(-0.08460000000000001, 0.0, 0.08799999999999997));
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
      properties.put("r_arm_shz", new HashMap<>());
      properties.put("r_clav", new HashMap<>());
      properties.get("r_clav").put("mass", 4.466);
      properties.get("r_clav").put("centerOfMass", new Vector3D(0.0, 0.0, -0.084));
      properties.get("r_clav").put("inertia", new Matrix3D(0.011, 0.0, 0.0, 0.0, 0.009, 0.004, 0.0, 0.004, 0.004));
      properties.get("r_arm_shz").put("offsetFromParentJoint", new Vector3D(0.134895, -0.2256, 0.4776));
      properties.get("r_arm_shz").put("positionLowerLimit", -0.785398);
      properties.get("r_arm_shz").put("positionUpperLimit", 1.5708);
      properties.get("r_arm_shz").put("velocityLowerLimit", -12.0);
      properties.get("r_arm_shz").put("velocityUpperLimit", 12.0);
      properties.get("r_arm_shz").put("effortLowerLimit", -87.0);
      properties.get("r_arm_shz").put("effortUpperLimit", 87.0);
      properties.get("r_arm_shz").put("kpPositionLimit", 100.0);
      properties.get("r_arm_shz").put("kdPositionLimit", 20.0);
      properties.get("r_arm_shz").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_shz").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("r_arm_shz").put("damping", 0.1);
      properties.get("r_arm_shz").put("stiction", 0.0);
      properties.put("r_arm_shx", new HashMap<>());
      properties.put("r_scap", new HashMap<>());
      properties.get("r_scap").put("mass", 3.899);
      properties.get("r_scap").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("r_scap").put("inertia", new Matrix3D(0.00319, 0.0, 0.0, 0.0, 0.00583, 0.0, 0.0, 0.0, 0.00583));
      properties.get("r_arm_shx").put("offsetFromParentJoint", new Vector3D(0.0, -0.11000000000000001, -0.245));
      properties.get("r_arm_shx").put("positionLowerLimit", -1.5708);
      properties.get("r_arm_shx").put("positionUpperLimit", 1.5708);
      properties.get("r_arm_shx").put("velocityLowerLimit", -12.0);
      properties.get("r_arm_shx").put("velocityUpperLimit", 12.0);
      properties.get("r_arm_shx").put("effortLowerLimit", -99.0);
      properties.get("r_arm_shx").put("effortUpperLimit", 99.0);
      properties.get("r_arm_shx").put("kpPositionLimit", 100.0);
      properties.get("r_arm_shx").put("kdPositionLimit", 20.0);
      properties.get("r_arm_shx").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_shx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("r_arm_shx").put("damping", 0.1);
      properties.get("r_arm_shx").put("stiction", 0.0);
      properties.put("r_arm_ely", new HashMap<>());
      properties.put("r_uarm", new HashMap<>());
      properties.get("r_uarm").put("mass", 4.386);
      properties.get("r_uarm").put("centerOfMass", new Vector3D(0.0, 0.065, 0.0));
      properties.get("r_uarm").put("inertia", new Matrix3D(0.00656, 0.0, 0.0, 0.0, 0.00358, 0.0, 0.0, 0.0, 0.00656));
      properties.get("r_arm_ely").put("offsetFromParentJoint", new Vector3D(0.0, -0.18699999999999994, -0.016000000000000014));
      properties.get("r_arm_ely").put("positionLowerLimit", 0.0);
      properties.get("r_arm_ely").put("positionUpperLimit", 3.14159);
      properties.get("r_arm_ely").put("velocityLowerLimit", -12.0);
      properties.get("r_arm_ely").put("velocityUpperLimit", 12.0);
      properties.get("r_arm_ely").put("effortLowerLimit", -63.0);
      properties.get("r_arm_ely").put("effortUpperLimit", 63.0);
      properties.get("r_arm_ely").put("kpPositionLimit", 100.0);
      properties.get("r_arm_ely").put("kdPositionLimit", 20.0);
      properties.get("r_arm_ely").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_ely").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("r_arm_ely").put("damping", 0.1);
      properties.get("r_arm_ely").put("stiction", 0.0);
      properties.put("r_arm_elx", new HashMap<>());
      properties.put("r_larm", new HashMap<>());
      properties.get("r_larm").put("mass", 3.248);
      properties.get("r_larm").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("r_larm").put("inertia", new Matrix3D(0.00265, 0.0, 0.0, 0.0, 0.00446, 0.0, 0.0, 0.0, 0.00446));
      properties.get("r_arm_elx").put("offsetFromParentJoint", new Vector3D(0.0, -0.119, 0.009200000000000041));
      properties.get("r_arm_elx").put("positionLowerLimit", -2.35619);
      properties.get("r_arm_elx").put("positionUpperLimit", 0.0);
      properties.get("r_arm_elx").put("velocityLowerLimit", -12.0);
      properties.get("r_arm_elx").put("velocityUpperLimit", 12.0);
      properties.get("r_arm_elx").put("effortLowerLimit", -112.0);
      properties.get("r_arm_elx").put("effortUpperLimit", 112.0);
      properties.get("r_arm_elx").put("kpPositionLimit", 100.0);
      properties.get("r_arm_elx").put("kdPositionLimit", 20.0);
      properties.get("r_arm_elx").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_elx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("r_arm_elx").put("damping", 0.1);
      properties.get("r_arm_elx").put("stiction", 0.0);
      properties.put("r_arm_wry", new HashMap<>());
      properties.put("r_ufarm", new HashMap<>());
      properties.get("r_ufarm").put("mass", 2.4798);
      properties.get("r_ufarm").put("centerOfMass", new Vector3D(1.5E-4, 0.08296, 3.7E-4));
      properties.get("r_ufarm").put("inertia", new Matrix3D(0.012731, 0.0, 0.0, 0.0, 0.002857, 0.0, 0.0, 0.0, 0.011948));
      properties.get("r_arm_wry").put("offsetFromParentJoint", new Vector3D(0.0, -0.2995500000000001, -0.009209999999999996));
      properties.get("r_arm_wry").put("positionLowerLimit", -3.011);
      properties.get("r_arm_wry").put("positionUpperLimit", 3.011);
      properties.get("r_arm_wry").put("velocityLowerLimit", -10.0);
      properties.get("r_arm_wry").put("velocityUpperLimit", 10.0);
      properties.get("r_arm_wry").put("effortLowerLimit", -25.0);
      properties.get("r_arm_wry").put("effortUpperLimit", 25.0);
      properties.get("r_arm_wry").put("kpPositionLimit", 100.0);
      properties.get("r_arm_wry").put("kdPositionLimit", 20.0);
      properties.get("r_arm_wry").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_wry").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("r_arm_wry").put("damping", 0.1);
      properties.get("r_arm_wry").put("stiction", 0.0);
      properties.put("r_arm_wrx", new HashMap<>());
      properties.put("r_lfarm", new HashMap<>());
      properties.get("r_lfarm").put("mass", 0.648);
      properties.get("r_lfarm").put("centerOfMass", new Vector3D(1.7E-4, -0.02515, 0.00163));
      properties.get("r_lfarm").put("inertia", new Matrix3D(7.64E-4, 0.0, 0.0, 0.0, 4.29E-4, 0.0, 0.0, 0.0, 8.25E-4));
      properties.get("r_arm_wrx").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("r_arm_wrx").put("positionLowerLimit", -1.7628);
      properties.get("r_arm_wrx").put("positionUpperLimit", 1.7628);
      properties.get("r_arm_wrx").put("velocityLowerLimit", -10.0);
      properties.get("r_arm_wrx").put("velocityUpperLimit", 10.0);
      properties.get("r_arm_wrx").put("effortLowerLimit", -25.0);
      properties.get("r_arm_wrx").put("effortUpperLimit", 25.0);
      properties.get("r_arm_wrx").put("kpPositionLimit", 100.0);
      properties.get("r_arm_wrx").put("kdPositionLimit", 20.0);
      properties.get("r_arm_wrx").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_wrx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("r_arm_wrx").put("damping", 0.1);
      properties.get("r_arm_wrx").put("stiction", 0.0);
      properties.put("r_arm_wry2", new HashMap<>());
      properties.put("r_hand", new HashMap<>());
      properties.get("r_hand").put("mass", 1.8839);
      properties.get("r_hand").put("centerOfMass", new Vector3D(-0.00118561, -0.141218, 6.19884E-6));
      properties.get("r_hand")
                .put("inertia", new Matrix3D(0.00689651, -6.78926E-5, -1.57141E-8, -6.78926E-5, 0.00397853, -6.96335E-7, -1.57141E-8, -6.96335E-7, 0.00688905));
      properties.get("r_arm_wry2").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("r_arm_wry2").put("positionLowerLimit", -2.9671);
      properties.get("r_arm_wry2").put("positionUpperLimit", 2.9671);
      properties.get("r_arm_wry2").put("velocityLowerLimit", -10.0);
      properties.get("r_arm_wry2").put("velocityUpperLimit", 10.0);
      properties.get("r_arm_wry2").put("effortLowerLimit", -25.0);
      properties.get("r_arm_wry2").put("effortUpperLimit", 25.0);
      properties.get("r_arm_wry2").put("kpPositionLimit", 100.0);
      properties.get("r_arm_wry2").put("kdPositionLimit", 20.0);
      properties.get("r_arm_wry2").put("kpVelocityLimit", 500.0);
      properties.get("r_arm_wry2").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("r_arm_wry2").put("damping", 0.1);
      properties.get("r_arm_wry2").put("stiction", 0.0);
      properties.put("r_palm_finger_1_joint", new HashMap<>());
      properties.put("r_finger_1_link_0", new HashMap<>());
      properties.get("r_finger_1_link_0").put("mass", 0.1);
      properties.get("r_finger_1_link_0").put("centerOfMass", new Vector3D(-6.326792890642185E-8, 0.009999996829116571, 7.963269158366412E-6));
      properties.get("r_finger_1_link_0").put("inertia",
                                              new Matrix3D(2.4999999999719796E-5,
                                                           4.4287536191429364E-11,
                                                           3.526736828820462E-14,
                                                           4.428753619142938E-11,
                                                           1.8000004439236095E-5,
                                                           -5.574286643314611E-9,
                                                           3.526736828820462E-14,
                                                           -5.574286643314611E-9,
                                                           2.4999995561044105E-5));
      properties.get("r_palm_finger_1_joint").put("offsetFromParentJoint", new Vector3D(0.03421000000000002, -0.21040000000000003, -0.04550000000000004));
      properties.get("r_palm_finger_1_joint").put("positionLowerLimit", -0.2967);
      properties.get("r_palm_finger_1_joint").put("positionUpperLimit", 0.2967);
      properties.get("r_palm_finger_1_joint").put("velocityLowerLimit", -Infinity);
      properties.get("r_palm_finger_1_joint").put("velocityUpperLimit", Infinity);
      properties.get("r_palm_finger_1_joint").put("effortLowerLimit", -60.0);
      properties.get("r_palm_finger_1_joint").put("effortUpperLimit", 60.0);
      properties.get("r_palm_finger_1_joint").put("kpPositionLimit", 100.0);
      properties.get("r_palm_finger_1_joint").put("kdPositionLimit", 20.0);
      properties.get("r_palm_finger_1_joint").put("kpVelocityLimit", 0.0);
      properties.get("r_palm_finger_1_joint").put("axis", new Vector3D(6.326267994014398E-6, -7.963267474941006E-4, 0.9999996829117945));
      properties.get("r_palm_finger_1_joint").put("damping", 1.0);
      properties.get("r_palm_finger_1_joint").put("stiction", 0.0);
      properties.put("r_finger_1_joint_1", new HashMap<>());
      properties.put("r_finger_1_link_1", new HashMap<>());
      properties.get("r_finger_1_link_1").put("mass", 0.047);
      properties.get("r_finger_1_link_1").put("centerOfMass", new Vector3D(8.027214656517109E-8, -0.028187647991534195, -0.015522451504157278));
      properties.get("r_finger_1_link_1").put("inertia",
                                              new Matrix3D(6.061920000684515E-5,
                                                           -9.865434347538967E-10,
                                                           -4.4562641161364085E-10,
                                                           -9.865434347538967E-10,
                                                           2.0475715781825436E-5,
                                                           -2.607757760944979E-5,
                                                           -4.4562641161364085E-10,
                                                           -2.607757760944979E-5,
                                                           4.88629042113294E-5));
      properties.get("r_finger_1_joint_1").put("offsetFromParentJoint", new Vector3D(0.0, -0.0, 0.0));
      properties.get("r_finger_1_joint_1").put("positionLowerLimit", 0.0);
      properties.get("r_finger_1_joint_1").put("positionUpperLimit", 1.2217);
      properties.get("r_finger_1_joint_1").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_1_joint_1").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_1_joint_1").put("effortLowerLimit", -60.0);
      properties.get("r_finger_1_joint_1").put("effortUpperLimit", 60.0);
      properties.get("r_finger_1_joint_1").put("kpPositionLimit", 100.0);
      properties.get("r_finger_1_joint_1").put("kdPositionLimit", 20.0);
      properties.get("r_finger_1_joint_1").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_1_joint_1").put("axis", new Vector3D(-0.9999999999576259, -6.692819999716398E-6, 6.320939999732156E-6));
      properties.get("r_finger_1_joint_1").put("damping", 1.0);
      properties.get("r_finger_1_joint_1").put("stiction", 0.0);
      properties.put("r_finger_1_joint_2", new HashMap<>());
      properties.put("r_finger_1_link_2", new HashMap<>());
      properties.get("r_finger_1_link_2").put("mass", 0.03284);
      properties.get("r_finger_1_link_2").put("centerOfMass", new Vector3D(-1.7987652690034674E-4, -0.019035900494228906, -0.01036599425086113));
      properties.get("r_finger_1_link_2").put("inertia",
                                              new Matrix3D(1.9863498513248567E-5,
                                                           1.1463076896000476E-7,
                                                           6.243026784582213E-8,
                                                           1.1463076896000474E-7,
                                                           8.46002507006287E-6,
                                                           -8.362217309156877E-6,
                                                           6.243026784582213E-8,
                                                           -8.362217309156874E-6,
                                                           1.7501256416688562E-5));
      properties.get("r_finger_1_joint_2").put("offsetFromParentJoint", new Vector3D(-3.961228987173541E-18, -0.04997999999999999, -0.02803999999999998));
      properties.get("r_finger_1_joint_2").put("positionLowerLimit", 0.0);
      properties.get("r_finger_1_joint_2").put("positionUpperLimit", 1.5708);
      properties.get("r_finger_1_joint_2").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_1_joint_2").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_1_joint_2").put("effortLowerLimit", -60.0);
      properties.get("r_finger_1_joint_2").put("effortUpperLimit", 60.0);
      properties.get("r_finger_1_joint_2").put("kpPositionLimit", 100.0);
      properties.get("r_finger_1_joint_2").put("kdPositionLimit", 20.0);
      properties.get("r_finger_1_joint_2").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_1_joint_2").put("axis", new Vector3D(-0.9999999999576259, -6.692819999716398E-6, 6.320939999732156E-6));
      properties.get("r_finger_1_joint_2").put("damping", 1.0);
      properties.get("r_finger_1_joint_2").put("stiction", 0.0);
      properties.put("r_finger_1_joint_3", new HashMap<>());
      properties.put("r_finger_1_link_3", new HashMap<>());
      properties.get("r_finger_1_link_3").put("mass", 0.03354);
      properties.get("r_finger_1_link_3").put("centerOfMass", new Vector3D(-1.9989402374137025E-4, -0.016260386310679293, 4.770528343337863E-4));
      properties.get("r_finger_1_link_3").put("inertia",
                                              new Matrix3D(1.2641300119070603E-5,
                                                           2.5354892902699895E-9,
                                                           1.1928078521684928E-8,
                                                           2.5354892902699895E-9,
                                                           3.0529472272684657E-6,
                                                           -9.068601880703036E-7,
                                                           1.1928078521684928E-8,
                                                           -9.068601880703036E-7,
                                                           1.4278362653660927E-5));
      properties.get("r_finger_1_joint_3").put("offsetFromParentJoint", new Vector3D(-3.3714452399562916E-17, -0.03382999999999998, -0.01940500000000009));
      properties.get("r_finger_1_joint_3").put("positionLowerLimit", -0.6632);
      properties.get("r_finger_1_joint_3").put("positionUpperLimit", 1.0471);
      properties.get("r_finger_1_joint_3").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_1_joint_3").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_1_joint_3").put("effortLowerLimit", -60.0);
      properties.get("r_finger_1_joint_3").put("effortUpperLimit", 60.0);
      properties.get("r_finger_1_joint_3").put("kpPositionLimit", 100.0);
      properties.get("r_finger_1_joint_3").put("kdPositionLimit", 20.0);
      properties.get("r_finger_1_joint_3").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_1_joint_3").put("axis", new Vector3D(-0.9999999999576259, -6.692819999716398E-6, 6.320939999732156E-6));
      properties.get("r_finger_1_joint_3").put("damping", 1.0);
      properties.get("r_finger_1_joint_3").put("stiction", 0.0);
      properties.put("r_palm_finger_2_joint", new HashMap<>());
      properties.put("r_finger_2_link_0", new HashMap<>());
      properties.get("r_finger_2_link_0").put("mass", 0.1);
      properties.get("r_finger_2_link_0").put("centerOfMass", new Vector3D(-6.326792890642185E-8, 0.009999996829116571, 7.963269158366412E-6));
      properties.get("r_finger_2_link_0").put("inertia",
                                              new Matrix3D(2.4999999999719796E-5,
                                                           4.4287536191429364E-11,
                                                           3.526736828820462E-14,
                                                           4.428753619142938E-11,
                                                           1.8000004439236095E-5,
                                                           -5.574286643314611E-9,
                                                           3.526736828820462E-14,
                                                           -5.574286643314611E-9,
                                                           2.4999995561044105E-5));
      properties.get("r_palm_finger_2_joint").put("offsetFromParentJoint", new Vector3D(-0.037790299999999985, -0.21040000000000003, -0.04550000000000004));
      properties.get("r_palm_finger_2_joint").put("positionLowerLimit", -0.2967);
      properties.get("r_palm_finger_2_joint").put("positionUpperLimit", 0.2967);
      properties.get("r_palm_finger_2_joint").put("velocityLowerLimit", -Infinity);
      properties.get("r_palm_finger_2_joint").put("velocityUpperLimit", Infinity);
      properties.get("r_palm_finger_2_joint").put("effortLowerLimit", -60.0);
      properties.get("r_palm_finger_2_joint").put("effortUpperLimit", 60.0);
      properties.get("r_palm_finger_2_joint").put("kpPositionLimit", 100.0);
      properties.get("r_palm_finger_2_joint").put("kdPositionLimit", 20.0);
      properties.get("r_palm_finger_2_joint").put("kpVelocityLimit", 0.0);
      properties.get("r_palm_finger_2_joint").put("axis", new Vector3D(6.326267994014398E-6, -7.963267474941006E-4, 0.9999996829117945));
      properties.get("r_palm_finger_2_joint").put("damping", 1.0);
      properties.get("r_palm_finger_2_joint").put("stiction", 0.0);
      properties.put("r_finger_2_joint_1", new HashMap<>());
      properties.put("r_finger_2_link_1", new HashMap<>());
      properties.get("r_finger_2_link_1").put("mass", 0.047);
      properties.get("r_finger_2_link_1").put("centerOfMass", new Vector3D(8.027214656517109E-8, -0.028187647991534195, -0.015522451504157278));
      properties.get("r_finger_2_link_1").put("inertia",
                                              new Matrix3D(6.061920000684515E-5,
                                                           -9.865434347538967E-10,
                                                           -4.4562641161364085E-10,
                                                           -9.865434347538967E-10,
                                                           2.0475715781825436E-5,
                                                           -2.607757760944979E-5,
                                                           -4.4562641161364085E-10,
                                                           -2.607757760944979E-5,
                                                           4.88629042113294E-5));
      properties.get("r_finger_2_joint_1").put("offsetFromParentJoint", new Vector3D(0.0, -0.0, 0.0));
      properties.get("r_finger_2_joint_1").put("positionLowerLimit", 0.0);
      properties.get("r_finger_2_joint_1").put("positionUpperLimit", 1.2217);
      properties.get("r_finger_2_joint_1").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_2_joint_1").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_2_joint_1").put("effortLowerLimit", -60.0);
      properties.get("r_finger_2_joint_1").put("effortUpperLimit", 60.0);
      properties.get("r_finger_2_joint_1").put("kpPositionLimit", 100.0);
      properties.get("r_finger_2_joint_1").put("kdPositionLimit", 20.0);
      properties.get("r_finger_2_joint_1").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_2_joint_1").put("axis", new Vector3D(-0.9999999999576259, -6.692819999716398E-6, 6.320939999732156E-6));
      properties.get("r_finger_2_joint_1").put("damping", 1.0);
      properties.get("r_finger_2_joint_1").put("stiction", 0.0);
      properties.put("r_finger_2_joint_2", new HashMap<>());
      properties.put("r_finger_2_link_2", new HashMap<>());
      properties.get("r_finger_2_link_2").put("mass", 0.03284);
      properties.get("r_finger_2_link_2").put("centerOfMass", new Vector3D(-1.7987652690034674E-4, -0.019035900494228906, -0.01036599425086113));
      properties.get("r_finger_2_link_2").put("inertia",
                                              new Matrix3D(1.9863498513248567E-5,
                                                           1.1463076896000476E-7,
                                                           6.243026784582213E-8,
                                                           1.1463076896000474E-7,
                                                           8.46002507006287E-6,
                                                           -8.362217309156877E-6,
                                                           6.243026784582213E-8,
                                                           -8.362217309156874E-6,
                                                           1.7501256416688562E-5));
      properties.get("r_finger_2_joint_2").put("offsetFromParentJoint", new Vector3D(1.9999999998204655E-7, -0.04998000000000006, -0.028040000000000027));
      properties.get("r_finger_2_joint_2").put("positionLowerLimit", 0.0);
      properties.get("r_finger_2_joint_2").put("positionUpperLimit", 1.5708);
      properties.get("r_finger_2_joint_2").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_2_joint_2").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_2_joint_2").put("effortLowerLimit", -60.0);
      properties.get("r_finger_2_joint_2").put("effortUpperLimit", 60.0);
      properties.get("r_finger_2_joint_2").put("kpPositionLimit", 100.0);
      properties.get("r_finger_2_joint_2").put("kdPositionLimit", 20.0);
      properties.get("r_finger_2_joint_2").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_2_joint_2").put("axis", new Vector3D(-0.9999999999576259, -6.692819999716398E-6, 6.320939999732156E-6));
      properties.get("r_finger_2_joint_2").put("damping", 1.0);
      properties.get("r_finger_2_joint_2").put("stiction", 0.0);
      properties.put("r_finger_2_joint_3", new HashMap<>());
      properties.put("r_finger_2_link_3", new HashMap<>());
      properties.get("r_finger_2_link_3").put("mass", 0.03354);
      properties.get("r_finger_2_link_3").put("centerOfMass", new Vector3D(-1.9989402374137025E-4, -0.016260386310679293, 4.770528343337863E-4));
      properties.get("r_finger_2_link_3").put("inertia",
                                              new Matrix3D(1.2641300119070603E-5,
                                                           2.5354892902699895E-9,
                                                           1.1928078521684928E-8,
                                                           2.5354892902699895E-9,
                                                           3.0529472272684657E-6,
                                                           -9.068601880703036E-7,
                                                           1.1928078521684928E-8,
                                                           -9.068601880703036E-7,
                                                           1.4278362653660927E-5));
      properties.get("r_finger_2_joint_3").put("offsetFromParentJoint", new Vector3D(1.0000000000291364E-7, -0.033830000000000165, -0.01940499999999996));
      properties.get("r_finger_2_joint_3").put("positionLowerLimit", -0.6632);
      properties.get("r_finger_2_joint_3").put("positionUpperLimit", 1.0471);
      properties.get("r_finger_2_joint_3").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_2_joint_3").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_2_joint_3").put("effortLowerLimit", -60.0);
      properties.get("r_finger_2_joint_3").put("effortUpperLimit", 60.0);
      properties.get("r_finger_2_joint_3").put("kpPositionLimit", 100.0);
      properties.get("r_finger_2_joint_3").put("kdPositionLimit", 20.0);
      properties.get("r_finger_2_joint_3").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_2_joint_3").put("axis", new Vector3D(-0.9999999999576259, -6.692819999716398E-6, 6.320939999732156E-6));
      properties.get("r_finger_2_joint_3").put("damping", 1.0);
      properties.get("r_finger_2_joint_3").put("stiction", 0.0);
      properties.put("r_palm_finger_middle_joint", new HashMap<>());
      properties.put("r_finger_middle_link_0", new HashMap<>());
      properties.get("r_finger_middle_link_0").put("mass", 0.1);
      properties.get("r_finger_middle_link_0").put("centerOfMass", new Vector3D(3.673203938689571E-8, 0.00999999682924925, -7.963269158366412E-6));
      properties.get("r_finger_middle_link_0").put("inertia",
                                                   new Matrix3D(2.4999999999905547E-5,
                                                                -2.5712419418057083E-11,
                                                                2.0475498146155928E-14,
                                                                -2.5712419418057083E-11,
                                                                1.800000443905034E-5,
                                                                5.574286643388574E-9,
                                                                2.0475498146143003E-14,
                                                                5.574286643388574E-9,
                                                                2.4999995561044105E-5));
      properties.get("r_palm_finger_middle_joint").put("offsetFromParentJoint",
                                                       new Vector3D(-0.001789999999999986, -0.21040000000000003, 0.045499999999999985));
      properties.get("r_palm_finger_middle_joint").put("positionLowerLimit", -Infinity);
      properties.get("r_palm_finger_middle_joint").put("positionUpperLimit", Infinity);
      properties.get("r_palm_finger_middle_joint").put("velocityLowerLimit", -Infinity);
      properties.get("r_palm_finger_middle_joint").put("velocityUpperLimit", Infinity);
      properties.get("r_palm_finger_middle_joint").put("effortLowerLimit", -60.0);
      properties.get("r_palm_finger_middle_joint").put("effortUpperLimit", 60.0);
      properties.get("r_palm_finger_middle_joint").put("kpPositionLimit", 0.0);
      properties.get("r_palm_finger_middle_joint").put("kdPositionLimit", 0.0);
      properties.get("r_palm_finger_middle_joint").put("kpVelocityLimit", 0.0);
      properties.get("r_palm_finger_middle_joint").put("axis", new Vector3D(-6.327307993684585E-6, -7.963267474940952E-4, -0.9999996829117879));
      properties.get("r_palm_finger_middle_joint").put("damping", 1.0);
      properties.get("r_palm_finger_middle_joint").put("stiction", 0.0);
      properties.put("r_finger_middle_joint_1", new HashMap<>());
      properties.put("r_finger_middle_link_1", new HashMap<>());
      properties.get("r_finger_middle_link_1").put("mass", 0.047);
      properties.get("r_finger_middle_link_1").put("centerOfMass", new Vector3D(-4.6604333352145056E-8, -0.028187647991496718, 0.015522451504362931));
      properties.get("r_finger_middle_link_1").put("inertia",
                                                   new Matrix3D(6.061920000397187E-5,
                                                                1.0239534311037176E-9,
                                                                -4.836843339505656E-10,
                                                                1.0239534311037174E-9,
                                                                2.0475715787856707E-5,
                                                                2.607757761125996E-5,
                                                                -4.836843339505654E-10,
                                                                2.607757761125996E-5,
                                                                4.8862904208171405E-5));
      properties.get("r_finger_middle_joint_1").put("offsetFromParentJoint", new Vector3D(0.0, -0.0, 0.0));
      properties.get("r_finger_middle_joint_1").put("positionLowerLimit", 0.0);
      properties.get("r_finger_middle_joint_1").put("positionUpperLimit", 1.2217);
      properties.get("r_finger_middle_joint_1").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_middle_joint_1").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_middle_joint_1").put("effortLowerLimit", -60.0);
      properties.get("r_finger_middle_joint_1").put("effortUpperLimit", 60.0);
      properties.get("r_finger_middle_joint_1").put("kpPositionLimit", 100.0);
      properties.get("r_finger_middle_joint_1").put("kdPositionLimit", 20.0);
      properties.get("r_finger_middle_joint_1").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_middle_joint_1").put("axis", new Vector3D(0.9999999999797724, -6.535899999867794E-7, -6.326789999872025E-6));
      properties.get("r_finger_middle_joint_1").put("damping", 1.0);
      properties.get("r_finger_middle_joint_1").put("stiction", 0.0);
      properties.put("r_finger_middle_joint_2", new HashMap<>());
      properties.put("r_finger_middle_link_2", new HashMap<>());
      properties.get("r_finger_middle_link_2").put("mass", 0.03284);
      properties.get("r_finger_middle_link_2").put("centerOfMass", new Vector3D(1.801186730861277E-4, -0.01903589797895351, 0.010365994665177057));
      properties.get("r_finger_middle_link_2").put("inertia",
                                                   new Matrix3D(1.986350142783164E-5,
                                                                -1.144906655149054E-7,
                                                                6.231885210966547E-8,
                                                                -1.144906655149054E-7,
                                                                8.460021868334035E-6,
                                                                8.362217917080904E-6,
                                                                6.231885210966547E-8,
                                                                8.3622179170809E-6,
                                                                1.750125670383432E-5));
      properties.get("r_finger_middle_joint_2").put("offsetFromParentJoint", new Vector3D(1.9906955091480766E-17, -0.04997999999999988, 0.02803999999999992));
      properties.get("r_finger_middle_joint_2").put("positionLowerLimit", 0.0);
      properties.get("r_finger_middle_joint_2").put("positionUpperLimit", 1.5708);
      properties.get("r_finger_middle_joint_2").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_middle_joint_2").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_middle_joint_2").put("effortLowerLimit", -60.0);
      properties.get("r_finger_middle_joint_2").put("effortUpperLimit", 60.0);
      properties.get("r_finger_middle_joint_2").put("kpPositionLimit", 100.0);
      properties.get("r_finger_middle_joint_2").put("kdPositionLimit", 20.0);
      properties.get("r_finger_middle_joint_2").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_middle_joint_2").put("axis", new Vector3D(0.9999999999797724, -6.535899999867794E-7, -6.326789999872025E-6));
      properties.get("r_finger_middle_joint_2").put("damping", 1.0);
      properties.get("r_finger_middle_joint_2").put("stiction", 0.0);
      properties.put("r_finger_middle_joint_3", new HashMap<>());
      properties.put("r_finger_middle_link_3", new HashMap<>());
      properties.get("r_finger_middle_link_3").put("mass", 0.03354);
      properties.get("r_finger_middle_link_3").put("centerOfMass", new Vector3D(1.999384723974703E-4, -0.01626038577976746, -4.7705230362249726E-4));
      properties.get("r_finger_middle_link_3").put("inertia",
                                                   new Matrix3D(1.2641300069189865E-5,
                                                                -2.512431907612802E-9,
                                                                1.192132607923154E-8,
                                                                -2.5124319076128012E-9,
                                                                3.0529472138867517E-6,
                                                                9.068602131901544E-7,
                                                                1.1921326079231539E-8,
                                                                9.068602131901544E-7,
                                                                1.4278362716923379E-5));
      properties.get("r_finger_middle_joint_3").put("offsetFromParentJoint", new Vector3D(9.999999999810156E-7, -0.03383000000000006, 0.019405000000000044));
      properties.get("r_finger_middle_joint_3").put("positionLowerLimit", -0.6632);
      properties.get("r_finger_middle_joint_3").put("positionUpperLimit", 1.0471);
      properties.get("r_finger_middle_joint_3").put("velocityLowerLimit", -Infinity);
      properties.get("r_finger_middle_joint_3").put("velocityUpperLimit", Infinity);
      properties.get("r_finger_middle_joint_3").put("effortLowerLimit", -60.0);
      properties.get("r_finger_middle_joint_3").put("effortUpperLimit", 60.0);
      properties.get("r_finger_middle_joint_3").put("kpPositionLimit", 100.0);
      properties.get("r_finger_middle_joint_3").put("kdPositionLimit", 20.0);
      properties.get("r_finger_middle_joint_3").put("kpVelocityLimit", 0.0);
      properties.get("r_finger_middle_joint_3").put("axis", new Vector3D(0.9999999999797724, -6.535899999867794E-7, -6.326789999872025E-6));
      properties.get("r_finger_middle_joint_3").put("damping", 1.0);
      properties.get("r_finger_middle_joint_3").put("stiction", 0.0);
      properties.put("l_leg_hpz", new HashMap<>());
      properties.put("l_uglut", new HashMap<>());
      properties.get("l_uglut").put("mass", 1.959);
      properties.get("l_uglut").put("centerOfMass", new Vector3D(0.00529262, -0.00344732, 0.00313046));
      properties.get("l_uglut").put("inertia",
                                    new Matrix3D(7.4276E-4, -3.79607E-8, -2.79549E-5, -3.79607E-8, 6.88179E-4, -3.2735E-8, -2.79549E-5, -3.2735E-8, 4.1242E-4));
      properties.get("l_leg_hpz").put("offsetFromParentJoint", new Vector3D(0.0, 0.089, 0.0));
      properties.get("l_leg_hpz").put("positionLowerLimit", -0.174358);
      properties.get("l_leg_hpz").put("positionUpperLimit", 0.786794);
      properties.get("l_leg_hpz").put("velocityLowerLimit", -12.0);
      properties.get("l_leg_hpz").put("velocityUpperLimit", 12.0);
      properties.get("l_leg_hpz").put("effortLowerLimit", -275.0);
      properties.get("l_leg_hpz").put("effortUpperLimit", 275.0);
      properties.get("l_leg_hpz").put("kpPositionLimit", 100.0);
      properties.get("l_leg_hpz").put("kdPositionLimit", 20.0);
      properties.get("l_leg_hpz").put("kpVelocityLimit", 500.0);
      properties.get("l_leg_hpz").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("l_leg_hpz").put("damping", 0.1);
      properties.get("l_leg_hpz").put("stiction", 0.0);
      properties.put("l_leg_hpx", new HashMap<>());
      properties.put("l_lglut", new HashMap<>());
      properties.get("l_lglut").put("mass", 0.898);
      properties.get("l_lglut").put("centerOfMass", new Vector3D(0.0133341, 0.0170484, -0.0312052));
      properties.get("l_lglut").put("inertia",
                                    new Matrix3D(6.91326E-4, -2.24344E-5, 2.50508E-6, -2.24344E-5, 0.00126856, 1.37862E-4, 2.50508E-6, 1.37862E-4, 0.00106487));
      properties.get("l_leg_hpx").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("l_leg_hpx").put("positionLowerLimit", -0.523599);
      properties.get("l_leg_hpx").put("positionUpperLimit", 0.523599);
      properties.get("l_leg_hpx").put("velocityLowerLimit", -12.0);
      properties.get("l_leg_hpx").put("velocityUpperLimit", 12.0);
      properties.get("l_leg_hpx").put("effortLowerLimit", -530.0);
      properties.get("l_leg_hpx").put("effortUpperLimit", 530.0);
      properties.get("l_leg_hpx").put("kpPositionLimit", 100.0);
      properties.get("l_leg_hpx").put("kdPositionLimit", 20.0);
      properties.get("l_leg_hpx").put("kpVelocityLimit", 500.0);
      properties.get("l_leg_hpx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("l_leg_hpx").put("damping", 0.1);
      properties.get("l_leg_hpx").put("stiction", 0.0);
      properties.put("l_leg_hpy", new HashMap<>());
      properties.put("l_uleg", new HashMap<>());
      properties.get("l_uleg").put("mass", 8.204);
      properties.get("l_uleg").put("centerOfMass", new Vector3D(0.0, 0.0, -0.21));
      properties.get("l_uleg").put("inertia", new Matrix3D(0.09, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.02));
      properties.get("l_leg_hpy").put("offsetFromParentJoint", new Vector3D(0.05, 0.022500000000000006, -0.066));
      properties.get("l_leg_hpy").put("positionLowerLimit", -1.61234);
      properties.get("l_leg_hpy").put("positionUpperLimit", 0.65764);
      properties.get("l_leg_hpy").put("velocityLowerLimit", -12.0);
      properties.get("l_leg_hpy").put("velocityUpperLimit", 12.0);
      properties.get("l_leg_hpy").put("effortLowerLimit", -840.0);
      properties.get("l_leg_hpy").put("effortUpperLimit", 840.0);
      properties.get("l_leg_hpy").put("kpPositionLimit", 100.0);
      properties.get("l_leg_hpy").put("kdPositionLimit", 20.0);
      properties.get("l_leg_hpy").put("kpVelocityLimit", 500.0);
      properties.get("l_leg_hpy").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("l_leg_hpy").put("damping", 0.1);
      properties.get("l_leg_hpy").put("stiction", 0.0);
      properties.put("l_leg_kny", new HashMap<>());
      properties.put("l_lleg", new HashMap<>());
      properties.get("l_lleg").put("mass", 4.515);
      properties.get("l_lleg").put("centerOfMass", new Vector3D(0.001, 0.0, -0.187));
      properties.get("l_lleg").put("inertia", new Matrix3D(0.077, 0.0, -0.003, 0.0, 0.076, 0.0, -0.003, 0.0, 0.01));
      properties.get("l_leg_kny").put("offsetFromParentJoint", new Vector3D(-0.05, 0.0, -0.374));
      properties.get("l_leg_kny").put("positionLowerLimit", 0.0);
      properties.get("l_leg_kny").put("positionUpperLimit", 2.35637);
      properties.get("l_leg_kny").put("velocityLowerLimit", -12.0);
      properties.get("l_leg_kny").put("velocityUpperLimit", 12.0);
      properties.get("l_leg_kny").put("effortLowerLimit", -890.0);
      properties.get("l_leg_kny").put("effortUpperLimit", 890.0);
      properties.get("l_leg_kny").put("kpPositionLimit", 100.0);
      properties.get("l_leg_kny").put("kdPositionLimit", 20.0);
      properties.get("l_leg_kny").put("kpVelocityLimit", 500.0);
      properties.get("l_leg_kny").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("l_leg_kny").put("damping", 0.1);
      properties.get("l_leg_kny").put("stiction", 0.0);
      properties.put("l_leg_aky", new HashMap<>());
      properties.put("l_talus", new HashMap<>());
      properties.get("l_talus").put("mass", 0.125);
      properties.get("l_talus").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("l_talus").put("inertia", new Matrix3D(1.01674E-5, 0.0, 0.0, 0.0, 8.42775E-6, 0.0, 0.0, 0.0, 1.30101E-5));
      properties.get("l_leg_aky").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, -0.422));
      properties.get("l_leg_aky").put("positionLowerLimit", -1.0);
      properties.get("l_leg_aky").put("positionUpperLimit", 0.7);
      properties.get("l_leg_aky").put("velocityLowerLimit", -12.0);
      properties.get("l_leg_aky").put("velocityUpperLimit", 12.0);
      properties.get("l_leg_aky").put("effortLowerLimit", -740.0);
      properties.get("l_leg_aky").put("effortUpperLimit", 740.0);
      properties.get("l_leg_aky").put("kpPositionLimit", 100.0);
      properties.get("l_leg_aky").put("kdPositionLimit", 20.0);
      properties.get("l_leg_aky").put("kpVelocityLimit", 500.0);
      properties.get("l_leg_aky").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("l_leg_aky").put("damping", 0.1);
      properties.get("l_leg_aky").put("stiction", 0.0);
      properties.put("l_leg_akx", new HashMap<>());
      properties.put("l_foot", new HashMap<>());
      properties.get("l_foot").put("mass", 2.41);
      properties.get("l_foot").put("centerOfMass", new Vector3D(0.027, 0.0, -0.067));
      properties.get("l_foot").put("inertia", new Matrix3D(0.002, 0.0, 0.0, 0.0, 0.007, 0.0, 0.0, 0.0, 0.008));
      properties.get("l_leg_akx").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("l_leg_akx").put("positionLowerLimit", -0.8);
      properties.get("l_leg_akx").put("positionUpperLimit", 0.8);
      properties.get("l_leg_akx").put("velocityLowerLimit", -12.0);
      properties.get("l_leg_akx").put("velocityUpperLimit", 12.0);
      properties.get("l_leg_akx").put("effortLowerLimit", -360.0);
      properties.get("l_leg_akx").put("effortUpperLimit", 360.0);
      properties.get("l_leg_akx").put("kpPositionLimit", 100.0);
      properties.get("l_leg_akx").put("kdPositionLimit", 20.0);
      properties.get("l_leg_akx").put("kpVelocityLimit", 500.0);
      properties.get("l_leg_akx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("l_leg_akx").put("damping", 0.1);
      properties.get("l_leg_akx").put("stiction", 0.0);
      properties.put("r_leg_hpz", new HashMap<>());
      properties.put("r_uglut", new HashMap<>());
      properties.get("r_uglut").put("mass", 1.959);
      properties.get("r_uglut").put("centerOfMass", new Vector3D(0.00529262, 0.00344732, 0.00313046));
      properties.get("r_uglut").put("inertia",
                                    new Matrix3D(7.4276E-4, 3.79607E-8, -2.79549E-5, 3.79607E-8, 6.88179E-4, 3.2735E-8, -2.79549E-5, 3.2735E-8, 4.1242E-4));
      properties.get("r_leg_hpz").put("offsetFromParentJoint", new Vector3D(0.0, -0.089, 0.0));
      properties.get("r_leg_hpz").put("positionLowerLimit", -0.786794);
      properties.get("r_leg_hpz").put("positionUpperLimit", 0.174358);
      properties.get("r_leg_hpz").put("velocityLowerLimit", -12.0);
      properties.get("r_leg_hpz").put("velocityUpperLimit", 12.0);
      properties.get("r_leg_hpz").put("effortLowerLimit", -275.0);
      properties.get("r_leg_hpz").put("effortUpperLimit", 275.0);
      properties.get("r_leg_hpz").put("kpPositionLimit", 100.0);
      properties.get("r_leg_hpz").put("kdPositionLimit", 20.0);
      properties.get("r_leg_hpz").put("kpVelocityLimit", 500.0);
      properties.get("r_leg_hpz").put("axis", new Vector3D(0.0, 0.0, 1.0));
      properties.get("r_leg_hpz").put("damping", 0.1);
      properties.get("r_leg_hpz").put("stiction", 0.0);
      properties.put("r_leg_hpx", new HashMap<>());
      properties.put("r_lglut", new HashMap<>());
      properties.get("r_lglut").put("mass", 0.898);
      properties.get("r_lglut").put("centerOfMass", new Vector3D(0.0133341, -0.0170484, -0.0312052));
      properties.get("r_lglut").put("inertia",
                                    new Matrix3D(6.91326E-4, 2.24344E-5, 2.50508E-6, 2.24344E-5, 0.00126856, -1.37862E-4, 2.50508E-6, -1.37862E-4, 0.00106487));
      properties.get("r_leg_hpx").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("r_leg_hpx").put("positionLowerLimit", -0.523599);
      properties.get("r_leg_hpx").put("positionUpperLimit", 0.523599);
      properties.get("r_leg_hpx").put("velocityLowerLimit", -12.0);
      properties.get("r_leg_hpx").put("velocityUpperLimit", 12.0);
      properties.get("r_leg_hpx").put("effortLowerLimit", -530.0);
      properties.get("r_leg_hpx").put("effortUpperLimit", 530.0);
      properties.get("r_leg_hpx").put("kpPositionLimit", 100.0);
      properties.get("r_leg_hpx").put("kdPositionLimit", 20.0);
      properties.get("r_leg_hpx").put("kpVelocityLimit", 500.0);
      properties.get("r_leg_hpx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("r_leg_hpx").put("damping", 0.1);
      properties.get("r_leg_hpx").put("stiction", 0.0);
      properties.put("r_leg_hpy", new HashMap<>());
      properties.put("r_uleg", new HashMap<>());
      properties.get("r_uleg").put("mass", 8.204);
      properties.get("r_uleg").put("centerOfMass", new Vector3D(0.0, 0.0, -0.21));
      properties.get("r_uleg").put("inertia", new Matrix3D(0.09, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.02));
      properties.get("r_leg_hpy").put("offsetFromParentJoint", new Vector3D(0.05, -0.022500000000000006, -0.066));
      properties.get("r_leg_hpy").put("positionLowerLimit", -1.61234);
      properties.get("r_leg_hpy").put("positionUpperLimit", 0.65764);
      properties.get("r_leg_hpy").put("velocityLowerLimit", -12.0);
      properties.get("r_leg_hpy").put("velocityUpperLimit", 12.0);
      properties.get("r_leg_hpy").put("effortLowerLimit", -840.0);
      properties.get("r_leg_hpy").put("effortUpperLimit", 840.0);
      properties.get("r_leg_hpy").put("kpPositionLimit", 100.0);
      properties.get("r_leg_hpy").put("kdPositionLimit", 20.0);
      properties.get("r_leg_hpy").put("kpVelocityLimit", 500.0);
      properties.get("r_leg_hpy").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("r_leg_hpy").put("damping", 0.1);
      properties.get("r_leg_hpy").put("stiction", 0.0);
      properties.put("r_leg_kny", new HashMap<>());
      properties.put("r_lleg", new HashMap<>());
      properties.get("r_lleg").put("mass", 4.515);
      properties.get("r_lleg").put("centerOfMass", new Vector3D(0.001, 0.0, -0.187));
      properties.get("r_lleg").put("inertia", new Matrix3D(0.077, -0.0, -0.003, -0.0, 0.076, -0.0, -0.003, -0.0, 0.01));
      properties.get("r_leg_kny").put("offsetFromParentJoint", new Vector3D(-0.05, 0.0, -0.374));
      properties.get("r_leg_kny").put("positionLowerLimit", 0.0);
      properties.get("r_leg_kny").put("positionUpperLimit", 2.35637);
      properties.get("r_leg_kny").put("velocityLowerLimit", -12.0);
      properties.get("r_leg_kny").put("velocityUpperLimit", 12.0);
      properties.get("r_leg_kny").put("effortLowerLimit", -890.0);
      properties.get("r_leg_kny").put("effortUpperLimit", 890.0);
      properties.get("r_leg_kny").put("kpPositionLimit", 100.0);
      properties.get("r_leg_kny").put("kdPositionLimit", 20.0);
      properties.get("r_leg_kny").put("kpVelocityLimit", 500.0);
      properties.get("r_leg_kny").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("r_leg_kny").put("damping", 0.1);
      properties.get("r_leg_kny").put("stiction", 0.0);
      properties.put("r_leg_aky", new HashMap<>());
      properties.put("r_talus", new HashMap<>());
      properties.get("r_talus").put("mass", 0.125);
      properties.get("r_talus").put("centerOfMass", new Vector3D(-0.0, -0.0, -0.0));
      properties.get("r_talus").put("inertia", new Matrix3D(1.01674E-5, 0.0, 0.0, 0.0, 8.42775E-6, 0.0, 0.0, 0.0, 1.30101E-5));
      properties.get("r_leg_aky").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, -0.422));
      properties.get("r_leg_aky").put("positionLowerLimit", -1.0);
      properties.get("r_leg_aky").put("positionUpperLimit", 0.7);
      properties.get("r_leg_aky").put("velocityLowerLimit", -12.0);
      properties.get("r_leg_aky").put("velocityUpperLimit", 12.0);
      properties.get("r_leg_aky").put("effortLowerLimit", -740.0);
      properties.get("r_leg_aky").put("effortUpperLimit", 740.0);
      properties.get("r_leg_aky").put("kpPositionLimit", 100.0);
      properties.get("r_leg_aky").put("kdPositionLimit", 20.0);
      properties.get("r_leg_aky").put("kpVelocityLimit", 500.0);
      properties.get("r_leg_aky").put("axis", new Vector3D(0.0, 1.0, 0.0));
      properties.get("r_leg_aky").put("damping", 0.1);
      properties.get("r_leg_aky").put("stiction", 0.0);
      properties.put("r_leg_akx", new HashMap<>());
      properties.put("r_foot", new HashMap<>());
      properties.get("r_foot").put("mass", 2.41);
      properties.get("r_foot").put("centerOfMass", new Vector3D(0.027, 0.0, -0.067));
      properties.get("r_foot").put("inertia", new Matrix3D(0.002, 0.0, 0.0, 0.0, 0.007, 0.0, 0.0, 0.0, 0.008));
      properties.get("r_leg_akx").put("offsetFromParentJoint", new Vector3D(0.0, 0.0, 0.0));
      properties.get("r_leg_akx").put("positionLowerLimit", -0.8);
      properties.get("r_leg_akx").put("positionUpperLimit", 0.8);
      properties.get("r_leg_akx").put("velocityLowerLimit", -12.0);
      properties.get("r_leg_akx").put("velocityUpperLimit", 12.0);
      properties.get("r_leg_akx").put("effortLowerLimit", -360.0);
      properties.get("r_leg_akx").put("effortUpperLimit", 360.0);
      properties.get("r_leg_akx").put("kpPositionLimit", 100.0);
      properties.get("r_leg_akx").put("kdPositionLimit", 20.0);
      properties.get("r_leg_akx").put("kpVelocityLimit", 500.0);
      properties.get("r_leg_akx").put("axis", new Vector3D(1.0, 0.0, 0.0));
      properties.get("r_leg_akx").put("damping", 0.1);
      properties.get("r_leg_akx").put("stiction", 0.0);

      return properties;
   }

   public static Map<String, Map<String, Object>> atlasSensorProperties()
   {
      Map<String, Map<String, Object>> sensorProperties = new HashMap<>();
      sensorProperties.put("pelvis_imu_sensor_at_pelvis_frame", new HashMap<>());
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame")
                      .put("transformToJoint", new RigidBodyTransform(1.0, 0.0, 0.0, -0.0905, 0.0, 1.0, 0.0, -0.04, 0.0, 0.0, 1.0, -0.0125));
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("accelerationNoiseMean", 0.0);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("accelerationNoiseStandardDeviation", 0.017);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("accelerationBiasMean", 0.1);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("accelerationBiasStandardDeviation", 0.001);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("angularVelocityNoiseMean", 7.5E-6);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("angularVelocityNoiseStandardDeviation", 8.0E-7);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("angularVelocityBiasMean", 0.0);
      sensorProperties.get("pelvis_imu_sensor_at_pelvis_frame").put("angularVelocityBiasStandardDeviation", 0.0);
      sensorProperties.put("l_situational_awareness_camera_sensor_l_situational_awareness_camera", new HashMap<>());
      sensorProperties.get("l_situational_awareness_camera_sensor_l_situational_awareness_camera").put("transformToJoint",
                                                                                                       new RigidBodyTransform(0.2588160883982461,
                                                                                                                              -0.9659266185307408,
                                                                                                                              0.0,
                                                                                                                              0.155,
                                                                                                                              0.9659266185307408,
                                                                                                                              0.2588160883982461,
                                                                                                                              -0.0,
                                                                                                                              0.121,
                                                                                                                              0.0,
                                                                                                                              0.0,
                                                                                                                              1.0,
                                                                                                                              0.785));
      sensorProperties.get("l_situational_awareness_camera_sensor_l_situational_awareness_camera").put("fieldOfView", 2.0);
      sensorProperties.get("l_situational_awareness_camera_sensor_l_situational_awareness_camera").put("clipNear", 0.02);
      sensorProperties.get("l_situational_awareness_camera_sensor_l_situational_awareness_camera").put("clipFar", 300.0);
      sensorProperties.get("l_situational_awareness_camera_sensor_l_situational_awareness_camera").put("imageWidth", 1280);
      sensorProperties.get("l_situational_awareness_camera_sensor_l_situational_awareness_camera").put("imageHeight", 1024);
      sensorProperties.put("r_situational_awareness_camera_sensor_r_situational_awareness_camera", new HashMap<>());
      sensorProperties.get("r_situational_awareness_camera_sensor_r_situational_awareness_camera").put("transformToJoint",
                                                                                                       new RigidBodyTransform(0.2588160883982461,
                                                                                                                              0.9659266185307408,
                                                                                                                              0.0,
                                                                                                                              0.155,
                                                                                                                              -0.9659266185307408,
                                                                                                                              0.2588160883982461,
                                                                                                                              -0.0,
                                                                                                                              -0.121,
                                                                                                                              -0.0,
                                                                                                                              0.0,
                                                                                                                              1.0,
                                                                                                                              0.785));
      sensorProperties.get("r_situational_awareness_camera_sensor_r_situational_awareness_camera").put("fieldOfView", 2.0);
      sensorProperties.get("r_situational_awareness_camera_sensor_r_situational_awareness_camera").put("clipNear", 0.02);
      sensorProperties.get("r_situational_awareness_camera_sensor_r_situational_awareness_camera").put("clipFar", 300.0);
      sensorProperties.get("r_situational_awareness_camera_sensor_r_situational_awareness_camera").put("imageWidth", 1280);
      sensorProperties.get("r_situational_awareness_camera_sensor_r_situational_awareness_camera").put("imageHeight", 1024);
      sensorProperties.put("stereo_camera_left", new HashMap<>());
      sensorProperties.get("stereo_camera_left").put("transformToJoint",
                                                     new RigidBodyTransform(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.035, 0.0, 0.0, 1.0, -0.002));
      sensorProperties.get("stereo_camera_left").put("fieldOfView", 1.39626);
      sensorProperties.get("stereo_camera_left").put("clipNear", 0.02);
      sensorProperties.get("stereo_camera_left").put("clipFar", 300.0);
      sensorProperties.get("stereo_camera_left").put("imageWidth", 1024);
      sensorProperties.get("stereo_camera_left").put("imageHeight", 544);
      sensorProperties.put("stereo_camera_right", new HashMap<>());
      sensorProperties.get("stereo_camera_right").put("transformToJoint",
                                                      new RigidBodyTransform(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.035, 0.0, 0.0, 1.0, -0.002));
      sensorProperties.get("stereo_camera_right").put("fieldOfView", 1.39626);
      sensorProperties.get("stereo_camera_right").put("clipNear", 0.02);
      sensorProperties.get("stereo_camera_right").put("clipFar", 300.0);
      sensorProperties.get("stereo_camera_right").put("imageWidth", 1024);
      sensorProperties.get("stereo_camera_right").put("imageHeight", 544);
      sensorProperties.put("head_head_imu_sensor", new HashMap<>());
      sensorProperties.get("head_head_imu_sensor").put("transformToJoint",
                                                       new RigidBodyTransform(1.0, 0.0, 0.0, -0.0475, 0.0, 1.0, 0.0, 0.035, 0.0, 0.0, 1.0, 0.0));
      sensorProperties.get("head_head_imu_sensor").put("accelerationNoiseMean", 0.0);
      sensorProperties.get("head_head_imu_sensor").put("accelerationNoiseStandardDeviation", 0.017);
      sensorProperties.get("head_head_imu_sensor").put("accelerationBiasMean", 0.1);
      sensorProperties.get("head_head_imu_sensor").put("accelerationBiasStandardDeviation", 0.001);
      sensorProperties.get("head_head_imu_sensor").put("angularVelocityNoiseMean", 7.5E-6);
      sensorProperties.get("head_head_imu_sensor").put("angularVelocityNoiseStandardDeviation", 8.0E-7);
      sensorProperties.get("head_head_imu_sensor").put("angularVelocityBiasMean", 0.0);
      sensorProperties.get("head_head_imu_sensor").put("angularVelocityBiasStandardDeviation", 0.0);
      sensorProperties.put("head_hokuyo_sensor", new HashMap<>());
      sensorProperties.get("head_hokuyo_sensor").put("transformToJoint", new RigidBodyTransform(1.0, 0.0, 0.0, 0.03, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.015));
      sensorProperties.get("head_hokuyo_sensor").put("sweepYawMin", -1.5708);
      sensorProperties.get("head_hokuyo_sensor").put("sweepYawMax", 1.5708);
      sensorProperties.get("head_hokuyo_sensor").put("heightPitchMin", 0.0);
      sensorProperties.get("head_hokuyo_sensor").put("heightPitchMax", 0.0);
      sensorProperties.get("head_hokuyo_sensor").put("minRange", 0.1);
      sensorProperties.get("head_hokuyo_sensor").put("maxRange", 30.0);
      sensorProperties.get("head_hokuyo_sensor").put("pointsPerSweep", 720);
      sensorProperties.get("head_hokuyo_sensor").put("scanHeight", 1);
      return sensorProperties;
   }
}
