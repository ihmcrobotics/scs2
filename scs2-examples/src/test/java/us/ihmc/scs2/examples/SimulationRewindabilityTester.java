package us.ihmc.scs2.examples;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class SimulationRewindabilityTester extends RobotDefinition
{
   private enum RobotSide
   {
      LEFT, RIGHT;

      public String getLowerCaseName()
      {
         return name().toLowerCase();
      }
   }

   ;
   // Materials:
   private final MaterialDefinition redMaterial = new MaterialDefinition(ColorDefinitions.Red());
   private final MaterialDefinition greenMaterial = new MaterialDefinition(ColorDefinitions.Green());
   private final MaterialDefinition grayMaterial = new MaterialDefinition(ColorDefinitions.Gray());

   private final EnumMap<RobotSide, MaterialDefinition> appendageMaterial = new EnumMap<>(Map.of(RobotSide.LEFT, redMaterial, RobotSide.RIGHT, greenMaterial));

   // Joint Spacing:
   private static final double hipToHipSpacingY = 0.15;
   private static final double thighLength = 0.40;

   // Pelvis and Leg Mass Properties:
   private static final double pelvisMass = 3.0;
   private static final double pelvisRadiusOfGyrationX = 0.1;
   private static final double pelvisRadiusOfGyrationY = 0.1;
   private static final double pelvisRadiusOfGyrationZ = 0.1;

   private static final double thighMass = 5.0;
   private static final double thighRadiusOfGyrationX = 0.05;
   private static final double thighRadiusOfGyrationY = 0.05;
   private static final double thighRadiusOfGyrationZ = thighLength * 0.8;

   // Visual Properties:
   private static final double thighTopVisualRadius = 0.05;
   private static final double thighVisualRadius = 0.025;

   // Joint Definitions:
   private SixDoFJointDefinition pelvisJointDefinition;
   private RevoluteJointDefinition leftHipZ;

   public SimulationRewindabilityTester()
   {
      super("SimulationRewindabilityTester");

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      pelvisJointDefinition = createPelvis();
      rootBody.addChildJoint(pelvisJointDefinition);

      leftHipZ = createHipZ(RobotSide.LEFT);
      pelvisJointDefinition.getSuccessor().addChildJoint(leftHipZ);

      setRootBodyDefinition(rootBody);
   }

   public void setInitialState(boolean setInitialLinearVelocity, boolean setInitialAngularVelocity)
   {
      SixDoFJointState initialSixDoFState = new SixDoFJointState(new YawPitchRoll(0.1, 0.02, 0.03), new Point3D(0.02, 0.03, 0.04));

      Vector3D initialLinearVelocity = new Vector3D();
      Vector3D initialAngularVelocity = new Vector3D();

      if (setInitialLinearVelocity)
         initialLinearVelocity.set(0.2, 0.13, -0.2);

      if (setInitialAngularVelocity)
         initialAngularVelocity.set(0.2, 0.6, -0.2);

      initialSixDoFState.setVelocity(initialAngularVelocity, initialLinearVelocity);
      pelvisJointDefinition.setInitialJointState(initialSixDoFState);
   }

   // Joints and Rigid Bodies:
   private SixDoFJointDefinition createPelvis()
   {
      SixDoFJointDefinition pelvisJointDefinition = new SixDoFJointDefinition("pelvis");

      RigidBodyDefinition pelvisRigidBody = new RigidBodyDefinition("pelvis");
      pelvisRigidBody.setMass(pelvisMass);
      pelvisRigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(pelvisMass,
                                                                                           pelvisRadiusOfGyrationX,
                                                                                           pelvisRadiusOfGyrationY,
                                                                                           pelvisRadiusOfGyrationZ));
      pelvisRigidBody.setCenterOfMassOffset(new Vector3D());

      KinematicPointDefinition pelvisMiddlePoint = new KinematicPointDefinition("pelvisPoint");
      pelvisJointDefinition.addKinematicPointDefinition(pelvisMiddlePoint);

      List<VisualDefinition> pelvisVisuals = createPelvisVisuals();

      pelvisRigidBody.addVisualDefinitions(pelvisVisuals);
      pelvisJointDefinition.setSuccessor(pelvisRigidBody);

      return pelvisJointDefinition;
   }

   private RevoluteJointDefinition createHipZ(RobotSide robotSide)
   {
      String sidePrefix = robotSide.getLowerCaseName();

      RevoluteJointDefinition hipZJoint = new RevoluteJointDefinition(sidePrefix + "HipZ", new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 1.0));

      RigidBodyDefinition thighRigidBody = new RigidBodyDefinition(sidePrefix + "Thigh");
      thighRigidBody.setMass(thighMass);
      thighRigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(thighMass,
                                                                                          thighRadiusOfGyrationX,
                                                                                          thighRadiusOfGyrationY,
                                                                                          thighRadiusOfGyrationZ));
      thighRigidBody.setCenterOfMassOffset(new Vector3D(0.0, 0.0, -thighLength / 2.0));

      List<VisualDefinition> thighVisualDefinitions = createThighVisuals(robotSide);
      thighRigidBody.addVisualDefinitions(thighVisualDefinitions);

      hipZJoint.setSuccessor(thighRigidBody);
      return hipZJoint;
   }

   // Visuals:

   private List<VisualDefinition> createPelvisVisuals()
   {
      VisualDefinitionFactory pelvisVisualFactory = new VisualDefinitionFactory();

      pelvisVisualFactory.appendTranslation(new Vector3D(0.0, 0.0, 0.04));
      pelvisVisualFactory.addCylinder(0.02, hipToHipSpacingY / 2.0, redMaterial);

      return pelvisVisualFactory.getVisualDefinitions();
   }

   private List<VisualDefinition> createThighVisuals(RobotSide robotSide)
   {
      VisualDefinitionFactory thighVisualFactory = new VisualDefinitionFactory();

      thighVisualFactory.addSphere(thighTopVisualRadius, grayMaterial);
      thighVisualFactory.appendTranslation(0.0, 0.0, -thighLength / 2.0);
      thighVisualFactory.addCylinder(thighLength, thighVisualRadius, appendageMaterial.get(robotSide));

      return thighVisualFactory.getVisualDefinitions();
   }

   private record VariableEntry(String name, double value)
   {
   }

   private record SimulationState(List<VariableEntry> variableEntries)
   {
      @Override
      public String toString()
      {
         return getClass().getSimpleName() + ":" + EuclidCoreIOTools.getCollectionString("[\n\t",
                                                                                         "\n]",
                                                                                         "\n\t",
                                                                                         variableEntries,
                                                                                         entry -> entry.name + ": " + entry.value);
      }
   }

   @Test
   public void testContactPointBasedPhysicsEngineRewindability()
   {
      PhysicsEngineFactory physicsEngineFactoryToTest = SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory();
      assertRewindability(physicsEngineFactoryToTest);
   }

   @Test
   public void testImpulseBasedPhysicsEngineRewindability()
   {
      PhysicsEngineFactory physicsEngineFactoryToTest = SimulationConstructionSet2.impulseBasedPhysicsEngineFactory();
      assertRewindability(physicsEngineFactoryToTest);
   }

   @Test
   public void testBulletPhysicsEngineRewindability()
   {
      PhysicsEngineFactory physicsEngineFactoryToTest = BulletPhysicsEngine::new;
      assertRewindability(physicsEngineFactoryToTest);
   }

   private static void assertRewindability(PhysicsEngineFactory physicsEngineFactoryToTest)
   {
      boolean applyTorque = true;
      boolean setGravity = true;
      boolean setInitialLinearVelocity = true;
      boolean setInitialAngularVelocity = true;
      boolean visualize = false;

      SimulationRewindabilityTester definition = new SimulationRewindabilityTester();

      definition.setInitialState(setInitialLinearVelocity, setInitialAngularVelocity);

      SimulationSession simulationSession = new SimulationSession(physicsEngineFactoryToTest);
      SimulationSessionControls simulationSessionControls = simulationSession.getSimulationSessionControls();

      simulationSession.addRobot(definition);

      simulationSession.setSessionDTSeconds(0.0001);
      simulationSession.setBufferRecordTickPeriod(1);

      if (setGravity)
         simulationSession.setGravity(0.1, 0.05, 0.02);
      else
         simulationSession.setGravity(0.0, 0.0, 0.0);

      simulationSession.submitBufferSizeRequestAndWait(40000);

      YoRegistry rootRegistry = simulationSession.getRootRegistry();
      YoDouble tau_leftHipZ = (YoDouble) rootRegistry.findVariable("tau_leftHipZ");
      YoDouble qd_pelvis_x = (YoDouble) rootRegistry.findVariable("qd_pelvis_x");
      YoDouble qdd_pelvis_x = (YoDouble) rootRegistry.findVariable("qdd_pelvis_x");
      YoDouble time = (YoDouble) rootRegistry.findVariable("time[sec]");

      if (applyTorque)
         tau_leftHipZ.set(1.0);

      SessionVisualizerControls sessionVisualizerControls = null;
      if (visualize)
      {
         sessionVisualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
         sessionVisualizerControls.waitUntilVisualizerFullyUp();
      }

      double initialSimulationDuration = 0.2;

      // Use these doubles to make a lot of tick changes:
      //      double simulationDurationToCheckPoint = 0.2;
      //      double simulationDurationAfterCheckPoint = 0.2;

      // Use these ints to just do one tick tests:
      int simulationDurationToCheckPoint = 1;
      int simulationDurationAfterCheckPoint = 1;

      int numberOfRuns = 4;

      simulationSessionControls.simulateNow(initialSimulationDuration);
      simulationSessionControls.setBufferInPoint();

      SimulationState[] begin_states = new SimulationState[numberOfRuns];
      SimulationState[] check_states = new SimulationState[numberOfRuns];
      SimulationState[] end_states = new SimulationState[numberOfRuns];

      for (int i = 0; i < numberOfRuns; i++)
      {
         simulationSessionControls.gotoBufferInPoint();
         begin_states[i] = extractSimulationState(rootRegistry);

         simulationSessionControls.simulateNow(simulationDurationToCheckPoint);
         check_states[i] = extractSimulationState(rootRegistry);

         System.out.println("\nt = " + time);
         System.out.println(qd_pelvis_x);
         System.out.println(qdd_pelvis_x);

         simulationSessionControls.simulateNow(simulationDurationAfterCheckPoint);
         end_states[i] = extractSimulationState(rootRegistry);
      }

      System.out.println("Done doing rewindability tests!");

      if (sessionVisualizerControls != null)
         sessionVisualizerControls.shutdownSession();
      else
         simulationSession.shutdownSession();

      for (int i = 0; i < numberOfRuns - 1; i++)
      {
         assertEquals(begin_states[i], begin_states[i + 1]);
         assertEquals(check_states[i], check_states[i + 1]);
         assertEquals(end_states[i], end_states[i + 1]);
      }
   }

   private static SimulationState extractSimulationState(YoRegistry rootRegistry)
   {
      List<YoVariable> allVariables = collectSubtreeVariables(rootRegistry);
      return new SimulationState(allVariables.stream().map(variable -> new VariableEntry(variable.getName(), variable.getValueAsDouble())).toList());
   }

   private static ArrayList<YoVariable> collectSubtreeVariables(YoRegistry registry)
   {
      ArrayList<YoVariable> variables = new ArrayList<>();
      collectSubtreeVariables(registry, variables);
      return variables;
   }

   private static void collectSubtreeVariables(YoRegistry registry, List<YoVariable> variablesToPack)
   {
      if (registry.getName().contains("Statistics"))
         return;

      // Add ours:
      variablesToPack.addAll(registry.getVariables());

      // Add children's recursively:
      for (YoRegistry child : registry.getChildren())
      {
         collectSubtreeVariables(child, variablesToPack);
      }
   }
}
