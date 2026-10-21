package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.MujocoNativeLibrary;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoMotorParameters;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * Covers the {@code <dcmotor>} actuation path: that configuring a motor keeps the controller's
 * torque contract intact below the drive's limits, and that past them the torque-speed envelope
 * takes over.
 *
 * <p>Two joints throughout, only one of them motorised. That is not padding -- it is what makes the
 * control addressing testable. A multi-input dcmotor owns three of {@code mjData.ctrl}, so the
 * second actuator's controls no longer start at its own actuator index, and a model with one of
 * each is the smallest one where indexing {@code ctrl} by actuator silently writes a joint's
 * command into its neighbour's setpoint.
 */
public class MujocoDcMotorTest
{
   private static final double DT = 5.0e-4;
   private static final String MOTORISED_JOINT = "motorised";
   private static final String PLAIN_JOINT = "plain";

   // Alex's knee, joint-side: ILM 11525SS phase resistance (0.140 line-to-line / 2), 48 V bus,
   // Kj from the URDF velocity limit of 9.3 rad/s, effort limit 217.2 N*m. Reflected inertia goes on
   // the joint, not the motor, so it is set through the joint definition below.
   private static final double SUPPLY_VOLTAGE = 48.0;
   private static final double NO_LOAD_SPEED = 9.3;
   private static final double RESISTANCE = 0.07;
   private static final double MAX_TORQUE = 217.2;
   private static final double ARMATURE = 0.167;
   private static final double TORQUE_CONSTANT = SUPPLY_VOLTAGE / NO_LOAD_SPEED;

   /** Enough viscous load that a commanded torque settles at a readable speed instead of running away. */
   private static final double JOINT_DAMPING = 25.0;
   private static final double LINK_INERTIA = 0.05;

   private SimulationSession session;

   @BeforeAll
   public static void loadNatives()
   {
      assertTrue(MujocoNativeLibrary.load(), "MuJoCo native library failed to load");
   }

   @AfterEach
   public void shutdown()
   {
      if (session != null)
         session.shutdownSession();
      session = null;
   }

   private static MujocoMotorParameters knee()
   {
      return MujocoMotorParameters.fromNoLoadSpeed(SUPPLY_VOLTAGE, NO_LOAD_SPEED, RESISTANCE, MAX_TORQUE);
   }

   private static RobotDefinition createRobot(double damping)
   {
      RobotDefinition robot = new RobotDefinition("robot");
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      RigidBodyDefinition parent = elevator;

      for (String jointName : new String[] {MOTORISED_JOINT, PLAIN_JOINT})
      {
         RevoluteJointDefinition joint = new RevoluteJointDefinition(jointName, new Vector3D(), new Vector3D(0.0, 1.0, 0.0));
         joint.setEffortLimits(MAX_TORQUE);
         joint.setDamping(damping);
         joint.setArmature(ARMATURE);
         RigidBodyDefinition link = new RigidBodyDefinition(jointName + "Link");
         link.setMass(1.0);
         link.setMomentOfInertia(new MomentOfInertiaDefinition(LINK_INERTIA, LINK_INERTIA, LINK_INERTIA));
         joint.setSuccessor(link);
         parent.addChildJoint(joint);
         parent = link;
      }

      robot.setRootBodyDefinition(elevator);
      return robot;
   }

   /** @param motor the motor for {@link #MOTORISED_JOINT}, or null to leave both joints on the affine servo. */
   private Robot createSession(MujocoMotorParameters motor, double damping)
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      if (motor != null)
         parameters.setMotor(MOTORISED_JOINT, motor);
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addRobot(createRobot(damping));
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(2000);
      return (Robot) session.getPhysicsEngine().getRobots().get(0);
   }

   private MujocoPhysicsEngine engine()
   {
      return (MujocoPhysicsEngine) session.getPhysicsEngine();
   }

   private MujocoRobot mujocoRobot()
   {
      return engine().getMujocoRobots().get(0);
   }

   private MujocoJointActuation actuation(String jointName)
   {
      return mujocoRobot().getJointActuation(jointName);
   }

   private void addFeedforwardTorque(Robot robot, String jointName, double tau)
   {
      OneDoFJointStateBasics output = robot.getControllerManager().getControllerOutput().getOneDoFJointOutput(jointName);
      robot.getControllerManager().addController((Controller) () -> output.setEffort(tau));
   }

   private void simulate(int ticks)
   {
      assertTrue(session.getSimulationSessionControls().simulateNow(ticks), "Simulation reported a failure");
   }

   private double qd(Robot robot, String jointName)
   {
      return ((OneDoFJointBasics) robot.getJoint(jointName)).getQd();
   }

   private double tau(Robot robot, String jointName)
   {
      return ((OneDoFJointBasics) robot.getJoint(jointName)).getTau();
   }

   @Test
   public void testAMotorisedJointCompilesToADcMotorAndAPlainJointDoesNot()
   {
      Robot robot = createSession(knee(), JOINT_DAMPING);
      simulate(1);

      assertTrue(actuation(MOTORISED_JOINT).isDcMotor(), "The motorised joint should be driven by a <dcmotor>");
      assertFalse(actuation(PLAIN_JOINT).isDcMotor(), "A joint with no motor should keep the affine servo");

      int dcMotorIndex = actuation(MOTORISED_JOINT).getActuatorIndex();
      assertEquals(3,
                   mujocoRobot().getMujocoMultiBodyRobot().getActuatorControlCount(dcMotorIndex),
                   "input=\"pos vel ff\" should own three controls");
      assertEquals(1,
                   mujocoRobot().getMujocoMultiBodyRobot().getActuatorControlCount(actuation(PLAIN_JOINT).getActuatorIndex()),
                   "The affine servo should own one control");
   }

   /**
    * The regression this whole addressing change exists for: with a three-control actuator in the
    * model, the plain joint's control address must have moved past it.
    */
   @Test
   public void testControlAddressIsNotTheActuatorIndexOnceADcMotorExists()
   {
      createSession(knee(), JOINT_DAMPING);
      simulate(1);

      MujocoJointActuation plain = actuation(PLAIN_JOINT);
      assertNotEquals(plain.getActuatorIndex(),
                      plain.getControlAddress(),
                      "A joint after a dcmotor must not be addressed by its actuator index");
      assertEquals(Mujoco.mjINPUT_POS | Mujoco.mjINPUT_VEL | Mujoco.mjINPUT_FF,
                   engine().getDynamicsWorld().getModel().actuator_ctrlspec().get(actuation(MOTORISED_JOINT).getActuatorIndex()),
                   "The dcmotor's input signature should be pos|vel|ff");
   }

   /**
    * The behavioural half of the control-addressing fix, and the test that actually bites: command
    * the joint that sits AFTER the dcmotor in the model and check it receives the command.
    *
    * <p>Addressing {@code ctrl} by actuator index sends this joint's torque to {@code ctrl[1]},
    * which is the dcmotor's velocity setpoint, and leaves its own {@code ctrl[3]} at zero. The
    * joint then produces no torque at all while silently perturbing its neighbour. Asserting on
    * the addresses alone does not catch that -- the addresses are a property of the compiled model,
    * not of the code that writes to them -- so this commands the joint and reads the torque back.
    */
   @Test
   public void testAJointAfterADcMotorStillReceivesItsOwnCommand()
   {
      Robot robot = createSession(knee(), JOINT_DAMPING);
      addFeedforwardTorque(robot, PLAIN_JOINT, 40.0);
      simulate(2000);

      assertEquals(40.0, tau(robot, PLAIN_JOINT), 1.0e-6, "The joint after the dcmotor did not receive its own command");
      assertNotEquals(0.0, qd(robot, PLAIN_JOINT), "The joint after the dcmotor never moved");
   }

   /**
    * The contract that must not change: below the drive's limits a dcmotor delivers exactly the
    * torque it was asked for, because its torque-to-voltage map cancels the back-EMF. If this
    * drifts, configuring a motor has silently changed every controller's effective gain.
    */
   @Test
   public void testCommandedTorqueIsDeliveredExactlyBelowTheLimits()
   {
      Robot robot = createSession(knee(), JOINT_DAMPING);
      addFeedforwardTorque(robot, MOTORISED_JOINT, 100.0);
      simulate(2000);

      assertEquals(100.0, tau(robot, MOTORISED_JOINT), 1.0e-6, "A dcmotor below saturation should deliver the commanded torque");
   }

   /** The same command on both drives must produce the same motion while nothing saturates. */
   @Test
   public void testADcMotorMatchesTheAffineServoBelowTheLimits()
   {
      Robot withMotor = createSession(knee(), JOINT_DAMPING);
      addFeedforwardTorque(withMotor, MOTORISED_JOINT, 100.0);
      simulate(2000);
      double qdWithMotor = qd(withMotor, MOTORISED_JOINT);
      shutdown();

      Robot withServo = createSession(null, JOINT_DAMPING);
      addFeedforwardTorque(withServo, MOTORISED_JOINT, 100.0);
      simulate(2000);
      double qdWithServo = qd(withServo, MOTORISED_JOINT);

      assertEquals(qdWithServo, qdWithMotor, 1.0e-9, "Below its limits a dcmotor should be indistinguishable from the affine servo");
   }

   /**
    * Past the current limit the torque clamps. The affine servo has no such limit, which is the
    * point: this is physics the old path could not express at any gain.
    */
   @Test
   public void testTorqueClampsAtTheDrivesLimit()
   {
      Robot robot = createSession(knee(), JOINT_DAMPING);
      addFeedforwardTorque(robot, MOTORISED_JOINT, 3.0 * MAX_TORQUE);
      simulate(2000);

      assertEquals(MAX_TORQUE, tau(robot, MOTORISED_JOINT), 1.0e-3, "A dcmotor should clamp at its saturation torque");
   }

   /**
    * The envelope's other edge, and the one that matters for a leg: no commanded torque, however
    * large, can push the joint past the speed at which back-EMF consumes the whole supply.
    */
   @Test
   public void testTheJointCannotExceedTheNoLoadSpeed()
   {
      Robot robot = createSession(knee(), 0.0);
      addFeedforwardTorque(robot, MOTORISED_JOINT, 10.0 * MAX_TORQUE);
      simulate(20000);

      assertEquals(NO_LOAD_SPEED, qd(robot, MOTORISED_JOINT), 1.0e-3, "An unloaded dcmotor should settle at supplyVoltage / Kj");
      assertEquals(NO_LOAD_SPEED,
                   knee().noLoadSpeed(),
                   1.0e-12,
                   "The parameter block's own no-load speed should agree with what the simulation does");
   }

   /**
    * The counterpart to {@link #testTheJointCannotExceedTheNoLoadSpeed}, and the one that shows the
    * capability is new: the affine servo has no speed ceiling at all, so the same command runs the
    * joint far past it. Without this, the no-load-speed test could be measuring an effort limit
    * rather than an electrical one -- note that the torque CLAMP on its own proves nothing, since
    * the joint's actuatorfrcrange clamps the affine servo at the same 217.2 N*m.
    */
   @Test
   public void testTheAffineServoHasNoSpeedCeiling()
   {
      Robot robot = createSession(null, 0.0);
      addFeedforwardTorque(robot, MOTORISED_JOINT, 10.0 * MAX_TORQUE);
      simulate(20000);

      double qdServo = qd(robot, MOTORISED_JOINT);
      assertTrue(qdServo > 10.0 * NO_LOAD_SPEED,
                 "The affine servo should have no speed ceiling, but settled at " + qdServo + " rad/s");
   }

   /** A simulation that configures no motor must emit no dcmotor at all. */
   @Test
   public void testNoMotorMeansNoDcMotorAnywhere()
   {
      createSession(null, JOINT_DAMPING);
      simulate(1);

      for (MujocoJointActuation actuation : mujocoRobot().getJointActuations())
      {
         assertFalse(actuation.isDcMotor(), "Joint '" + actuation.getJointName() + "' should keep the affine servo");
         assertEquals(actuation.getActuatorIndex(),
                      actuation.getControlAddress(),
                      "With only single-control actuators, control address and actuator index should agree");
      }
   }

   /**
    * The path Alex actually uses: {@code maxTorque = 0} inherits the joint's effort limit, so the
    * torque limit is stated once in the URDF instead of being restated per motor where it could
    * drift. If this regresses, every motor configured that way silently becomes unlimited.
    */
   @Test
   public void testMaxTorqueOfZeroInheritsTheJointsEffortLimit()
   {
      MujocoMotorParameters inheriting = MujocoMotorParameters.fromNoLoadSpeed(SUPPLY_VOLTAGE, NO_LOAD_SPEED, RESISTANCE, 0.0);
      Robot robot = createSession(inheriting, JOINT_DAMPING);
      addFeedforwardTorque(robot, MOTORISED_JOINT, 3.0 * MAX_TORQUE);
      simulate(2000);

      assertEquals(MAX_TORQUE,
                   tau(robot, MOTORISED_JOINT),
                   1.0e-3,
                   "A motor with maxTorque 0 should clamp at the joint's effort limit");
   }

   /**
    * A joint with no effort limit and a motor that does not state one has no torque clamp, so only
    * the electrical limit remains. It must still run rather than compiling to something degenerate.
    */
   @Test
   public void testAMotorWithNoTorqueLimitAnywhereStillRuns()
   {
      MujocoMotorParameters noClamp = MujocoMotorParameters.fromNoLoadSpeed(SUPPLY_VOLTAGE, NO_LOAD_SPEED, RESISTANCE, 0.0);
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.setMotor(MOTORISED_JOINT, noClamp);
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      RobotDefinition definition = createRobot(JOINT_DAMPING);
      for (String jointName : new String[] {MOTORISED_JOINT, PLAIN_JOINT})
         ((RevoluteJointDefinition) definition.getJointDefinition(jointName)).setEffortLimits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
      session.addRobot(definition);
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(2000);

      addFeedforwardTorque((Robot) session.getPhysicsEngine().getRobots().get(0), MOTORISED_JOINT, 50.0);
      simulate(2000);

      assertTrue(actuation(MOTORISED_JOINT).isDcMotor(), "The motor should still have compiled to a dcmotor");
      assertEquals(50.0, tau((Robot) session.getPhysicsEngine().getRobots().get(0), MOTORISED_JOINT), 1.0e-6,
                   "With no clamp anywhere the commanded torque should pass through");
   }

   /** The parameter block's own envelope arithmetic should agree with the motor equations. */
   @Test
   public void testAvailableTorqueMatchesTheEnvelope()
   {
      MujocoMotorParameters motor = knee();
      assertEquals(MAX_TORQUE, motor.availableTorque(0.0), 1.0e-9, "At rest the current clamp should bind, not the voltage");
      assertEquals(0.0, motor.availableTorque(motor.noLoadSpeed()), 1.0e-9, "No torque should remain at the no-load speed");
      assertEquals(0.0, motor.availableTorque(2.0 * motor.noLoadSpeed()), 0.0, "Past the no-load speed there is nothing left");
      assertTrue(motor.stallTorque() > MAX_TORQUE, "Alex's legs should be current-limited, not resistance-limited");
   }

   /**
    * A motor keyed to a joint that does not exist must not crash the session and must not attach
    * itself to some other joint. It warns and leaves everything on the affine servo -- documented
    * behaviour, because a shared parameter block may legitimately cover joints only some robots
    * have.
    */
   @Test
   public void testAMotorForAnUnknownJointIsIgnoredRatherThanMisapplied()
   {
      MujocoSimulationParameters parameters = new MujocoSimulationParameters();
      parameters.setMotor("thisJointDoesNotExist", knee());
      session = new SimulationSession((inertialFrame, rootRegistry) -> new MujocoPhysicsEngine(inertialFrame, rootRegistry, parameters));
      session.addRobot(createRobot(JOINT_DAMPING));
      session.setSessionDTSeconds(DT);
      session.setGravity(0.0, 0.0, 0.0);
      session.initializeBufferSize(2000);
      simulate(1);

      for (MujocoJointActuation actuation : mujocoRobot().getJointActuations())
         assertFalse(actuation.isDcMotor(), "Joint '" + actuation.getJointName() + "' should not have been motorised");
   }

   /** A half-specified motor is a silent trap, so the parameter block refuses to be one. */
   @Test
   public void testMotorParametersRejectNonsense()
   {
      assertAllThrow(() -> MujocoMotorParameters.quasiStatic(0.0, RESISTANCE, SUPPLY_VOLTAGE, MAX_TORQUE),
                     () -> MujocoMotorParameters.quasiStatic(TORQUE_CONSTANT, 0.0, SUPPLY_VOLTAGE, MAX_TORQUE),
                     () -> MujocoMotorParameters.quasiStatic(TORQUE_CONSTANT, RESISTANCE, 0.0, MAX_TORQUE),
                     () -> MujocoMotorParameters.quasiStatic(TORQUE_CONSTANT, RESISTANCE, SUPPLY_VOLTAGE, -1.0),
                     () -> MujocoMotorParameters.quasiStatic(Double.NaN, RESISTANCE, SUPPLY_VOLTAGE, MAX_TORQUE),
                     () -> MujocoMotorParameters.fromNoLoadSpeed(SUPPLY_VOLTAGE, 0.0, RESISTANCE, MAX_TORQUE));
   }

   private static void assertAllThrow(Runnable... cases)
   {
      for (int i = 0; i < cases.length; i++)
      {
         int index = i;
         org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class,
                                                       cases[i]::run,
                                                       "Case " + index + " should have been rejected");
      }
   }
}
