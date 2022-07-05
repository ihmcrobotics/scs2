package us.ihmc.scs2.examples.simulations;

import javafx.scene.control.Button;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.simulation.SimulationSession;

public class FallingBallSimulation
{
   private final SessionVisualizerControls controls;
   private final BallRobotDefinition robotDefinition = new BallRobotDefinition();

   public FallingBallSimulation()
   {
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      initialJointState.setVelocity(new Vector3D(10.0, 0.0, 0.0), new Vector3D(-1.0, 0.0, 0.0));
      robotDefinition.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      SimulationSession simulationSession = new SimulationSession();
      simulationSession.addRobot(robotDefinition);
      simulationSession.addTerrainObject(new SlopeGroundDefinition(Math.toRadians(15.0)));

      controls = SessionVisualizer.startSessionVisualizer(simulationSession);
      controls.addSessionChangedListener((oldSession, newSession) ->
      {
         if (newSession != null && newSession instanceof SimulationSession)
            initializeCustomGUIControls();
         else
            disposeCustomGUIControls();
      });
   }

   private YoDoubleProperty q_z;
   private Button moveUpButton;

   public void initializeCustomGUIControls()
   {
      if (controls.isVisualizerShutdown())
         return;

      if (q_z != null)
         return;

      q_z = controls.newYoDoubleProperty("q_" + robotDefinition.getRootJointDefinitions().get(0).getName() + "_z");
      moveUpButton = new Button("Move Up");
      moveUpButton.setOnAction(e -> q_z.set(q_z.get() + 0.1));
      controls.addCustomGUIControl(moveUpButton);
   }

   public void disposeCustomGUIControls()
   {
      if (controls.isVisualizerShutdown())
         return;

      if (moveUpButton != null)
      {
         controls.removeCustomGUIControl(moveUpButton);
         moveUpButton = null;
      }

      if (q_z != null)
      {
         q_z.dispose();
         q_z = null;
      }
   }

   public static void main(String[] args)
   {
      new FallingBallSimulation();
   }
}
