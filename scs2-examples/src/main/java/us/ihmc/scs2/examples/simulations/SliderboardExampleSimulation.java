package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;

public class SliderboardExampleSimulation
{
   public static void main(String[] args)
   {
      BoxRobotDefinition definition = new BoxRobotDefinition();
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      definition.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory());
      scs.addRobot(definition);
      scs.addTerrainObject(new SlopeGroundDefinition());

      scs.clearAllSliderboards();
      scs.setSliderboard(createSliderboard());
      scs.setSliderboardButton("AnotherSliderboard", 0, "is_rootJoint_pinned");
      scs.setDefaultSliderboardKnob(0, "qd_rootJoint_wZ");

      scs.start(true, false, false);
   }

   private static YoSliderboardDefinition createSliderboard()
   {
      YoSliderboardDefinition sliderboard = new YoSliderboardDefinition("Default");
      sliderboard.getSliders().add(new YoSliderDefinition("q_rootJoint_yaw", -1, -Math.PI, Math.PI));
      sliderboard.getSliders().add(new YoSliderDefinition("q_rootJoint_pitch", -1, -Math.PI / 2.0, Math.PI / 2.0));
      sliderboard.getSliders().add(new YoSliderDefinition("q_rootJoint_roll", -1, -Math.PI, Math.PI));
      sliderboard.getSliders().add(new YoSliderDefinition("root.gravityZ", -1, -10, 10));
      return sliderboard;
   }
}
