package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardType;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

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
      scs.setSliderboard(createSliderboardWithYoVariables(YoSliderboardType.XTOUCHCOMPACT, "XTouch", 16, 39, 9, scs.getRootRegistry()));
      scs.setSliderboard(createSliderboardWithYoVariables(YoSliderboardType.BCF2000, "BCF2000", 8, 16, 8, scs.getRootRegistry()));

      scs.setSliderboardButton("AnotherSliderboard", YoSliderboardType.XTOUCHCOMPACT, 0, "is_rootJoint_pinned");
      scs.setDefaultSliderboardKnob(0, "qd_rootJoint_wZ");

      scs.start(true, false, false);
   }

   private static YoSliderboardDefinition createSliderboardWithYoVariables(YoSliderboardType type,
                                                                           String prefix,
                                                                           int knobs,
                                                                           int buttons,
                                                                           int sliders,
                                                                           YoRegistry registry)
   {
      YoSliderboardDefinition sliderboard = new YoSliderboardDefinition(prefix);
      sliderboard.setType(type);

      for (int i = 0; i < knobs; i++)
      {
         YoDouble knob = new YoDouble(prefix + "Knob" + i, registry);
         sliderboard.getKnobs().add(new YoKnobDefinition(knob.getName(), i, -128, 128));
      }

      for (int i = 0; i < buttons; i++)
      {
         YoBoolean button = new YoBoolean(prefix + "Button" + i, registry);
         sliderboard.getButtons().add(new YoButtonDefinition(button.getName(), i));
      }

      for (int i = 0; i < sliders; i++)
      {
         YoDouble slider = new YoDouble(prefix + "Slider" + i, registry);
         sliderboard.getSliders().add(new YoSliderDefinition(slider.getName(), i, 0, 255));
      }

      return sliderboard;
   }
}
