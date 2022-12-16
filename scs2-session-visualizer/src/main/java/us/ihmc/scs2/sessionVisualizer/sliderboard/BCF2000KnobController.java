package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;

import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Knob;

public class BCF2000KnobController extends BCF2000ChannelController
{
   private final Knob knob;

   public BCF2000KnobController(Knob knob, Receiver midiOut)
   {
      super(new SliderboardVariable(knob.getMin(), knob.getMax()), knob.getChannel(), midiOut);
      this.knob = knob;
   }

   public Knob getKnob()
   {
      return knob;
   }
}
