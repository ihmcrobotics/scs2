package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;

public class BehringerKnobController extends BehringerChannelController
{
   private final MidiChannelConfig knob;

   public BehringerKnobController(MidiChannelConfig knob, Receiver midiOut)
   {
      super(new SliderboardVariable(knob.getMin(), knob.getMax()), knob.getChannel(), midiOut);
      this.knob = knob;
   }

   public MidiChannelConfig getKnob()
   {
      return knob;
   }
}
