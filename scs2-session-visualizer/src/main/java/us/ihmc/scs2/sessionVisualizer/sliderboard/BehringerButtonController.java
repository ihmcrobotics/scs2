package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;

public class BehringerButtonController extends BehringerChannelController
{
   private final MidiChannelConfig button;

   public BehringerButtonController(MidiChannelConfig button, Receiver midiOut)
   {
      super(new SliderboardVariable(0, 1), button.getChannel(), midiOut);

      this.button = button;
   }

   public MidiChannelConfig getButton()
   {
      return button;
   }
}
