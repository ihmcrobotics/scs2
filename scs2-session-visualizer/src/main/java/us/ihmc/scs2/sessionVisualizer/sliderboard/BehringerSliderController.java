package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;

public class BehringerSliderController extends BehringerChannelController
{
   private final MidiChannelConfig slider;

   public BehringerSliderController(MidiChannelConfig slider, Receiver midiOut)
   {
      super(new SliderboardVariable(slider.getMin(), slider.getMax()), slider.getChannel(), midiOut);

      this.slider = slider;
   }

   public MidiChannelConfig getSlider()
   {
      return slider;
   }
}
