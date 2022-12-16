package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;

import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Slider;

public class BCF2000SliderController extends BCF2000ChannelController
{
   private final Slider slider;

   public BCF2000SliderController(Slider slider, Receiver midiOut)
   {
      super(new SliderboardVariable(slider.getMin(), slider.getMax()), slider.getChannel(), midiOut);

      this.slider = slider;
   }

   public Slider getSlider()
   {
      return slider;
   }
}
