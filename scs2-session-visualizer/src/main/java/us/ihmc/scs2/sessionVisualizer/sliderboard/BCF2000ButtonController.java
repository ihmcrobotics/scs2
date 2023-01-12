package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;

import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Button;

public class BCF2000ButtonController extends BCF2000ChannelController
{
   private final Button button;

   public BCF2000ButtonController(Button button, Receiver midiOut)
   {
      super(new SliderboardVariable(0, 1), button.getChannel(), midiOut);

      this.button = button;
   }

   public Button getButton()
   {
      return button;
   }
}
