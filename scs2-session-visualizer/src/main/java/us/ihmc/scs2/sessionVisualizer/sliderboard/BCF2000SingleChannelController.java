package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

public abstract class BCF2000SingleChannelController
{
   protected final SliderboardVariable controlVariable;
   protected final Receiver midiOut;

   public BCF2000SingleChannelController(SliderboardVariable controlVariable, Receiver midiOut)
   {
      this.controlVariable = controlVariable;
      this.midiOut = midiOut;
   }

   public abstract boolean handleMessage(ShortMessage message, long timestamp);

   public abstract void moveSlider();

   public abstract void moveSlider(int value);

   public abstract void update();

   public SliderboardVariable getControlVariable()
   {
      return controlVariable;
   }
}
