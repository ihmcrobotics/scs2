package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.commons.MathTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Slider;

public class BCF2000SliderController
{
   private final Slider slider;
   private final Receiver midiOut;

   private final SliderboardVariable controlVariable;

   private int currentSliderValue = -1;
   private int newSliderValue = -1;

   public BCF2000SliderController(Slider slider, Receiver midiOut)
   {
      this.slider = slider;
      this.midiOut = midiOut;

      controlVariable = new SliderboardVariable(slider.getMin(), slider.getMax());
   }

   public boolean handleMessage(ShortMessage message, long timestamp)
   {
      if (slider.getChannel() != message.getData1()) // Should it use getChannel instead
         return false;

      newSliderValue = MathTools.clamp(message.getData2(), slider.getMin(), slider.getMax());
      return true;
   }

   public void moveSlider(int value)
   {
      if (value == -1)
         return;

      try
      {
         ShortMessage message = new ShortMessage();
         message.setMessage(176, 0, slider.getChannel(), value);
         midiOut.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         e.printStackTrace();
      }
   }

   public void update()
   {
      if (newSliderValue != -1)
      {
         currentSliderValue = newSliderValue;
      }

      if (newSliderValue != -1)
      {
         controlVariable.setValue(newSliderValue);
         newSliderValue = -1;
      }
      else if (controlVariable.getValue() != currentSliderValue)
      {
         currentSliderValue = controlVariable.getValue(); // TODO Not sure if that's the best way
         moveSlider(controlVariable.getValue());
      }
   }

   public SliderboardVariable getControlVariable()
   {
      return controlVariable;
   }
}
