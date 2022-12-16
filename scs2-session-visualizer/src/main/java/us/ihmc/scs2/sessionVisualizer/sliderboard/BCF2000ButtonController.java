package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.commons.MathTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Button;

public class BCF2000ButtonController
{
   private final Button button;
   private final Receiver midiOut;

   private final SliderboardVariable controlVariable;

   private int currentButtonValue = -1;
   private int newButtonValue = -1;

   public BCF2000ButtonController(Button button, Receiver midiOut)
   {
      this.button = button;
      this.midiOut = midiOut;

      controlVariable = new SliderboardVariable(0, 1);
   }

   public boolean handleMessage(ShortMessage message, long timestamp)
   {
      if (button.getChannel() != message.getData1()) // Should it use getChannel instead
         return false;

      newButtonValue = MathTools.clamp(message.getData2(), 0, 1);
      return true;
   }

   public void moveSlider()
   {
      currentButtonValue = controlVariable.getValue(); // TODO Not sure if that's the best way
      moveSlider(controlVariable.getValue());
   }

   public void moveSlider(int value)
   {
      if (value == -1)
         return;

      try
      {
         ShortMessage message = new ShortMessage();
         message.setMessage(176, 0, button.getChannel(), value == 0 ? 0 : 127);
         midiOut.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         e.printStackTrace();
      }
   }

   public void update()
   {
      if (newButtonValue != -1)
      {
         currentButtonValue = newButtonValue;
      }

      if (newButtonValue != -1)
      {
         controlVariable.setValue(newButtonValue);
         newButtonValue = -1;
      }
      else if (controlVariable.getValue() != currentButtonValue)
      {
         moveSlider();
      }
   }

   public SliderboardVariable getControlVariable()
   {
      return controlVariable;
   }
}
