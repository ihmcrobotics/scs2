package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.commons.MathTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController.Knob;

public class BCF2000KnobController
{
   private final Knob knob;
   private final Receiver midiOut;

   private final SliderboardVariable controlVariable;

   private int currentKnobValue = -1;
   private int newKnobValue = -1;

   public BCF2000KnobController(Knob knob, Receiver midiOut)
   {
      this.knob = knob;
      this.midiOut = midiOut;

      controlVariable = new SliderboardVariable(knob.getMin(), knob.getMax());
   }

   public boolean handleMessage(ShortMessage message, long timestamp)
   {
      if (knob.getChannel() != message.getData1()) // Should it use getChannel instead
         return false;

      newKnobValue = MathTools.clamp(message.getData2(), knob.getMin(), knob.getMax());
      return true;
   }

   public void moveSlider()
   {
      currentKnobValue = controlVariable.getValue(); // TODO Not sure if that's the best way
      moveSlider(controlVariable.getValue());
   }

   public void moveSlider(int value)
   {
      if (value == -1)
         return;

      try
      {
         ShortMessage message = new ShortMessage();
         message.setMessage(176, 0, knob.getChannel(), value);
         midiOut.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         e.printStackTrace();
      }
   }

   public void update()
   {
      if (newKnobValue != -1)
      {
         currentKnobValue = newKnobValue;
      }

      if (newKnobValue != -1)
      {
         controlVariable.setValue(newKnobValue);
         newKnobValue = -1;
      }
      else if (controlVariable.getValue() != currentKnobValue)
      {
         moveSlider();
      }
   }

   public SliderboardVariable getControlVariable()
   {
      return controlVariable;
   }
}
