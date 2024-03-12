package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.Collection;
import java.util.List;

/**
 * Interface to indicate that the implementing class is an accessor to input devices such as a keyboard or a mouse.
 */
public interface InputAccessor
{
   /**
    * Get the list of input accesses to be documented.
    *
    * @return the list of input accesses to be documented.
    */
   List<InputAccessDoc> getAvailableInputAccesses();

   /**
    * Record to document the access to an input device.
    * <p>
    * One record per access to an input device is expected.
    * </p>
    *
    * @param guiElement        the name of the GUI element using the input.
    * @param deviceName        the name of the device being accessed.
    * @param inputNames        the name of the inputs triggering the action.
    * @param actionDescription a description of the resulting action.
    */
   record InputAccessDoc(String guiElement, String deviceName, Collection<String> inputNames, String actionDescription)
   {
      public InputAccessDoc(String guiElement, String deviceName, String inputName, String actionDescription)
      {
         this(guiElement, deviceName, List.of(inputName), actionDescription);
      }

      public static String MOUSE = "Mouse";
      public static String KEYBOARD = "Keyboard";
   }
}
