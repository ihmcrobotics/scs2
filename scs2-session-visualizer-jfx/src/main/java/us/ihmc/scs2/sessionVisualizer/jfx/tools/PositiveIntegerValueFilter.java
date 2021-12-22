package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.function.UnaryOperator;

import javafx.scene.control.TextFormatter.Change;

public class PositiveIntegerValueFilter implements UnaryOperator<Change>
{
   @Override
   public Change apply(Change change)
   {
      String newText = change.getControlNewText();
      return newText.matches("[0-9]*") ? change : null;
   }
}