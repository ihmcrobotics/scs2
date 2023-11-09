package us.ihmc.scs2.symbolic.parser.parser;

public class EquationParseError extends RuntimeException
{
   private static final long serialVersionUID = -8531642550464632855L;

   public EquationParseError(String message)
   {
      super(message);
   }
}