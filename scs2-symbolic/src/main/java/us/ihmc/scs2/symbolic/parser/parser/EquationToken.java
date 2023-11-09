package us.ihmc.scs2.symbolic.parser.parser;

import us.ihmc.scs2.symbolic.parser.EquationVariable;

/**
 * The token class contains a reference to parsed data (e.g. function, variable, or symbol) and
 * reference to list elements before and after it.
 */
class EquationToken
{
   /**
    * Specifies the type of data stored in a Token.
    */
   public enum Type
   {
      /** The token has been identified as referring to a function. */
      FUNCTION,
      /** The token has been resolved as a variable, can either be a constant or a variable. */
      VARIABLE,
      /** The token has been resolved as a symbol (*, /, +, -, etc.) */
      SYMBOL,
      /**
       * A word is a {@code String} that has not been resolved yet and can be the name of a variable or a
       * function.
       */
      WORD;
   }

   public final String functionName;
   public final EquationVariable variable;
   public final EquationSymbol symbol;
   public final String word;

   public static EquationToken newFunctionToken(String functionName)
   {
      return new EquationToken(functionName, null, null, null);
   }

   public static EquationToken newVariableToken(EquationVariable variable)
   {
      return new EquationToken(null, variable, null, null);
   }

   public static EquationToken newSymbolToken(EquationSymbol symbol)
   {
      return new EquationToken(null, null, symbol, null);
   }

   public static EquationToken newWordToken(String word)
   {
      return new EquationToken(null, null, null, word);
   }

   private EquationToken(String functionName, EquationVariable variable, EquationSymbol symbol, String word)
   {
      this.functionName = functionName;
      this.variable = variable;
      this.symbol = symbol;
      this.word = word;
   }

   public Type getType()
   {
      if (functionName != null)
         return Type.FUNCTION;
      else if (variable != null)
         return Type.VARIABLE;
      else if (word != null)
         return Type.WORD;
      else
         return Type.SYMBOL;
   }

   public EquationVariable getVariable()
   {
      return variable;
   }

   public String getFunctionName()
   {
      return functionName;
   }

   public EquationSymbol getSymbol()
   {
      return symbol;
   }

   public String getWord()
   {
      return word;
   }

   @Override
   public String toString()
   {
      switch (getType())
      {
         case FUNCTION:
            return "Func:" + functionName;
         case SYMBOL:
            return "" + symbol;
         case VARIABLE:
            return variable.toString();
         case WORD:
            return "Word:" + word;
      }
      throw new RuntimeException("Unknown type");
   }
}