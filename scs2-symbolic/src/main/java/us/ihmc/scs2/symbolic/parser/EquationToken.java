package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;

/**
 * The token class contains a reference to parsed data (e.g. function, variable, or symbol) and
 * reference to list elements before and after it.
 */
public class EquationToken
{
   /**
    * Specifies the type of data stored in a Token.
    */
   public enum Type
   {
      /**
       * The token has been identified as referring to a function.
       */
      FUNCTION,
      /**
       * The token has been resolved as a variable, can either be a constant or a variable.
       */
      VARIABLE,
      /**
       * The token has been resolved as a symbol (*, /, +, -, etc.)
       */
      SYMBOL,
      /**
       * The token has been resolved as an operation (=, +=, -=, etc.), the operation once created will provide the result for the next operation.
       */
      OPERATION,
      /**
       * A word is a {@code String} that has not been resolved yet and can be the name of a variable or a
       * function.
       */
      WORD;
   }

   public final String functionName;
   public final EquationInput variable;
   public final EquationSymbol symbol;
   public final EquationOperationFactory operationFactory;
   public final String word;

   /**
    * Allows to locate the start position of the token in the equation string.
    */
   public final int equationStringStartIndex;
   /**
    * Allows to identify the length of the token in the equation string.
    */
   public final int equationStringTokenLength;

   public static EquationToken newFunctionToken(String functionName, int equationStringStartIndex, int equationStringTokenLength)
   {
      return new EquationToken(functionName, null, null, null, null, equationStringStartIndex, equationStringTokenLength);
   }

   public static EquationToken newVariableToken(EquationInput variable, int equationStringStartIndex, int equationStringTokenLength)
   {
      return new EquationToken(null, variable, null, null, null, equationStringStartIndex, equationStringTokenLength);
   }

   public static EquationToken newSymbolToken(EquationSymbol symbol, int equationStringStartIndex, int equationStringTokenLength)
   {
      return new EquationToken(null, null, symbol, null, null, equationStringStartIndex, equationStringTokenLength);
   }

   public static EquationToken newOperationToken(EquationOperationFactory operationFactory, int equationStringStartIndex, int equationStringTokenLength)
   {
      return new EquationToken(null, null, null, operationFactory, null, equationStringStartIndex, equationStringTokenLength);
   }

   public static EquationToken newWordToken(String word, int equationStringStartIndex, int equationStringTokenLength)
   {
      return new EquationToken(null, null, null, null, word, equationStringStartIndex, equationStringTokenLength);
   }

   private EquationToken(String functionName,
                         EquationInput variable,
                         EquationSymbol symbol,
                         EquationOperationFactory operationFactory,
                         String word,
                         int equationStringStartIndex,
                         int equationStringTokenLength)
   {
      this.functionName = functionName;
      this.variable = variable;
      this.symbol = symbol;
      this.operationFactory = operationFactory;
      this.word = word;
      this.equationStringStartIndex = equationStringStartIndex;
      this.equationStringTokenLength = equationStringTokenLength;
   }

   public Type getType()
   {
      if (functionName != null)
         return Type.FUNCTION;
      else if (variable != null)
         return Type.VARIABLE;
      else if (operationFactory != null)
         return Type.OPERATION;
      else if (word != null)
         return Type.WORD;
      else
         return Type.SYMBOL;
   }

   public EquationInput getVariable()
   {
      return variable;
   }

   public String getFunctionName()
   {
      return functionName;
   }

   public EquationOperationFactory getOperationFactory()
   {
      return operationFactory;
   }

   public EquationSymbol getSymbol()
   {
      return symbol;
   }

   public String getWord()
   {
      return word;
   }

   public int getEquationStringStartIndex()
   {
      return equationStringStartIndex;
   }

   public int getEquationStringTokenLength()
   {
      return equationStringTokenLength;
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
         case OPERATION:
            return "Operation:" + operationFactory.name;
         case WORD:
            return "Word:" + word;
      }
      throw new RuntimeException("Unknown type");
   }
}