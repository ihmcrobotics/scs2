package us.ihmc.scs2.symbolic.parser;

import java.util.Arrays;

public enum EquationSymbol
{
   PLUS('+'),
   MINUS('-'),
   TIMES('*'),
   DIVIDE('/'),
   POWER('^'),
   PERIOD('.'),
   ASSIGN('='),
   PAREN_LEFT('('),
   PAREN_RIGHT(')'),
   BRACKET_LEFT('['),
   BRACKET_RIGHT(']'),
   GREATER_THAN('>'),
   LESS_THAN('<'),
   GREATER_THAN_EQ(">="),
   LESS_THAN_EQ("<="),
   COMMA(','),
   COLON(':'),
   SEMICOLON(';');

   final boolean isSingleChar;
   final char symbolChar;
   final String symbolString;

   EquationSymbol(char symbolChar)
   {
      isSingleChar = true;
      this.symbolChar = symbolChar;
      this.symbolString = new String(new char[] {symbolChar});
   }

   EquationSymbol(String symbolString)
   {
      isSingleChar = false;
      this.symbolChar = 0;
      this.symbolString = symbolString;
   }

   public static boolean isSymbol(char symbolChar)
   {
      for (EquationSymbol symbol : values())
      {
         if (symbol.isSingleChar && symbol.symbolChar == symbolChar)
            return true;
      }
      return false;
   }

   public static EquationSymbol lookupSymbolAtStart(String string)
   {
      for (EquationSymbol symbol : values())
      {
         if (string.startsWith(symbol.symbolString))
            return symbol;
      }
      return null;
   }

   public static boolean isSymbolSupported(EquationSymbol symbol)
   {
      return switch (symbol)
      {
         case PERIOD:
         case BRACKET_LEFT:
         case BRACKET_RIGHT:
         case COLON:
         case SEMICOLON:
            yield false;
         default:
            yield true;
      };
   }

   public static boolean isSymbolDuoValid(EquationSymbol firstSymbol, EquationSymbol secondSymbol)
   {
      return switch (firstSymbol)
      {
         case PLUS -> Arrays.asList(PAREN_LEFT).contains(secondSymbol);
         case MINUS -> Arrays.asList(PAREN_LEFT).contains(secondSymbol);
         case TIMES -> Arrays.asList(PAREN_LEFT).contains(secondSymbol);
         case DIVIDE -> Arrays.asList(PAREN_LEFT).contains(secondSymbol);
         case POWER -> Arrays.asList(PAREN_LEFT).contains(secondSymbol);
         case PERIOD -> false;
         case ASSIGN -> Arrays.asList(PLUS, MINUS, PAREN_LEFT).contains(secondSymbol);
         case PAREN_LEFT -> Arrays.asList(PLUS, MINUS, PAREN_LEFT).contains(secondSymbol);
         case PAREN_RIGHT ->
               Arrays.asList(PLUS, MINUS, TIMES, DIVIDE, POWER, PAREN_RIGHT, GREATER_THAN, LESS_THAN, GREATER_THAN_EQ, LESS_THAN_EQ).contains(secondSymbol);
         case BRACKET_LEFT -> false;
         case BRACKET_RIGHT -> false;
         case GREATER_THAN -> Arrays.asList(PLUS, MINUS, PAREN_LEFT).contains(secondSymbol);
         case LESS_THAN -> Arrays.asList(PLUS, MINUS, PAREN_LEFT).contains(secondSymbol);
         case GREATER_THAN_EQ -> Arrays.asList(PLUS, MINUS, PAREN_LEFT).contains(secondSymbol);
         case LESS_THAN_EQ -> Arrays.asList(PLUS, MINUS, PAREN_LEFT).contains(secondSymbol);
         case COMMA -> false;
         case COLON -> false;
         case SEMICOLON -> false;
      };
   }
}