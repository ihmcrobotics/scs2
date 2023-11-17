package us.ihmc.scs2.symbolic.parser;

import java.util.Arrays;
import java.util.List;

public enum EquationSymbol
{
   PLUS('+', "Addition"),
   MINUS('-', "Subtraction"),
   TIMES('*', "Multiplication"),
   DIVIDE('/', "Division"),
   POWER('^', "Power"),
   PERIOD('.', "Not supported for now"),
   ASSIGN('=', "Assignment"),
   PAREN_LEFT('(', "Parenthesis (opening)"),
   PAREN_RIGHT(')', "Parenthesis (closing)"),
   BRACKET_LEFT('[', "Not supported for now"),
   BRACKET_RIGHT(']', "Not supported for now"),
   GREATER_THAN('>', "Not supported for now"),
   LESS_THAN('<', "Not supported for now"),
   GREATER_THAN_EQ(">=", "Not supported for now"),
   LESS_THAN_EQ("<=", "Not supported for now"),
   COMMA(',', "Separator for function inputs"),
   COLON(':', "Not supported for now"),
   SEMICOLON(';', "Not supported for now");

   final boolean isSingleChar;
   final char symbolChar;
   final String symbolString;
   final String description;

   EquationSymbol(char symbolChar, String description)
   {
      isSingleChar = true;
      this.symbolChar = symbolChar;
      this.symbolString = new String(new char[] {symbolChar});
      this.description = description;
   }

   EquationSymbol(String symbolString, String description)
   {
      isSingleChar = false;
      this.symbolChar = 0;
      this.symbolString = symbolString;
      this.description = description;
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
         case GREATER_THAN:
         case LESS_THAN:
         case GREATER_THAN_EQ:
         case LESS_THAN_EQ:
         case COLON:
         case SEMICOLON:
            yield false;
         default:
            yield true;
      };
   }

   public static List<EquationSymbol> getSupportedSymbols()
   {
      return Arrays.stream(values()).filter(EquationSymbol::isSymbolSupported).toList();
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

   public String getSymbolString()
   {
      return symbolString;
   }

   public String getDescription()
   {
      return description;
   }
}