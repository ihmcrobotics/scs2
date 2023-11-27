package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary.*;

import java.util.Arrays;
import java.util.List;

public enum EquationSymbol
{
   PLUS('+', AddOperation.NAME, "Addition"),
   MINUS('-', SubtractOperation.NAME, "Subtraction"),
   TIMES('*', MultiplyOperation.NAME, "Multiplication"),
   DIVIDE('/', DivideOperation.NAME, "Division"),
   POWER('^', PowerDoubleOperation.NAME, "Power"),
   PERIOD('.', null, "Not supported for now"),
   ASSIGN('=', AssignmentOperation.NAME, "Assignment"),
   PAREN_LEFT('(', null, "Parenthesis (opening)"),
   PAREN_RIGHT(')', null, "Parenthesis (closing)"),
   BRACKET_LEFT('[', null, "Not supported for now"),
   BRACKET_RIGHT(']', null, "Not supported for now"),
   GREATER_THAN('>', null, "Not supported for now"),
   LESS_THAN('<', null, "Not supported for now"),
   GREATER_THAN_EQ(">=", null, "Not supported for now"),
   LESS_THAN_EQ("<=", null, "Not supported for now"),
   COMMA(',', null, "Separator for function inputs"),
   COLON(':', null, "Not supported for now"),
   SEMICOLON(';', null, "Not supported for now");

   final boolean isSingleChar;
   final char symbolChar;
   final String symbolString;
   final String operationName;
   final String description;

   EquationSymbol(char symbolChar, String operationName, String description)
   {
      isSingleChar = true;
      this.symbolChar = symbolChar;
      this.symbolString = new String(new char[] {symbolChar});
      this.operationName = operationName;
      this.description = description;
   }

   EquationSymbol(String symbolString, String operationName, String description)
   {
      isSingleChar = false;
      this.symbolChar = 0;
      this.symbolString = symbolString;
      this.operationName = operationName;
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
         case PAREN_RIGHT -> Arrays.asList(PLUS, MINUS, TIMES, DIVIDE, POWER, PAREN_RIGHT, GREATER_THAN, LESS_THAN, GREATER_THAN_EQ, LESS_THAN_EQ, COMMA)
                                   .contains(secondSymbol);
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