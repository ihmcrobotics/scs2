package us.ihmc.scs2.symbolic.parser;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.symbolic.EquationBuilder;
import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.InputType;
import us.ihmc.scs2.symbolic.EquationInput.ScalarInput;
import us.ihmc.scs2.symbolic.parser.EquationParseError.ProblemType;
import us.ihmc.scs2.symbolic.parser.EquationToken.TokenType;

import java.util.ArrayList;
import java.util.List;

public class EquationParser
{
   private final EquationAliasManager aliasManager = new EquationAliasManager();
   private final EquationOperationFactoryLibrary operationLibrary = new EquationOperationFactoryLibrary();

   public EquationParser()
   {
   }

   public EquationAliasManager getAliasManager()
   {
      return aliasManager;
   }

   public EquationOperationFactoryLibrary getOperationLibrary()
   {
      return operationLibrary;
   }

   // TODO Need to add a way to add constants like immutable.

   /**
    * Parses the string representing an equation and compiles it into a {@code YoEquation} which can be executed later on.
    *
    * @param equationString the equation to parse.
    * @return the parsed equation.
    */
   public EquationBuilder parse(String equationString)
   {
      return parse(equationString, true);
   }

   /**
    * Parses the string representing an equation and compiles it into a {@code YoEquation} which can be executed later on.
    *
    * @param equationString the equation to parse.
    * @param assignment     if true, then the equation is assumed to be an assignment to an output variable.
    * @return the parsed equation.
    */
   public EquationBuilder parse(String equationString, boolean assignment)
   {
      List<EquationToken> tokens = tokenizeEquation(equationString);

      if (tokens.size() < 3)
         throw new EquationParseError("Too few tokens", ProblemType.TOO_FEW_TOKENS, equationString);

      EquationToken t0 = tokens.get(0);

      insertFunctionsAndVariables(tokens);
      EquationBuilder equationBuilder = new EquationBuilder(equationString, aliasManager.duplicate());

      if (t0.getType() != TokenType.VARIABLE && t0.getType() != TokenType.WORD)
      {
         compileTokens(equationString, tokens, equationBuilder);
         // If there's no output, then this is acceptable; otherwise, it's assumed to be a bug.
         EquationInput variable = tokens.get(0).getVariable();
         if (variable != null && assignment)
            throw new IllegalArgumentException("No assignment to an output variable could be found. Found " + t0);
      }
      else
      {
         parseAssignment(equationString, tokens, equationBuilder);
      }
      return equationBuilder;
   }

   /**
    * Compiles the right side of an assignment.
    * The left side is assumed to be a variable.
    * <p>
    * The tokens are expected to represent an equation of the form: "A = ..." where A is a variable and is the first token, the second token being the
    * assignment operator.
    * </p>
    *
    * @param equationString        the initial equation string.
    * @param tokens                the tokens representing the whole equation.
    * @param equationBuilderToPack the operation factories are added to this equation builder.
    */
   private void parseAssignment(String equationString, List<EquationToken> tokens, EquationBuilder equationBuilderToPack)
   {
      EquationToken t0 = tokens.get(0);
      EquationToken t1 = tokens.get(1);

      if (t1.getType() != TokenType.SYMBOL || t1.getSymbol() != EquationSymbol.ASSIGN)
         throw new EquationParseError("Expected assignment operator next", ProblemType.UNEXPECTED_TOKEN_TYPE, equationString, t1);

      // Parse the right side of the equation
      List<EquationToken> tokensRight = tokens.subList(2, tokens.size());

      EquationToken tokenRight;
      if (tokensRight.size() > 1)
      {
         compileTokens(equationString, tokensRight, equationBuilderToPack);

         EquationToken lastToken = tokensRight.get(tokensRight.size() - 1);
         if (lastToken.getType() != TokenType.OPERATION)
            throw new EquationParseError("Something went wrong with parsing the block, the last token should be an operation",
                                         ProblemType.UNEXPECTED_TOKEN_TYPE,
                                         equationString,
                                         tokensRight.get(tokens.size() - 1));
         if (tokensRight.size() != 1)
            throw new EquationParseError("Something went wrong with parsing the block, there should be a single token left",
                                         ProblemType.UNEXPECTED_TOKEN_TYPE,
                                         equationString,
                                         tokensRight.get(tokens.size() - 1));
         tokenRight = lastToken;
      }
      else
      { // A single token, let's check that it is a variable or a number.
         tokenRight = tokensRight.get(0);

         if (tokenRight.getType() == TokenType.OPERATION)
         {
            throw new EquationParseError("Something went wrong with tokenizing the equation, the last token should not be an operation",
                                         ProblemType.UNEXPECTED_TOKEN_TYPE,
                                         equationString,
                                         tokensRight.get(tokens.size() - 1));
         }
         else if (tokenRight.getType() == TokenType.SYMBOL)
         {
            throw new EquationParseError("Something went wrong with tokenizing the equation, the last token should not be a symbol",
                                         ProblemType.INVALID_SYMBOL_USE,
                                         equationString,
                                         tokensRight.get(tokens.size() - 1));
         }
      }

      EquationOperationFactory factory = operationLibrary.get(EquationSymbol.ASSIGN);
      equationBuilderToPack.addOperationFactory(factory, t0, tokenRight);
   }

   /**
    * Compiles the right side of an equation.
    *
    * @param tokens                the tokens representing the whole equation.
    * @param equationBuilderToPack the operation factories are added to this equation builder.
    */
   private void compileTokens(String equationString, List<EquationToken> tokens, EquationBuilder equationBuilderToPack)
   {
      handleParentheses(equationString, tokens, equationBuilderToPack);

      if (tokens.size() > 1)
         parseBlockNoParentheses(equationString, tokens, equationBuilderToPack);

      // see if it needs to be parsed more
      if (tokens.size() != 1)
         throw new RuntimeException("BUG");
   }

   /**
    * Searches for pairs of parentheses and processes blocks inside them.
    * This handles nested parentheses.
    *
    * @param unprocessedTokenStack the list of tokens to process
    * @param equationBuilderToPack the operation factories are added to this equation builder.
    */
   protected void handleParentheses(String equationString, List<EquationToken> unprocessedTokenStack, EquationBuilder equationBuilderToPack)
   {
      // have a list to handle embedded parentheses, e.g. (((((a)))))
      List<Integer> leftIndices = new ArrayList<>();

      // find all of them
      for (int i = 0; i < unprocessedTokenStack.size(); i++)
      {
         EquationToken t = unprocessedTokenStack.get(i);

         if (t.getType() != TokenType.SYMBOL)
            continue;

         if (t.getSymbol() == EquationSymbol.PAREN_LEFT)
         {
            leftIndices.add(i);
            continue;
         }

         if (t.getSymbol() != EquationSymbol.PAREN_RIGHT)
            continue;

         if (leftIndices.isEmpty())
            throw new EquationParseError(") found with no matching (", ProblemType.PARENTHESES_MISMATCH, equationString, t);

         int leftIndex = leftIndices.remove(leftIndices.size() - 1);

         // remember the element before so the new one can be inserted afterward.
         EquationToken beforeLeft = unprocessedTokenStack.get(leftIndex - 1);

         // Sublist with parentheses
         List<EquationToken> sublist = unprocessedTokenStack.subList(leftIndex, i + 1);
         i -= sublist.size();
         // Remove the parentheses
         sublist.remove(0);
         sublist.remove(sublist.size() - 1);

         // if it is a function before "()" then the "()" indicates it is an input to a function
         if (beforeLeft != null && beforeLeft.getType() == TokenType.FUNCTION)
         {
            List<EquationToken> inputs;
            try
            {
               inputs = parseParameterCommaBlock(equationString, sublist, equationBuilderToPack);
            }
            catch (EquationParseError e)
            {
               e.setProblemToken(beforeLeft);
               throw e;
            }

            if (inputs.isEmpty())
               throw new EquationParseError("Empty function input parameters", ProblemType.FUNCTION_MISSING_INPUTS, equationString, beforeLeft);
            parseFunction(beforeLeft, inputs, unprocessedTokenStack, equationBuilderToPack);
         }
         else
         {
            // if null then it was empty inside
            EquationToken output = parseBlockNoParentheses(equationString, sublist, equationBuilderToPack);
            if (sublist.size() != 1 && sublist.get(0) != output)
               throw new EquationParseError("Something went wrong with parsing the block", ProblemType.OTHER, equationString, sublist.get(0));
         }
      }

      if (!leftIndices.isEmpty())
         throw new EquationParseError("Dangling ( parentheses",
                                      ProblemType.PARENTHESES_MISMATCH,
                                      equationString,
                                      unprocessedTokenStack.get(leftIndices.get(0)));
   }

   /**
    * Searches for commas in the set of tokens.
    * Used for inputs to functions.
    * Ignore commas which are inside a "[ ]" block.
    *
    * @return List of output tokens between the commas
    */
   protected List<EquationToken> parseParameterCommaBlock(String equationString, List<EquationToken> tokens, EquationBuilder operationFactoriesToPack)
   {
      // Find the start/end indices for each argument
      List<Integer> limitIndices = new ArrayList<>();
      limitIndices.add(-1);
      for (int i = 0; i < tokens.size(); i++)
      {
         EquationToken token = tokens.get(i);
         if (token.getType() == TokenType.SYMBOL && token.getSymbol() == EquationSymbol.COMMA)
            limitIndices.add(i);
      }
      limitIndices.add(tokens.size());

      List<EquationToken> output = new ArrayList<>();

      for (int i = 0; i < limitIndices.size() - 1; i++)
      {
         int start = limitIndices.get(i) + 1;
         int end = limitIndices.get(i + 1);

         if (start == end)
            throw new EquationParseError("No empty function inputs allowed!", ProblemType.FUNCTION_MISSING_INPUTS, equationString, null);

         output.add(parseBlockNoParentheses(equationString, tokens.subList(start, end), operationFactoriesToPack));
      }

      // Clear the tokens to mark them as processed
      tokens.clear();
      return output;
   }

   /**
    * Parses a code block with no parentheses and no commas. After it is done there should be a single
    * token left, which is returned.
    */
   private EquationToken parseBlockNoParentheses(String equationString, List<EquationToken> tokens, EquationBuilder equationBuilderToPack)
   {
      if (tokens.isEmpty())
         throw new EquationParseError("Empty block", ProblemType.TOO_FEW_TOKENS, equationString);

      // process operators depending on their priority
      EquationSymbol[] orderedOperations = {EquationSymbol.POWER, EquationSymbol.DIVIDE, EquationSymbol.TIMES, EquationSymbol.MINUS, EquationSymbol.PLUS};

      for (int symbolIndex = 0; symbolIndex < orderedOperations.length; symbolIndex++)
      {
         EquationSymbol nextSymbol = orderedOperations[symbolIndex];

         for (int tokenIndex = 0; tokenIndex < tokens.size(); tokenIndex++)
         {
            EquationToken curr = tokens.get(tokenIndex);

            if (curr.symbol != nextSymbol)
               continue;

            EquationToken prev = tokenIndex > 0 ? tokens.get(tokenIndex - 1) : null;
            EquationToken next = tokenIndex + 1 < tokens.size() ? tokens.get(tokenIndex + 1) : null;

            if (prev == null || next == null)
               throw new EquationParseError("Invalid use of symbol: " + curr.symbol.symbolString, ProblemType.INVALID_SYMBOL_USE, equationString, curr);

            EquationOperationFactory factory = operationLibrary.get(curr.symbol);
            equationBuilderToPack.addOperationFactory(factory, prev, next);

            // TODO need to test the token length
            // replace the symbols with their output
            EquationToken outputToken = EquationToken.newOperationToken(factory,
                                                                        prev.equationStringStartIndex,
                                                                        prev.equationStringTokenLength + curr.equationStringTokenLength
                                                                        + next.equationStringTokenLength);
            tokens.remove(tokenIndex + 1);
            tokens.set(tokenIndex, outputToken);
            tokens.remove(tokenIndex - 1);
            tokenIndex -= 2; // Tokens are getting removed from the list
         }
      }

      if (tokens.size() > 1)
      {
         System.err.println("Remaining tokens: " + tokens.size());
         System.err.println(EuclidCoreIOTools.getCollectionString("\n\t", "", "\n\t", tokens));
         throw new IllegalStateException("There should only be a single token left");
      }
      return tokens.get(0);
   }

   /**
    * Adds a new operation to the list from the operation and two variables. The inputs are removed
    * from the token list and replaced by their output.
    *
    * @param functionToken            token with the function name
    * @param inputTokens              list of tokens to use for creating the function inputs
    * @param tokens                   list of tokens to replace the given name token with the actual function
    *                                 output.
    * @param operationFactoriesToPack list of operations to add the new operation to
    * @return the token which replaces the function name
    */
   protected EquationToken parseFunction(EquationToken functionToken,
                                         List<EquationToken> inputTokens,
                                         List<EquationToken> tokens,
                                         EquationBuilder operationFactoriesToPack)
   {
      EquationOperationFactory operationFactory = operationLibrary.get(functionToken.getFunctionName());
      operationFactoriesToPack.addOperationFactory(operationFactory, inputTokens);

      // TODO need to test the token length
      // replace the symbols with the function's output
      EquationToken t = EquationToken.newOperationToken(operationFactory,
                                                        functionToken.equationStringStartIndex,
                                                        functionToken.equationStringTokenLength + inputTokens.stream()
                                                                                                             .mapToInt(EquationToken::getEquationStringTokenLength)
                                                                                                             .sum());
      tokens.set(tokens.indexOf(functionToken), t);
      return t;
   }

   /**
    * Used when parsing a number.
    * <p>
    * Initially, the number is assumed to be an integer until we discover a decimal point at which
    * point it becomes a float. A float can be defined with a scientific format "1.0e+10", the exponent
    * has to be an integer.
    * </p>
    */
   private enum NumberType
   {
      /**
       * The number being read is assumed to be an integer, can change to float.
       */
      INTEGER,
      /**
       * A decimal point has been read, the number is a float.
       */
      FLOAT,
      /**
       * An exponent has been read, the number is a float and we're reading the exponent number.
       */
      FLOAT_EXP,
   }

   /**
    * Parses the text string to extract tokens.
    */
   private static List<EquationToken> tokenizeEquation(String equationString)
   {
      equationString += " ";

      List<EquationToken> tokens = new ArrayList<>();

      for (int i = 0; i < equationString.length(); i++)
      {
         char c = equationString.charAt(i);

         if (Character.isWhitespace(c))
            continue;

         EquationSymbol next = EquationSymbol.lookupSymbolAtStart(equationString.substring(i));

         if (next != null)
         {
            if (!EquationSymbol.isSymbolSupported(next))
               throw new EquationParseError("Symbol (%s) is not yet supported.".formatted(next.symbolString),
                                            ProblemType.UNSUPPORTED_SYMBOL,
                                            equationString,
                                            i);

            EquationToken last = tokens.isEmpty() ? null : tokens.get(tokens.size() - 1);

            if (last != null && last.getType() == TokenType.SYMBOL && !EquationSymbol.isSymbolDuoValid(last.symbol, next))
               throw new EquationParseError("Invalid sequence of symbols: (%s) followed by (%s).".formatted(last.symbol.symbolString, next.symbolString),
                                            ProblemType.INVALID_SYMBOL_USE,
                                            equationString,
                                            i);

            boolean isSymbol = true;
            if (next == EquationSymbol.MINUS || next == EquationSymbol.PLUS)
            {
               if (i + 1 < equationString.length() && Character.isDigit(equationString.charAt(i + 1)) && (last == null || isOperatorLR(last.symbol)))
                  isSymbol = false;
            }

            if (isSymbol)
            {
               tokens.add(EquationToken.newSymbolToken(next, i, next.symbolString.length()));
               i += next.symbolString.length() - 1;
               continue;
            }
         }

         if (next == EquationSymbol.MINUS || next == EquationSymbol.PLUS || Character.isDigit(c))
         {
            int start = i;
            NumberType type = NumberType.INTEGER;

            while (i + 1 < equationString.length())
            {
               c = equationString.charAt(i + 1);

               if (Character.isWhitespace(c))
                  break;

               if (Character.isDigit(c))
               {
                  i++;
               }
               else if (c == '.')
               {
                  if (type == NumberType.FLOAT)
                     throw new EquationParseError("Can't have 2 decimal points.", ProblemType.INVALID_NUMBER_FORMAT, equationString, i);
                  if (type == NumberType.FLOAT_EXP)
                     throw new EquationParseError("Float exponent must be an integer.", ProblemType.INVALID_NUMBER_FORMAT, equationString, i);

                  type = NumberType.FLOAT;
                  i++;
               }
               else if (Character.toLowerCase(c) == 'e')
               {
                  if (type == NumberType.FLOAT_EXP)
                     throw new EquationParseError("Can't have 2 exponent.", ProblemType.INVALID_NUMBER_FORMAT, equationString, i);

                  type = NumberType.FLOAT_EXP;
                  i++;

                  // +/- Symbol typically follows exponent symbol
                  if (equationString.charAt(i + 1) == '-' || equationString.charAt(i + 1) == '+')
                     i++;
               }
               else
               {
                  break;
               }
            }

            InputType inputType = type == NumberType.INTEGER ? InputType.INTEGER : InputType.DOUBLE;
            ScalarInput constant = EquationInput.parseConstant(inputType, equationString.substring(start, i + 1));
            tokens.add(EquationToken.newVariableToken(constant, start, i + 1 - start));
            continue;
         }

         if (isLetter(c))
         {
            int start = i;

            while (i + 1 < equationString.length())
            {
               c = equationString.charAt(i + 1);

               if (isLetter(c))
                  i++;
               else
                  break;
            }

            tokens.add(EquationToken.newWordToken(equationString.substring(start, i + 1), start, i + 1 - start));
            continue;
         }

         throw new EquationParseError("Unexpected char (%c) at position (%d) in equation: %s".formatted(c, i, equationString),
                                      ProblemType.OTHER,
                                      equationString,
                                      i);
      }

      return tokens;
   }

   /**
    * Search for WORDS in the token list. Then see if the WORD is a function or a variable. If so
    * replace the work with the function/variable
    */
   private void insertFunctionsAndVariables(List<EquationToken> tokens)
   {
      for (int i = 0; i < tokens.size(); i++)
      {
         EquationToken token = tokens.get(i);

         if (token.getType() == TokenType.WORD)
         {
            if (operationLibrary.isFunctionName(token.word))
               tokens.set(i, EquationToken.newFunctionToken(token.word, token.equationStringStartIndex, token.equationStringTokenLength));
         }
      }
   }

   /**
    * Operators which affect the variables to its left and right
    */
   private static boolean isOperatorLR(EquationSymbol s)
   {
      if (s == null)
         return false;

      return switch (s)
      {
         case DIVIDE, TIMES, POWER, PLUS, MINUS, ASSIGN -> true;
         default -> false;
      };
   }

   /**
    * Returns true if the character is a valid letter for use in a variable name
    */
   private static boolean isLetter(char c)
   {
      return !(EquationSymbol.isSymbol(c) || Character.isWhitespace(c));
   }

   /**
    * Returns true if the specified name is NOT allowed. It isn't allowed if it matches a built-in
    * operator or if it contains a restricted character.
    */
   private boolean isReserved(String name)
   {
      if (operationLibrary.isFunctionName(name))
         return true;

      for (int i = 0; i < name.length(); i++)
      {
         if (!isLetter(name.charAt(i)))
            return true;
      }
      return false;
   }
}
