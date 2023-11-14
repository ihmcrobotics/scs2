package us.ihmc.scs2.symbolic.parser;

public class EquationParseError extends RuntimeException
{
   private static final long serialVersionUID = -8531642550464632855L;

   public enum ProblemType
   {TOO_FEW_TOKENS, UNEXPECTED_TOKEN_TYPE, PARENTHESES_MISMATCH, FUNCTION_MISSING_INPUTS, UNSUPPORTED_SYMBOL, INVALID_SYMBOL_USE, INVALID_NUMBER_FORMAT, OTHER}

   private final ProblemType problemType;
   private final String equationString;
   private EquationToken problemToken;
   private int problemIndex;

   public EquationParseError(String message, ProblemType problemType, String equationString)
   {
      super(message);
      this.equationString = equationString;
      this.problemType = problemType;
   }

   public EquationParseError(String message, ProblemType problemType, String equationString, EquationToken problemToken)
   {
      super(message);
      this.problemType = problemType;
      this.equationString = equationString;
      this.problemToken = problemToken;
      problemIndex = problemToken == null ? -1 : problemToken.getEquationStringStartIndex();
   }

   public EquationParseError(String message, ProblemType problemType, String equationString, int problemIndex)
   {
      super(message);
      this.problemType = problemType;
      this.equationString = equationString;
      this.problemIndex = problemIndex;
   }

   public void setProblemToken(EquationToken problemToken)
   {
      this.problemToken = problemToken;
   }

   public ProblemType getProblemType()
   {
      return problemType;
   }

   public String getEquationString()
   {
      return equationString;
   }

   public EquationToken getProblemToken()
   {
      return problemToken;
   }

   public int getProblemIndex()
   {
      return problemIndex;
   }
}