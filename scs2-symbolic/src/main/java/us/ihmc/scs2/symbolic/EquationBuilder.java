package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.symbolic.parser.EquationAliasManager;
import us.ihmc.scs2.symbolic.parser.EquationOperation;
import us.ihmc.scs2.symbolic.parser.EquationOperationFactory;
import us.ihmc.scs2.symbolic.parser.EquationToken;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class EquationBuilder
{
   private final String equationString;

   private final EquationAliasManager aliasManager;
   private final List<EquationOperationFactory> operationFactories = new ArrayList<>();

   public EquationBuilder(String equationString, EquationAliasManager aliasManager)
   {
      this.equationString = equationString;
      this.aliasManager = aliasManager;
   }

   public void addOperationFactory(EquationOperationFactory operationFactory, EquationToken... tokens)
   {
      addOperationFactory(operationFactory, List.of(tokens));
   }

   public void addOperationFactory(EquationOperationFactory operationFactory, Collection<EquationToken> tokens)
   {
      operationFactory.setInputs(aliasManager.submitInputRequest(tokens));
      operationFactories.add(operationFactory);
   }

   public EquationAliasManager getAliasManager()
   {
      return aliasManager;
   }

   public EquationBuilder duplicate()
   {
      EquationBuilder duplicate = new EquationBuilder(equationString, aliasManager.duplicate());
      duplicate.operationFactories.addAll(operationFactories);
      return duplicate;
   }

   public boolean isReady()
   {
      return aliasManager.getMissingInputs().isEmpty();
   }

   public List<? extends EquationOperation<?>> build()
   {
      if (!aliasManager.getMissingInputs().isEmpty())
         throw new EquationBuilderException("Missing inputs: " + aliasManager.getMissingInputs());
      return operationFactories.stream().map(EquationOperationFactory::build).toList();
   }

   public String getEquationString()
   {
      return equationString;
   }

   public static class EquationBuilderException extends RuntimeException
   {
      private static final long serialVersionUID = 1L;

      public EquationBuilderException(String message)
      {
         super(message);
      }
   }
}
