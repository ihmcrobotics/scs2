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
   public String name;
   public String description;
   public String equationString;

   private final EquationAliasManager aliasManager;
   private final List<EquationOperationFactory> operationFactories = new ArrayList<>();

   public EquationBuilder(String equationString, EquationAliasManager aliasManager)
   {
      this.equationString = equationString;
      this.aliasManager = aliasManager;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setDescription(String description)
   {
      this.description = description;
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

   public Equation build()
   {
      if (aliasManager.getMissingInputs().size() > 0)
         throw new EquationBuilderException("Missing inputs: " + aliasManager.getMissingInputs());
      List<? extends EquationOperation<?>> operations = operationFactories.stream().map(EquationOperationFactory::build).toList();
      return new Equation(name, description, equationString, operations);
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
