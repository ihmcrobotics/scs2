package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoNamespace;

public class YoCompositeTest
{

   @Test
   public void testComputeUniqueNames()
   {
      int numberOfVariables = 21;
      YoNamespace[] namespaces = new YoNamespace[numberOfVariables];
      int i = 0;
      namespaces[i++] = new YoNamespace("short.caramel");
      namespaces[i++] = new YoNamespace("un.miam.caramel");
      namespaces[i++] = new YoNamespace("deux.miam.caramel");
      namespaces[i++] = new YoNamespace("trois.different.caramel");
      namespaces[i++] = new YoNamespace("quatre.yummy.caramel");
      namespaces[i++] = new YoNamespace("cinq.choup.caramel");
      namespaces[i++] = new YoNamespace("six.yummy.caramel");
      namespaces[i++] = new YoNamespace("sept.mouais.caramel");
      namespaces[i++] = new YoNamespace("huit.cest.cacao.chocolat");
      namespaces[i++] = new YoNamespace("neuf.pas.cacao.chocolat");
      namespaces[i++] = new YoNamespace("dix.pareil.cafe.chocolat");
      namespaces[i++] = new YoNamespace("onze.la.cafe.chocolat");
      namespaces[i++] = new YoNamespace("douze.onsenfoula.demisel.beurre");
      namespaces[i++] = new YoNamespace("treize.onsenfoula.nature.beurre");
      namespaces[i++] = new YoNamespace("quatorze.quelquechose.nappe.table.sel");
      namespaces[i++] = new YoNamespace("quinze.quelquechose.poivre.table.sel");
      namespaces[i++] = new YoNamespace("seize.toi.maison.chat.miaou");
      namespaces[i++] = new YoNamespace("dixsept.moi.maison.chat.miaou");
      namespaces[i++] = new YoNamespace("dixhuit.serpent");
      namespaces[i++] = new YoNamespace("dixneuf.giraffe");
      namespaces[i++] = new YoNamespace("vingt.chat");

      String[] varNames = new String[numberOfVariables];

      for (int j = 0; j < numberOfVariables; j++)
      {
         YoNamespace namespace = namespaces[j];
         varNames[j] = namespace.getShortName();
         List<String> subNames = namespace.getSubNames();
         namespaces[j] = new YoNamespace(subNames.subList(0, subNames.size() - 1));
      }

      List<YoComposite> yoTypeReferences = new ArrayList<>();

      for (int j = 0; j < numberOfVariables; j++)
      {
         yoTypeReferences.add(new YoComposite(null, varNames[j], namespaces[j], null));
      }

      YoComposite.computeUniqueNames(yoTypeReferences);

      Set<String> uniqueNameSet = new HashSet<>();

      for (YoComposite yoTypeReference : yoTypeReferences)
      {
         assertTrue(uniqueNameSet.add(yoTypeReference.getUniqueName()));
      }

      i = 0;
      assertEquals("short.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("un.miam.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("deux.miam.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("different.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("quatre.yummy.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("choup.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("six.yummy.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("mouais.caramel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("cest.cacao.chocolat", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("pas.cacao.chocolat", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("pareil.cafe.chocolat", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("la.cafe.chocolat", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("demisel.beurre", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("nature.beurre", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("nappe.table.sel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("poivre.table.sel", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("toi.maison.chat.miaou", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("moi.maison.chat.miaou", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("serpent", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("giraffe", yoTypeReferences.get(i++).getUniqueName());
      assertEquals("chat", yoTypeReferences.get(i++).getUniqueName());
   }
}
