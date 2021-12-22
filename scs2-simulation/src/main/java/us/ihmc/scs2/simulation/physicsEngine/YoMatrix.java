package us.ihmc.scs2.simulation.physicsEngine;

import org.ejml.data.DMatrix;
import org.ejml.data.Matrix;
import org.ejml.data.MatrixType;
import org.ejml.data.ReshapeMatrix;
import org.ejml.ops.MatrixIO;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoMatrix implements DMatrix, ReshapeMatrix
{
   private static final long serialVersionUID = 2156411740647948028L;

   private final int maxNumberOfRows, maxNumberOfColumns;

   private final YoInteger numberOfRows, numberOfColumns;
   private final YoDouble[][] variables;

   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, YoRegistry registry)
   {
      this.maxNumberOfRows = maxNumberOfRows;
      this.maxNumberOfColumns = maxNumberOfColumns;

      this.numberOfRows = new YoInteger(name + "NumRows", registry);
      this.numberOfColumns = new YoInteger(name + "NumCols", registry);

      this.numberOfRows.set(maxNumberOfRows);
      this.numberOfColumns.set(maxNumberOfColumns);

      variables = new YoDouble[maxNumberOfRows][maxNumberOfColumns];

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            variables[row][column] = new YoDouble(name + "_" + row + "_" + column, registry);
         }
      }
   }

   @Override
   public void reshape(int numRows, int numCols)
   {
      if (numRows > maxNumberOfRows)
         throw new IllegalArgumentException("Too many rows. Expected less or equal to " + maxNumberOfRows + ", was " + numRows);
      else if (numCols > maxNumberOfColumns)
         throw new IllegalArgumentException("Too many columns. Expected less or equal to " + maxNumberOfColumns + ", was " + numCols);
      else if (numRows < 0 || numCols < 0)
         throw new IllegalArgumentException("Cannot reshape with a negative number of rows or columns.");

      numberOfRows.set(numRows);
      numberOfColumns.set(numCols);

      for (int row = 0; row < numRows; row++)
      {
         for (int col = numCols; col < maxNumberOfColumns; col++)
         {
            unsafe_set(row, col, Double.NaN);
         }
      }

      for (int row = numRows; row < maxNumberOfRows; row++)
      {
         for (int col = 0; col < maxNumberOfColumns; col++)
         {
            unsafe_set(row, col, Double.NaN);
         }
      }
   }

   @Override
   public void set(int row, int col, double val)
   {
      if (col < 0 || col >= getNumCols() || row < 0 || row >= getNumRows())
         throw new IllegalArgumentException("Specified element is out of bounds: (" + row + " , " + col + ")");
      unsafe_set(row, col, val);
   }

   @Override
   public void unsafe_set(int row, int col, double val)
   {
      variables[row][col].set(val);
   }

   @Override
   public double get(int row, int col)
   {
      if (col < 0 || col >= getNumCols() || row < 0 || row >= getNumRows())
         throw new IllegalArgumentException("Specified element is out of bounds: (" + row + " , " + col + ")");
      return unsafe_get(row, col);
   }

   @Override
   public double unsafe_get(int row, int col)
   {
      return variables[row][col].getValue();
   }

   @Override
   public void set(Matrix original)
   {
      if (original instanceof DMatrix)
      {
         DMatrix otherMatrix = (DMatrix) original;
         reshape(otherMatrix.getNumRows(), otherMatrix.getNumRows());
         for (int row = 0; row < getNumRows(); row++)
         {
            for (int col = 0; col < getNumCols(); col++)
            {
               set(row, col, otherMatrix.unsafe_get(row, col));
            }
         }
      }
   }

   @Override
   public void zero()
   {
      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            variables[row][col].set(0.0);
         }
      }
   }

   public void setToNaN(int numRows, int numCols)
   {
      reshape(numRows, numCols);
      for (int row = 0; row < numRows; row++)
      {
         for (int col = 0; col < numCols; col++)
         {
            unsafe_set(row, col, Double.NaN);
         }
      }
   }

   @Override
   public int getNumRows()
   {
      return numberOfRows.getValue();
   }

   @Override
   public int getNumCols()
   {
      return numberOfColumns.getValue();
   }

   @Override
   public int getNumElements()
   {
      return getNumRows() * getNumCols();
   }

   @Override
   public MatrixType getType()
   {
      return MatrixType.UNSPECIFIED;
   }

   @Override
   public void print()
   {
      MatrixIO.printFancy(System.out, this, MatrixIO.DEFAULT_LENGTH);
   }

   @Override
   public void print(String format)
   {
      MatrixIO.print(System.out, this, format);
   }

   @Override
   public <T extends Matrix> T createLike()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public <T extends Matrix> T create(int numRows, int numCols)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public <T extends Matrix> T copy()
   {
      throw new UnsupportedOperationException();
   }
}
