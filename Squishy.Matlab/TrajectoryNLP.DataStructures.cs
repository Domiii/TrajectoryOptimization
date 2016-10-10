using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SparseTrajTensor = Squishy.Matlab.SparseMatlabTensor<Squishy.Matlab.TrajectoryNLP.QInstanceIndex>;

namespace Squishy.Matlab
{
    public partial class TrajectoryNLP
    {
        public object ToMatrix(SparseTrajTensor tensor)
        {
            // one row per quantity, one column per constraint
            var totalSize = TDef.TotalTrajectorySize;
            var nq = TDef.TotalQuantityCount;
            StringBuilder sb = new StringBuilder(nq * 20);

            TrajectoryInstance colContent = new TrajectoryInstance(this);

            sb.Append("[");
            if (tensor.Count > 0)
            {
                var c1 = tensor.First().Key.Constraint;
                colContent.Reset(c1.NDims);
                var lastCol = c1.Index;
                bool added = false;
                foreach (var entry in tensor)
                {
                    var nDims = entry.Key.Constraint.NDims;
                    var col = entry.Key.Constraint.Index;
                    var row = entry.Key.Q.GlobalIndex;

                    if (col != lastCol)
                    {
                        if (added)
                        {
                            added = false;
                            sb.Append(colContent.ToString());              // append column to matrix
							sb.Append(",...\n");
                        }
                        else
                        {
                            --lastCol;
                        }
                        colContent.Reset(nDims);

                        // fill in empty columns
                        if (col - lastCol > 1)
                        {
                            sb.Append(M.Zeros(totalSize, col - lastCol + 1));
                            sb.Append(",...\n");
                        }
                        lastCol = col;
                    }
                    if (entry.Value.Count > 1)
                    {
                        colContent.SetValue(entry.Key.Q, M.Sum(M.ColVector(entry.Value)));
                    }
                    else
                    {
                        colContent.SetValue(entry.Key.Q, entry.Value[0]);
                    }
                    added = true;
                }
                sb.Append(colContent.ToString());              // append last column to matrix
            }
            sb.Append("]");
            return sb.ToString();
        }


        public struct Constraint
        {
            public int Index;
            public int NDims;
        }

        /// <summary>
        /// Used to associate a quantity instance with an index, e.g. for gradient tensors
        /// </summary>
        public struct QInstanceIndex : IComparable
        {
            public Constraint Constraint;
            public TrajectoryQuantityInstance Q;

            public QInstanceIndex(Constraint ci, TrajectoryQuantityInstance q)
            {
                Constraint = ci;
                Q = q;
            }

            public override bool Equals(object obj)
            {
                if (obj is QInstanceIndex)
                {
                    var rhs = (QInstanceIndex)obj;
                    return Constraint.Index == rhs.Constraint.Index && Q.Equals(rhs.Q);
                }
                return false;
            }

            public override int GetHashCode()
            {
                return (Constraint.Index.GetHashCode() & ((1 << 16) - 1)) | Q.GetHashCode() << 16;
            }

            int IComparable.CompareTo(object obj)
            {
                if (obj is QInstanceIndex)
                {
                    var rhs = (QInstanceIndex)obj;
                    return ((Constraint.Index - rhs.Constraint.Index) << 16) + (Q.GlobalIndex - rhs.Q.GlobalIndex);
                }
                return 1;
            }
        }
    }
}
