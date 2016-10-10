using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
	public class SparseMatlabTensor<Idx> : SortedDictionary<Idx, List<MatlabExpression>>
	{
		public void Add(Idx idx, MatlabExpression item)
        {
            List<MatlabExpression> list;
            if (!TryGetValue(idx, out list))
            {
                Add(idx, list = new List<MatlabExpression>());
            }
            else
            {
                ToString();
            }
            list.Add(item);
        }
	}
}
