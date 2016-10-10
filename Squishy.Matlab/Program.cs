using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
	class Program
	{
		static void Main(string[] args)
		{
            using (var nlp = new Dyn1Program("test2run"))
            {
                nlp.WriteProgram();
            }
		}
	}
}
