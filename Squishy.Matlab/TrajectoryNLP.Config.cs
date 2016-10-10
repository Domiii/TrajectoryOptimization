using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
    public partial class TrajectoryNLP
    {
        public class KeyValuePair : IEnumerable<object>
        {
            public object Key, Value;

            public KeyValuePair(object k, object v)
            {
                Key = k;
                Value = v;
            }

            public IEnumerator<object> GetEnumerator()
            {
                yield return Key;
                yield return Value;
            }

            IEnumerator IEnumerable.GetEnumerator()
            {
                yield return Key;
                yield return Value;
            }
        }

        public class Config
        {
            public Config(string name, double endTime = 10)
            {
                Name = name;
                T = endTime;
                OptimizerOptions = new List<KeyValuePair>();
                OutputNames = new List<object> {
                    "Qf", "fval", "exitFlag", "output"
                } ;
            }

            /// <summary>
            /// The name of the program (file & execution function)
            /// </summary>
            public string Name { get; private set; }

            /// <summary>
            /// Max amount of time to get to the goal (seconds)
            /// </summary>
            public double T { get; set; }

            /// <summary>
            /// Max amount of steps per trajectory
            /// </summary>
            public int NSteps { get; set; }

            /// <summary>
            /// Step length
            /// </summary>
            public double H { get { return T / NSteps; } }

            public List<KeyValuePair> OptimizerOptions { get; set; }

            /// <summary>
            /// All output arguments of the solver call to be returned by the solver function
            /// </summary>
            public List<object> OutputNames { get; set; }

            public void AddOptimizerOption(object key, object value)
            {
                OptimizerOptions.Add(new KeyValuePair(key, value));
            }
        }
    }
}
