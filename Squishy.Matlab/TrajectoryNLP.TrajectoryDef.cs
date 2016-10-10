using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
    public partial class TrajectoryNLP
    {
        /// <summary>
        /// A trajectory is made up of several components, each component has multiple quantities, each quantity is a vector or scalar.
        /// </summary>
        public enum TrajectoryQuantityType
        {
            /// <summary>
            /// Usually: Positions & angles
            /// </summary>
            State,

            /// <summary>
            /// Usually: Velocities & angular velocities
            /// </summary>
            Velocity,

            /// <summary>
            /// Contact force
            /// </summary>
            Lambda,

            /// <summary>
            /// Control parameter u
            /// </summary>
            Actuation
        }

        /// <summary>
        /// A trajectory is made up of several components, each component has multiple quantities, each quantity is a vector or scalar.
        /// </summary>
        public class TrajectoryQuantity
        {
            public TrajectoryNLP Program { get; internal set; }
            public TrajectoryQuantityType QType { get; internal set; }

            /// <summary>
            /// Length of the vector representing the values of this quantity (Length = 1 means this is a scalar quantity).
            /// </summary>
            public int Length { get; internal set; }

            public TrajectoryQuantity(TrajectoryNLP program, TrajectoryQuantityType type, double[] min, double[] max)
            {
                Debug.Assert(min.Length == max.Length);
                Program = program;
                QType = type;
                Length = min.Length;
                DefaultMin = min;
                DefaultMax = max;
            }

            public double[] DefaultMin
            {
                get;
                set;
            }

            public double[] DefaultMax
            {
                get;
                set;
            }


            /// <summary>
            /// The index of this quantity (0-based)
            /// </summary>
            public int Index { get; internal set; }

            /// <summary>
            /// The offset of the first scalar of this quantity (1-based)
            /// </summary>
            public int Offset { get; internal set; }

            /// <summary>
            /// Whether this is part of transition.
            /// If it is not part of the transition, it is part of state.
            /// </summary>
            public bool IsTransition { get { return QType == TrajectoryQuantityType.Actuation || QType == TrajectoryQuantityType.Lambda; } }

            /// <summary>
            /// 0-based global index of the k'th instance of this quantity in the trajectory
            /// </summary>
            public int GetGlobalIndex(int k)
            {
                return Program.TDef.GetGlobalIndex(this, k);
            }

            public int GetGlobalIndexWBoundaries(int k)
            {
                return Program.TDef.GetGlobalIndexWBoundaries(this, k);
            }


            /// <summary>
            /// 1-based global offset of the first scalar of the k'th instance of this quantity
            /// </summary>
            public int GetGlobalOffset(int k)
            {
                return Program.TDef.GetGlobalOffset(this, k);
            }

            /// <summary>
            /// 
            /// </summary>
            public MatlabExpression GetGlobalOffsetExprK(int k)
            {
                return GetGlobalOffsetExprOffset(GetGlobalOffset(k));
            }

            public MatlabExpression GetGlobalOffsetExprOffset(int offset)
            {
                return M.Range(offset, offset + Length-1);
            }

            public override string ToString()
            {
                return QType.ToString();
            }

        }

        public class NamedTrajectoryQuantityInstance : TrajectoryQuantityInstance
        {
            public readonly TrajectoryVar Var;

            public NamedTrajectoryQuantityInstance(TrajectoryVar var, int k, TrajectoryQuantity quantity)
                : base(k, quantity)
            {
                Var = var;
            }

            public override string Name
            {
                get { return Var.ToString(); }
            }
        }

        /// <summary>
        /// Represents some instance of some state quantity of some state (or state transition) in the trajectory
        /// </summary>
        public class TrajectoryQuantityInstance : MatlabExpression
        {
            public readonly int K;
            public readonly TrajectoryQuantity Quantity;

            public TrajectoryQuantityInstance(int k, TrajectoryQuantity quantity)
            {
                K = k;
                Quantity = quantity;
                Value = this;
            }

            /// <summary>
            /// The entire trajectory is made up of fixed start and goal state, and the varying "problem space" which is made up of al intermediate states and transitions
            /// </summary>
            public bool IsInProblemSpace
            {
                get
                {
                    return (Quantity.IsTransition || K > 1) && K <= Quantity.Program.Cfg.NSteps;
                }
            }

            /// <summary>
            /// Global index of this quantity instance
            /// </summary>
            public int GlobalIndex { get { return Quantity.GetGlobalIndex(K); } }

            /// <summary>
            /// Global offset of this quantity instance's scalars
            /// </summary>
            public int GlobalOffset { get { return Quantity.GetGlobalOffset(K); } }

            /// <summary>
            /// The index set of this state quantity instance as Matlab expression
            /// </summary>
            private MatlabExpression MIndex { get { return Quantity.GetGlobalOffsetExprK(K); } }

            public virtual string Name
            {
                get { return "Q"; }
            }

            public MatlabExpression Subset(int from)
            {
                return ToString(GlobalOffset + from, Quantity.Length);
            }

            public MatlabExpression Subset(int from, int len)
            {
                return ToString(GlobalOffset + from, len);
            }

            public override int GetHashCode()
            {
                return GlobalOffset.GetHashCode();
            }

            public override bool Equals(object obj)
            {
                return obj is TrajectoryQuantityInstance && ((TrajectoryQuantityInstance)obj).GlobalOffset == GlobalOffset;
            }

            public override string ToString()
            {
                return ToString(GlobalOffset, Quantity.Length);
            }

            public string ToString(int from, int len)
            {
                object name;
                if (from <= 0)
                {
                    // start state
                    name = Squishy.Matlab.TrajectoryNLP.Symbol.StartState;
                    from = Quantity.Offset;
                }
                else if (from <= Quantity.Program.TDef.TotalTrajectorySize)
                {
                    // actual part of the state space
                    name = Name;
                }
                else
                {
                    // final state
                    name = Squishy.Matlab.TrajectoryNLP.Symbol.GoalState;
                    from = Quantity.Offset;
                }
                return string.Format("{0}({1})", M.ConvertName(name), M.Range(from, from + len-1));
            }
        }

        /// <summary>
        /// A trajectory is made of many components, each representing the k'th state & state transition.
        /// </summary>
        public class TrajectoryDef
        {
            public readonly TrajectoryNLP Program;

            public TrajectoryDef(TrajectoryNLP program)
            {
                Program = program;
                Quantities = new List<TrajectoryQuantity>();
            }

            public List<TrajectoryQuantity> Quantities
            {
                get;
                private set;
            }

            /// <summary>
            /// Amount of quantities in a single step
            /// </summary>
            public int SingleStepQuantityCount { get { return Quantities.Count; } }

            /// <summary>
            /// The total size of scalars that make up state (no transitions)
            /// </summary>
            public int StateQuantitySize
            {
                get;
                private set;
            }

            /// <summary>
            /// Amount of quantities that represent state
            /// </summary>
            public int StateQuantityCount
            {
                get;
                private set;
            }

            /// <summary>
            /// Amount of quantities that represent transitions
            /// </summary>
            public int TransitionQuantityCount
            {
                get { return Quantities.Count - StateQuantityCount; }
            }

            /// <summary>
            /// The total amount of quantities in the free trajectory, not including boundaries
            /// </summary>
            public int TotalQuantityCount
            {
                get { return SingleStepQuantityCount * Program.Cfg.NSteps - InitialBoundaryLength; }
            }

            /// <summary>
            /// The total amount of quantities in the entire trajectory, including boundaries
            /// </summary>
            public int TotalQuantityCountWBoundaries
            {
                get { return SingleStepQuantityCount * (Program.Cfg.NSteps + 1); }
            }

            /// <summary>
            /// Total amount of scalars that make up one (non-boundary) step (state & transition)
            /// </summary>
            public int SingleStepSize
            {
                get;
                private set;
            }

            /// <summary>
            /// Total amount of scalars that make up all steps of the trajectory.
            /// </summary>
            public int TotalTrajectorySize
            {
                get;
                private set;
            }

            public int InitialBoundaryLength { get { return StateQuantityCount; } }

            /// <summary>
            /// The index of the given quantity in the problem trajectory (exlcuding boundaries)
            /// </summary>
            public int GetGlobalIndex(TrajectoryQuantity q, int k)
            {
                return GetGlobalIndexWBoundaries(q, k) - InitialBoundaryLength;
            }

            /// <summary>
            /// The index of the given quantity in the problem trajectory (including boundaries)
            /// </summary>
            public int GetGlobalIndexWBoundaries(TrajectoryQuantity q, int k)
            {
                return SingleStepQuantityCount * (k - 1) + q.Index;
            }

            /// <summary>
            /// The index of the first scalar of the given quantity in the problem trajectory (exlcuding boundaries)
            /// </summary>
            public int GetGlobalOffset(TrajectoryQuantity q, int k)
            {
                return SingleStepSize * (k - 1) + q.Offset - StateQuantitySize;
            }

            public TrajectoryQuantity AddQuantity(TrajectoryQuantityType type, double min, double max)
            {
                return AddQuantity(type, new[] { min }, new[] { max });
            }

            public TrajectoryQuantity AddQuantity(TrajectoryQuantityType type, double[] min, double[] max)
            {
                var q = new TrajectoryQuantity(Program, type, min, max);
                AddQuantity(q);
                return q;
            }

            /// <summary>
            /// Add a quantity
            /// </summary>
            public void AddQuantity(TrajectoryQuantity q)
            {
                var lastIdx = Quantities.Count;
                var nOccurancesDelta = q.IsTransition ? 0 : -1;
                TotalTrajectorySize += (Program.Cfg.NSteps + nOccurancesDelta) * q.Length;
                SingleStepSize += q.Length;

                q.Index = lastIdx;
                if (lastIdx > 0)
                    q.Offset = Quantities.Last().Offset + Quantities.Last().Length;
                else
                    q.Offset = 1;

                if (lastIdx > 0 && Quantities.Last().IsTransition)
                {
                    // Transitions must be added to the end
                    Debug.Assert(q.IsTransition);
                }
                else if (q.IsTransition)
                {
                    // The first transition
                    StateQuantityCount = lastIdx;
                    StateQuantitySize = q.Offset - 1;
                }
                Quantities.Add(q);
            }

            /// <summary>
            /// Create a named Matlab trajectory symbol
            /// </summary>
            public TrajectoryVar Var(object name)
            {
                return new TrajectoryVar(Program, name);
            }
        }

        /// <summary>
        /// Represents a trajectory in Matlab code. Includes boundaries.
        /// </summary>
        public class TrajectoryVar : MatlabExpression
        {
            public readonly TrajectoryNLP Program;

            public NamedTrajectoryQuantityInstance[] Instances
            {
                get;
                private set;
            }

            public TrajectoryVar(TrajectoryNLP program, object name)
                : base(name)
            {
                Program = program;

                var totalCount = program.TDef.TotalQuantityCountWBoundaries;
                Instances = new NamedTrajectoryQuantityInstance[totalCount];
                var idx = 0;
                for (var k = 1; k <= program.Cfg.NSteps + 1; ++k)
                {
                    for (var i = 0; i < program.TDef.SingleStepQuantityCount; ++i)
                    {
						var ins = Instances[idx] = new NamedTrajectoryQuantityInstance(this, k, program.TDef.Quantities[i]);
						if (ins.IsInProblemSpace)
							Console.WriteLine("{0}: {1}({2})", ins, ins.Quantity.QType, ins.K);
                        ++idx;
                    }
                }
            }

            /// <summary>
            /// Start state & transition
            /// </summary>
            public NamedTrajectoryQuantityInstance Start(TrajectoryQuantity q)
            {
                return this[q, 1];
            }

            /// <summary>
            /// Goal state & transition
            /// </summary>
            public NamedTrajectoryQuantityInstance Goal(TrajectoryQuantity q)
            {
                return this[q, Program.Cfg.NSteps + 1];
            }

            /// <summary>
            /// Returns "Q(idx)", where Q is this var's name and idx is the index set of the given StateQuantity in the k'th component.
            /// </summary>
            public NamedTrajectoryQuantityInstance this[TrajectoryQuantity sq, int k]
            {
                get
                {
                    return Instances[sq.GetGlobalIndexWBoundaries(k)];
                }
            }

            public NamedTrajectoryQuantityInstance GetInstance(int globalIdx)
            {
                return Instances[globalIdx + Program.TDef.InitialBoundaryLength];
            }
        }

        /// <summary>
        /// Represents values of a trajectory in Matlab code. Does not include boundaries.
        /// </summary>
        public class TrajectoryInstance : MatlabColVector
        {
            public TrajectoryNLP Program { get; private set; }

            public TrajectoryInstance(TrajectoryNLP program, int nDims = 1)
                : base(new MatlabExpression[program.TDef.TotalQuantityCount])
            {
                Program = program;

                // zero out everything initially
                Reset(nDims);
            }

            public MatlabExpression[] Values
            {
                get { return (MatlabExpression[])Value; }
            }

            public void Reset(int nDims = 1)
            {
                for (var k = 1; k <= Program.Cfg.NSteps + 1; ++k)
                {
                    for (var i = 0; i < Program.TDef.SingleStepQuantityCount; ++i)
                    {
                        SetValue(Program.Q[Program.TDef.Quantities[i], k], M.Zeros(Program.TDef.Quantities[i].Length, nDims));
                    }
                }
            }

            public void SetValue(TrajectoryQuantityInstance q, params object[] exprs)
            {
                Debug.Assert(q.Quantity.Length == exprs.Count());
                var idx = q.GlobalIndex;
                if (idx >= 0 && idx < Values.Length)
                {
                    Values[idx] = M.ColVector(exprs);
                }
                // Silently ignore invalid values
            }

            public void SetValue(TrajectoryQuantityInstance q, MatlabExpression value)
            {
                var idx = q.GlobalIndex;
                if (idx >= 0 && idx < Values.Length)
                {
                    Values[idx] = value;
                }
                // Silently ignore invalid values
            }
        }

        /// <summary>
        /// Represents an instance of a state (without transition). Used for start and goal state definition.
        /// </summary>
        public class TrajectoryStateInstance : MatlabColVector
        {
            public TrajectoryNLP Program { get; private set; }

            public TrajectoryStateInstance(TrajectoryNLP program)
                : base(new MatlabExpression[program.TDef.StateQuantityCount])
            {
                Program = program;

                // zero out everything initially
                for (var i = 0; i < program.TDef.StateQuantityCount; ++i)
                {
                    Values[i] = M.Zeros(Program.TDef.Quantities[i].Length, 1);
                }
            }

            public MatlabExpression[] Values
            {
                get { return (MatlabExpression[])Value; }
            }

            public void SetValue<T>(TrajectoryQuantity q, params T[] exprs)
            {
                Debug.Assert(q.Length == exprs.Count());

                var idx = q.Index;
                Debug.Assert(idx >= 0 && idx < Values.Length);
                Values[idx] = M.ColVector(exprs);
            }

            public void SetValue(TrajectoryQuantityInstance q, MatlabExpression value)
            {
                var idx = q.GlobalIndex;
                Debug.Assert(idx >= 0 && idx < Values.Length);
                Values[idx] = value;
            }
        }
    }
}
