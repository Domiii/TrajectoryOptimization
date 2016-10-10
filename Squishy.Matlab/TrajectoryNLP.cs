using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using SparseTrajTensor = Squishy.Matlab.SparseMatlabTensor<Squishy.Matlab.TrajectoryNLP.QInstanceIndex>;

namespace Squishy.Matlab
{
	/// <summary>
	/// A trjacetory optimization problem in form of a Matlab NLP.
	/// </summary>
	public abstract partial class TrajectoryNLP : MatlabWriter
    {
		/// <summary>
		/// Configuration of this Trajectory Optimization problem
		/// </summary>
        public readonly Config Cfg;

		public TrajectoryVar Q {
			get; private set;
		}

        public readonly TrajectoryDef TDef;

		/// <summary>
		/// Components of the cost function J
		/// </summary>
		List<MatlabExpression> JCode;

		/// <summary>
		/// Components of the cost function gradient nabla J
		/// </summary>
		List<MatlabExpression>[] JGradCode;

		/// <summary>
		/// Components of non-linear equality and inequality constraints.
		/// </summary>
		List<MatlabExpression> IConstraintsNLCode, EConstraintsNLCode;

		/// <summary>
		/// Components of non-linear equality and inequality constraints.
		/// </summary>
		SparseTrajTensor IConstraintsNLGradCode, EConstraintsNLGradCode;

		#region Init
        public TrajectoryNLP(Config cfg) : base(cfg.Name + ".m")
		{
            Cfg = cfg;
            TDef = new TrajectoryDef(this);
		}
		
        /// <summary>
        /// Define and write the entire program to file
        /// </summary>
        public void WriteProgram()
        {
            // define the problem structure
            InitSettings();
            AddQuantities();                                    // user-defined Trajectory structure
			InitProblem();                                      // pre-processing of Trajectory data

            // define the current trajectory in all parts of the code
            Q = TDef.Var(Symbol.Q);

            // write cost & constraint functions
            DefineCostToGo();                                   // all costs
            DefineConstraints();                                // all constraints

            // write the Matlab program
            DoWriteProgram();
		}

        /// <summary>
        /// Called to define the problem config
        /// </summary>
        protected virtual void InitSettings()
        { }

		/// <summary>
		/// Create the problem structure
		/// </summary>
		protected virtual void AddQuantities()
		{
		}

		/// <summary>
		/// Called after trajectory has been defined
		/// </summary>
		protected virtual void InitProblem()
		{
			JCode = new List<MatlabExpression>();
			JGradCode = new List<MatlabExpression>[TDef.TotalQuantityCount];
			IConstraintsNLCode = new List<MatlabExpression>();
			IConstraintsNLGradCode = new SparseTrajTensor();
			EConstraintsNLCode = new List<MatlabExpression>();
			EConstraintsNLGradCode = new SparseTrajTensor();
		}
		#endregion


        #region Start & Goal State, Initial Trajectory, Trajectory Bounds and Linear Constraints

        /// <summary>
        /// Creates the initial state
        /// </summary>
        protected abstract void CreateStartState(TrajectoryStateInstance start);

        /// <summary>
        /// Creates the initial state
        /// </summary>
        protected abstract void CreateGoalState(TrajectoryStateInstance goal);

        /// <summary>
        /// Creates the initial state
        /// </summary>
        protected abstract void CreateQ0(TrajectoryInstance Q0);

		/// <summary>
		/// Creates the lower bounds vector
		/// </summary>
		protected virtual void CreateLBounds(TrajectoryInstance bounds)
		{
			for (int k = 1; k <= Cfg.NSteps; ++k)
			{
				foreach (var q in TDef.Quantities)
				{
                    var min = q.DefaultMin.Select(val => double.IsNaN(val) ? M.ConvertName(M.Keyword.NegInf) : val.ToString()).ToArray();
					bounds.SetValue(Q[q, k], min);
				}
			}
		}

		/// <summary>
		/// Creates the upper bounds vector
		/// </summary>
		protected virtual void CreateUBounds(TrajectoryInstance bounds)
		{
			for (int k = 1; k <= Cfg.NSteps; ++k)
			{
				foreach (var q in TDef.Quantities)
                {
                    var max = q.DefaultMax.Select(val => double.IsNaN(val) ? M.Keyword.Inf.ToString() : val.ToString()).ToArray();
                    bounds.SetValue(Q[q, k], max);
				}
			}
		}

		/// <summary>
		/// Creates linear equality constraint matrix and RHS vector
		/// </summary>
		protected abstract void CreateLinEqualityConstraints(SparseTrajTensor Ae, List<MatlabExpression> be);

		/// <summary>
		/// Creates linear inequality constraint matrix and RHS vector
		/// </summary>
		protected abstract void CreateLinInequalityConstraints(SparseTrajTensor Ai, List<MatlabExpression> bi);

		#endregion


        #region Cost & Constraint Definitions
        /// <summary>
        /// Define cost (J) and it's gradient (JGradCode)
        /// </summary>
        protected virtual void DefineCostToGo()
        {
        }

        /// <summary>
        /// Define nonlinear constraint (E/IConstraintsNLCode) and their gradients (E/IConstraintsNLGradCode)
        /// </summary>
        protected virtual void DefineConstraints()
        {
        }


		protected void AddCost(MatlabExpression expr)
		{
			JCode.Add(expr);
		}

		protected void AddCostGrad(TrajectoryQuantityInstance q, MatlabExpression expr)
		{
			var idx = q.GlobalIndex;
			if (idx >= 0 && idx < JGradCode.Length)
			{
				var list = JGradCode[idx];
				if (list == null)
				{
					JGradCode[idx] = list = new List<MatlabExpression>(5);
				}
				list.Add(expr);
			}
			// for now: Silently ignore invalid gradients (such as those from the boundaries)
		}

        /// <summary>
        /// Add a dynamics equality constraint of the form:
        ///  qNew - qOld - h qDot = 0
        ///  qDotNew - qDotOld - h qDDot = 0
        /// Also adds all standard gradients (all but the gradients of qDDot).
        /// </summary>
        /// <returns>The force constraint, so the gradients of qDDot can be added.</returns>
        protected Constraint AddDynamicsEConstraintAndGradients(int k, TrajectoryQuantity q, TrajectoryQuantity qDot, MatlabExpression qDDot)
        {
            Debug.Assert(q.Length == qDot.Length);

            // qNew - qOld - h qDot = 0
            var qDotNew = Q[qDot, k + 1]; var qDotOld = Q[qDot, k];
            var c1 = AddVelocityIntegrationEConstraintAndGradient(k, q, qDot);

            // qDotNew - qDotOld - h qDDot = 0
            var c2 = AddDynamicsEConstraint(qDotNew, qDotOld, qDDot, q.Length);
            AddEConstraintGrad(c2, qDotNew, M.Eye(qDot.Length));
            AddEConstraintGrad(c2, qDotOld, "-" + M.Eye(qDot.Length));
            return c2;
        }

        /// <summary>
        /// Add a dynamics equality constraint of the form qNew - qOld - h qDot = 0
        /// </summary>
        protected Constraint AddDynamicsEConstraint(object qNew, object qOld, MatlabExpression qDot, int nDims = 1)
        {
            return AddEConstraint(string.Format("{0} - {1} - {2} * ({3})", qNew, qOld, Cfg.H, qDot), nDims);
        }

        /// <summary>
        /// Add a dynamics equality constraint of the form q(k+1) - q(k) - h qDot(k+1) = 0
        /// </summary>
        protected Constraint AddVelocityIntegrationEConstraintAndGradient(int k, TrajectoryQuantity q, TrajectoryQuantity qDot)
        {
            Debug.Assert(q.Length == qDot.Length);

            // qNew - qOld - h qDot = 0
            var qNew = Q[q, k + 1]; var qOld = Q[q, k];
            var qDotK = Q[qDot, k];
            var ci = AddDynamicsEConstraint(qNew, qOld, qDotK, q.Length);

            AddEConstraintGrad(ci, qNew, M.Eye(q.Length));
            AddEConstraintGrad(ci, qOld, "-" + M.Eye(q.Length));
            AddEConstraintGrad(ci, qDotK, "- " + Cfg.H + "*" + M.Eye(q.Length));

            return ci;
        }

        protected Constraint AddEConstraint(MatlabExpression expr, int nDims = 1)
        {
            return AddConstraint(EConstraintsNLCode, expr, nDims);
        }

        protected Constraint AddIConstraint(MatlabExpression expr, int nDims = 1)
        {
            return AddConstraint(IConstraintsNLCode, expr, nDims);
        }

        /// <summary>
        /// Add a multi-dimensional equality constraint gradient
        /// </summary>
        protected void AddEConstraintGrad(Constraint ci, TrajectoryQuantityInstance q, MatlabExpression expr)
        {
            if (expr.ToString() == "- 1*1")
                Console.WriteLine("{0}, {1}: {2}", ci.Index, q, expr);
            AddConstraintGrad(EConstraintsNLGradCode, q, ci, expr);
            //if (ci.NDims > 1)
        }

        /// <summary>
        /// Add an equality constraint gradient
        /// </summary>
        protected void AddIConstraintGrad(Constraint ci, TrajectoryQuantityInstance q, MatlabExpression expr)
        {
            AddConstraintGrad(IConstraintsNLGradCode, q, ci, expr);
        }
		
        /// <summary>
        /// Add a constraint
        /// </summary>
		private Constraint AddConstraint(List<MatlabExpression> constraints, MatlabExpression expr, int nDims)
		{
			constraints.Add(expr);
			return new Constraint{ Index = constraints.Count-1, NDims = nDims};
		}

        /// <summary>
        /// Add a constraint gradient
        /// </summary>
        private void AddConstraintGrad(SparseMatlabTensor<QInstanceIndex> constraintGrad, TrajectoryQuantityInstance q, Constraint ci, MatlabExpression expr)
        {
            constraintGrad.Add(new QInstanceIndex(ci, q), expr);
        }
        #endregion

        #region Writing Matlab Code
        /// <summary>
		/// 
		/// </summary>
		protected virtual void DoWriteProgram()
        {
            Write(M.FunctionDef(Cfg.OutputNames.ToArray(), Cfg.Name));

            WriteRunFunctionBody();
			WriteCostToGoFunction();
            WriteNLConstraintsFunction();
            
            Write(M.End());
		}

        
        /// <summary>
        /// The function that returns the cost (and its gradient)
        /// </summary>
		protected virtual void WriteCostToGoFunction()
		{
            // create new function
            Write(M.FunctionDef(new[] { Symbol.J, Symbol.JGrad }, Symbol.CostToGoFunction, Symbol.Q));
            
            // Assign J
            Write(M.Assign(Symbol.J, M.Sum(M.ColVector(JCode))));

            // Assign gradient, if required
            //Write(M.If("nargout > 1", M.Assign(Symbol.JGrad, M.CallFunction(Symbol.JGrad, Symbol.CostToGoFunctionGrad, Symbol.Q))));
            var code = JGradCode.Select((gradi, i) => gradi == null ? 
					M.Zeros(Q.GetInstance(i).Quantity.Length, 1) :
					(	gradi.Count > 1 ?
						M.Transpose(M.Sum(M.ColVector(gradi.Select(expr => M.Transpose(expr)).ToArray()))) : 
						gradi[0]
					)).ToArray();
            Write(M.If("nargout > 1", M.Assign(Symbol.JGrad, M.ColVector(code))));
            
            // end function
            Write(M.End());
		}

        protected virtual void WriteNLConstraintsFunction()
        {
            // create new function
            Write(M.FunctionDef(new[] { Symbol.Ci, Symbol.Ce, Symbol.CiGrad, Symbol.CeGrad }, Symbol.NonLinConstraintFun, Symbol.Q));

            // Assign ci
            Write(M.Assign(Symbol.Ci, M.ColVector(IConstraintsNLCode)));

            // Assign ce
            Write(M.Assign(Symbol.Ce, M.ColVector(EConstraintsNLCode)));

            // Assign gradient, if required
            Write(M.If("nargout > 2",
                M.Assign(Symbol.CiGrad, ToMatrix(IConstraintsNLGradCode)), 
                M.Assign(Symbol.CeGrad, ToMatrix(EConstraintsNLGradCode))));

            // end function
            Write(M.End());
        }

        /// <summary>
        /// The code that calls the solver
        /// </summary>
        protected virtual void WriteRunFunctionBody()
        {
            // create & assign initial state, trajectory, bounds and linear constraints


            var start = new TrajectoryStateInstance(this);
            CreateStartState(start);

            var goal = new TrajectoryStateInstance(this);
            CreateGoalState(goal);

            Write(M.Assign(Symbol.StartState, start));
            Write(M.Assign(Symbol.GoalState, goal));

            var Q0 = new TrajectoryInstance(this);
            CreateQ0(Q0);

			var lbounds = new TrajectoryInstance(this);
            CreateLBounds(lbounds);

			var ubounds = new TrajectoryInstance(this);
			CreateUBounds(ubounds);

			var Ae = new SparseTrajTensor();
			var be = new List<MatlabExpression>();
			CreateLinEqualityConstraints(Ae, be);

			var Ai = new SparseTrajTensor();
			var bi = new List<MatlabExpression>();
			CreateLinEqualityConstraints(Ai, bi);
            Write(M.Assign(Symbol.Q0, Q0));
            Write(M.Assign(Symbol.LBounds, lbounds));
			Write(M.Assign(Symbol.UBounds, ubounds));
            Write(M.Assign(Symbol.AE, ToMatrix(Ae)));
            Write(M.Assign(Symbol.BE, M.ColVector(be)));
            Write(M.Assign(Symbol.AI, ToMatrix(Ai)));
            Write(M.Assign(Symbol.BI, M.ColVector(bi)));

            // set optimizer options
            var optionArgs = Cfg.OptimizerOptions.SelectMany((options, idx) => options).ToArray();
            Write(M.Assign(Symbol.Options, M.CallFunction(Symbol.OptimSet, optionArgs)));

            // call the solver
            Write(M.Assign("[Qf, fval, exitFlag, output]", M.CallFunction(Symbol.FminCon,
                M.RefFunction(Symbol.CostToGoFunction), Symbol.Q0, Symbol.AI, Symbol.BI, Symbol.AE, Symbol.BE,
                Symbol.LBounds, Symbol.UBounds, M.RefFunction(Symbol.NonLinConstraintFun), Symbol.Options)));

			Write("output");
        }
        #endregion

        #region Utilities
        #endregion
    }
}
