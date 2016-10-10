using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
    public partial class TrajectoryNLP
    {
        #region Names of symbols used in the Matlab code
        /// <summary>
        /// Hardcoded names of symbols used in Trajectory NLPs
        /// </summary>
        public enum Symbol
        {
            /// <summary>
            /// Initial trajectory
            /// </summary>
			[Squishy.Matlab.M.Name("Q0")]
            Q0,

            /// <summary>
            /// General state (trajectory) variable name
            /// </summary>
			[Squishy.Matlab.M.Name("Q")]
			Q,

			/// <summary>
			/// Start state
			/// </summary>
			[Squishy.Matlab.M.Name("qs")]
			StartState,

			/// <summary>
			/// Goal state
			/// </summary>
			[Squishy.Matlab.M.Name("qg")]
			GoalState,

            /// <summary>
            /// Lower bounds of the NLP state
            /// </summary>
			[Squishy.Matlab.M.Name("lBounds")]
            LBounds,

            /// <summary>
            /// Upper bounds of the NLP state
            /// </summary>
            [Squishy.Matlab.M.Name("uBounds")]
            UBounds,

            /// <summary>
            /// Linear equality constraint matrix
            /// </summary>
            [Squishy.Matlab.M.Name("AE")]
            AE,

            /// <summary>
            /// Linear equality constraint RHS vector 
            /// </summary>
            [Squishy.Matlab.M.Name("bE")]
            BE,

            /// <summary>
            /// Linear inequality constraint matrix
            /// </summary>
            [Squishy.Matlab.M.Name("AI")]
            AI,

            /// <summary>
            /// Linear inequality constraint RHS vector 
            /// </summary>
            [Squishy.Matlab.M.Name("bI")]
            BI,

            /// <summary>
            /// Name of the matlab cost-to-go function
            /// </summary>
            [Squishy.Matlab.M.Name("objFun")]
            CostToGoFunction,

            /// <summary>
            /// Name of the matlab cost-to-go gradient function
            /// </summary>
            [Squishy.Matlab.M.Name("objFun")]
            CostToGoFunctionGrad,

            /// <summary>
            /// Name of the matlab function that evaluates non-linear constraints
            /// </summary>
            [Squishy.Matlab.M.Name("nonLinConstraintFun")]
            NonLinConstraintFun,

            /// <summary>
            /// Name of the option set function
            /// </summary>
            [Squishy.Matlab.M.Name("optimset")]
            OptimSet,

            /// <summary>
            /// Name of the NLP solver
            /// </summary>
            [Squishy.Matlab.M.Name("fmincon")]
            FminCon,

            /// <summary>
            /// Name of the options variable
            /// </summary>
            [Squishy.Matlab.M.Name("options")]
            Options,

            /// <summary>
            /// Name of the cost-to-go variable
            /// </summary>
            [Squishy.Matlab.M.Name("J")]
            J,

            /// <summary>
            /// Name of the cost-to-go gradient variable
            /// </summary>
            [Squishy.Matlab.M.Name("JGrad")]
            JGrad,

            /// <summary>
            /// Nonlinear inequality constraints
            /// </summary>
            [Squishy.Matlab.M.Name("ci")]
            Ci,

            /// <summary>
            /// Nonlinear equality constraints
            /// </summary>
            [Squishy.Matlab.M.Name("ce")]
            Ce,

            /// <summary>
            /// Nonlinear inequality constraint gradients
            /// </summary>
            [Squishy.Matlab.M.Name("ciGrad")]
            CiGrad,

            /// <summary>
            /// Nonlinear equality constraint gradients
            /// </summary>
            [Squishy.Matlab.M.Name("ceGrad")]
            CeGrad,
        }
        #endregion
    }
}
