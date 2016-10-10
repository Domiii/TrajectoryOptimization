using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SparseTrajTensor = Squishy.Matlab.SparseMatlabTensor<Squishy.Matlab.TrajectoryNLP.QInstanceIndex>;

namespace Squishy.Matlab
{
    /// <summary>
    /// Trajectory of a single SLIP (Spring-Loaded Inverted Pendulum)
    /// </summary>
    public class Dyn1Program : TrajectoryNLP
    {
        public readonly Test2Settings Settings;

        public Dyn1Program(string name)
            : this(CreateConfig(name))
        {
        }

        public Dyn1Program(Config cfg)
            : base(cfg)
        {
            Settings = new Test2Settings();
        }

        #region All Trajectory Quantities
        /// <summary>
        /// Center of mass
        /// </summary>
        public static TrajectoryQuantity CM, CMDot;

        /// <summary>
        /// Angle of the spring with the floor
        /// </summary>
        public static TrajectoryQuantity theta, thetaDot;

        /// <summary>
        /// Spring length
        /// </summary>
        public static TrajectoryQuantity d, dDot;

        /// <summary>
        /// Contact forces
        /// </summary>
        public static TrajectoryQuantity lambda1, lambda2;

        /// <summary>
        /// Controllable actuations
        /// </summary>
        public static TrajectoryQuantity uTheta, uD;

        /// <summary>
        /// Use above quantities to define trajectory
        /// </summary>
        protected override void AddQuantities()
        {
            var inf = double.NaN;
            var inf2 = new[] { inf, inf };

            CM = TDef.AddQuantity(TrajectoryQuantityType.State, Settings.BBoxMin, Settings.BBoxMax);
            CMDot = TDef.AddQuantity(TrajectoryQuantityType.Velocity, inf2, inf2);
            //theta = TDef.AddQuantity(TrajectoryQuantityType.State, inf, inf);
            //thetaDot = TDef.AddQuantity(TrajectoryQuantityType.Velocity, inf, inf);
            d = TDef.AddQuantity(TrajectoryQuantityType.State, -Settings.DMax, Settings.DMax);
            dDot = TDef.AddQuantity(TrajectoryQuantityType.Velocity, inf, inf);
            lambda1 = TDef.AddQuantity(TrajectoryQuantityType.Lambda, 0, inf);
            //lambda2 = TDef.AddQuantity(TrajectoryQuantityType.Lambda, 0, inf);
            //uTheta = TDef.AddQuantity(TrajectoryQuantityType.Actuation, -Settings.UThetaMax, Settings.UThetaMax);
            uD = TDef.AddQuantity(TrajectoryQuantityType.Actuation, -Settings.UDMax, Settings.UDMax);
        }
        #endregion


        #region Configuration & Settings
        static Config CreateConfig(string name)
        {
            var cfg = new Config(name)
            {
                NSteps = 15,
                T = 4
            };
			cfg.OutputNames = new List<object> {
                    //"Qf", "fval", "exitFlag", "output"
					"cms", "ds", "us", "lambdas", "worldMin", "worldMax", "groundHeight", "restLen", Symbol.StartState, Symbol.GoalState
                };
            
            cfg.AddOptimizerOption("'Algorithm'", "'sqp'");
            cfg.AddOptimizerOption("'GradObj'" , "'on'");
            cfg.AddOptimizerOption("'GradConstr'", "'on'");
            //'UseParallel','never'

            return cfg;
        }

        protected override void InitSettings()
        {
            var lenRest = 2.0;
            var groundHeight = 0;
            var goalD = .4;

            // geometry
            Settings.BBoxMin = new[] { -1.0, -2 };
            Settings.BBoxMax = new[] { 1.0, 60 };
            Settings.GroundHeight = groundHeight;

            // start & goal: Spring in stance, at rest
            Settings.StartCM = new[] { 0.0, groundHeight + lenRest / 2 };
            Settings.GoalCM = new[] { 0.0, groundHeight + (lenRest + goalD) / 2 + 10 };
            Settings.GoalD = goalD;

            // dynamics
            Settings.Gravity = -1;
            Settings.DMax = 1.5;
			Settings.KSpring = 10;
            Settings.LenRest = lenRest;
            Settings.M1 = 1;
            Settings.M2 = 1;
            Settings.UDMax = 20;
            Settings.UThetaMax = 2;
        }
        #endregion


        #region Start & Goal States, Initial Trajectory and Linear Constraints

        /// <summary>
        /// Create the initial state
        /// </summary>
        protected override void CreateStartState(TrajectoryStateInstance start)
        {
            start.SetValue(CM, Settings.StartCM);
        }

        /// <summary>
        /// Create the goal state
        /// </summary>
        protected override void CreateGoalState(TrajectoryStateInstance goal)
        {
            goal.SetValue(CM, Settings.GoalCM);
            goal.SetValue(d, Settings.GoalD);
        }

        /// <summary>
        /// Create the initial trajectory
        /// </summary>
        protected override void CreateQ0(TrajectoryInstance Q0)
        {
			Write(M.Assign("dist", M.Div(M.Subtract(Q.Goal(CM), Q.Start(CM)), Cfg.NSteps)));
            for (int k = 1; k <= Cfg.NSteps; ++k)
            {
                //Q0.SetValue(Q[CM, k], Q.Start(CM) + " + " + M.Mult(k, "dist"));
            }
        }

        /// <summary>
        /// Creates linear equality constraint matrix and RHS vector
        /// </summary>
        protected override void CreateLinEqualityConstraints(SparseTrajTensor Ae, List<MatlabExpression> be)
        {
        }

        /// <summary>
        /// Creates linear inequality constraint matrix and RHS vector
        /// </summary>
        protected override void CreateLinInequalityConstraints(SparseTrajTensor Ai, List<MatlabExpression> bi)
        {
        }

        #endregion

        /// <summary>
        /// Define cost (J) and it's gradient (JGradCode)
        /// </summary>
        protected override void DefineCostToGo()
        {
            // Shortest path objective
            //for (var k = 1; k <= Cfg.NSteps; ++k)
            for (var k = 1; k < Cfg.NSteps; ++k)
			{
				var cmNow = Q[CM, k];
				var cmNext = Q[CM, k + 1];
				var cmDotk = Q[CMDot, k];
				//var cmDotNext = Q[CMDot, k + 1];
				var ud = Q[uD, k];
				//AddCost(M.Norm(cmNow, cmNext));
				//AddCostGrad(cmNow, M.NormGrad(cmNow, cmNext));
				//AddCostGrad(cmNext, M.NormGrad(cmNext, cmNow));

				//AddCost("-" + cmDotk + "'*" + cmDotk);
				//AddCostGrad(cmDotk, "-2 * " + cmDotk);

                AddCost(cmNow.Subset(1, 1));
                AddCostGrad(cmNow, M.ColVector(0, 1));

				//AddCost("-" + cmNow.Subset(1, 1));
				//AddCostGrad(cmNow, "[0;-1]");

				//AddCost(M.Mult(ud, ud));
				//AddCostGrad(ud, "2 * " + ud);
            }
        }

        protected override void DefineConstraints()
        {
            for (var k = 1; k <= Cfg.NSteps; ++k)
            {
                var ud = Q[uD, k];
                var dk = Q[d, k]; var dNew = Q[d, k + 1];
                var dDotk = Q[dDot, k]; var dDotNew = Q[dDot, k + 1];

                var cmk = Q[CM, k]; var cmNew = Q[CM, k + 1];
				var cmDotk = Q[CMDot, k]; var cmDotNew = Q[CMDot, k + 1];
				var l1k = Q[lambda1, k]; //var l2 = Q[lambda2, k];
				var l1New = Q[lambda1, k+1]; //var l2 = Q[lambda2, k];


                // Dynamics of the system are given in form of constraints, representing forward-Euler integrators
                //if (k < Cfg.NSteps)

				// Center-of-Mass dynamics
				var cCM = AddDynamicsEConstraintAndGradients(k, CM, CMDot, 
					string.Format("{0} + 1 * [0; 1] * {1}", Settings.GravityVector, l1k));

				AddEConstraintGrad(cCM, l1k, Cfg.H + " * (1 * [0; 1]')");

                // spring dynamics
                var cd = AddDynamicsEConstraintAndGradients(k, d, dDot,
                    string.Format("{0} * ({1} - {2} * {3}) + .5 * {4}", 1 / Settings.Mass, ud, 2 * Settings.KSpring, dNew, l1k));

                AddEConstraintGrad(cd, ud, Cfg.H * 1 / Settings.Mass);
                AddEConstraintGrad(cd, dNew, Cfg.H * -2 * Settings.KSpring * 1 / Settings.Mass);
                AddEConstraintGrad(cd, l1k, Cfg.H * .5);

                if (k > 1)
                {
                    // non-penetration constraint: Phi <= 0
                    var phi = AddIConstraint(string.Format("-({0} - .5 * ({1}+{2}) - {3})", cmk.Subset(1,1), dk, Settings.LenRest, Settings.GroundHeight));
                    AddIConstraintGrad(phi, cmk, M.ColVector(0.0, -1.0));
                    AddIConstraintGrad(phi, dk, .5);


                    // complementariy constraint: Lambda * Phi = 0
                    var lambdaPhi = AddEConstraint(string.Format("({0} - .5 * ({1}+{2}) - {3})*{4}", cmk.Subset(1, 1), dk, Settings.LenRest, Settings.GroundHeight, l1k));
                    AddEConstraintGrad(lambdaPhi, cmk, M.ColVector("0.0", l1k.ToString()));
                    AddEConstraintGrad(lambdaPhi, dk, "-.5 * " + l1k);
                    AddEConstraintGrad(lambdaPhi, l1k, string.Format("({0} - .5 * ({1}+{2}) - {3})", cmk.Subset(1, 1), dk, Settings.LenRest, Settings.GroundHeight));
                }
            }
        }

		#region PostProcessing
		protected override void WriteRunFunctionBody()
		{
			base.WriteRunFunctionBody();

			var qName = Q.Value;
			Q.Value = "Qf";
			//Write("Qf");
			Write(M.Assign("worldMin", M.ColVector(Settings.BBoxMin)));
			Write(M.Assign("worldMax", M.ColVector(Settings.BBoxMax)));
			Write(M.Assign("groundHeight", Settings.GroundHeight));
			Write(M.Assign("restLen", Settings.LenRest));
			Write(M.Assign("cms", M.CellArray(Q.Instances.Where(q => q.Quantity == CM).Select(q => q.ToString()))));
			Write(M.Assign("ds", M.CellArray(Q.Instances.Where(q => q.Quantity == d).Select(q => q.ToString()))));
			Write(M.Assign("us", M.CellArray(Q.Instances.Where(q => q.Quantity == uD && q.K <= Cfg.NSteps).Select(q => q.ToString()))));
			Write(M.Assign("lambdas", M.CellArray(Q.Instances.Where(q => q.Quantity == lambda1 && q.K <= Cfg.NSteps).Select(q => q.ToString()))));
			Q.Value = qName;
		}
		#endregion

		#region Settings Class
		public class Test2Settings
        {
            /// <summary>
            /// The bounding box defines the scene size
            /// </summary>
            public double[] BBoxMin, BBoxMax;

            /// <summary>
            /// The height of the flat ground
            /// </summary>
            public double GroundHeight;


            // Start & goal conditions
            public double[] StartCM;

            public double[] GoalCM;

            public double GoalD;


            /// <summary>
            /// Acceleration due to gravity
            /// </summary>
            public double Gravity;

            /// <summary>
            /// Mass at top and bottom of our spring
            /// </summary>
            public double M1, M2;

            /// <summary>
            /// Total mass of the SLIP
            /// </summary>
            public double Mass { get { return M1 + M2; } }

            /// <summary>
            /// Spring stiffness
            /// </summary>
            public double KSpring;

            /// <summary>
            /// Rest length of the spring
            /// </summary>
            public double LenRest;

            public double RestHeight { get { return GroundHeight + LenRest; } }

            /// <summary>
            /// Max extension/contraction of the spring (should be less than LenRest)
            /// </summary>
            public double DMax;

            /// <summary>
            /// Max compression and extension acceleration excerted by the spring length controller
            /// </summary>
            public double UDMax;

            /// <summary>
            /// Max angular acceleration excerted by the virtual foot joint
            /// </summary>
            public double UThetaMax;

            /// <summary>
            /// Expression representing the gravity vector
            /// </summary>
            public MatlabExpression GravityVector
            {
                get { return M.ColVector(0, Gravity); }
            }
		}
		#endregion
	}
}
