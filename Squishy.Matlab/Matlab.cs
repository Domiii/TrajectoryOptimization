using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
	/// <summary>
	/// Generates basic Matlab expression
	/// </summary>
	public static class M
	{

		#region Matlab Code
		#endregion


		#region Expression Generators

		public static MatlabExpression FunctionDef<T>(T[] returnArgNames, object fname, params object[] args)
		{
			return new MatlabStatement(
                string.Format("{0} [{1}] = {2}({3})", ConvertName(Keyword.Function), string.Join(",", returnArgNames.Select(arg => ConvertName(arg))),
                ConvertName(fname), 
                string.Join(",", args.Select(arg => ConvertName(arg)))),
				MatlabStatementFlags.Opening);
		}

        public static MatlabExpression FunctionDef(object fname, params object[] args)
		{
			return new MatlabStatement(
                string.Format("{0} {1}({2})", ConvertName(Keyword.Function), ConvertName(fname), string.Join(",", args.Select(arg => ConvertName(arg)))),
				MatlabStatementFlags.Opening);
		}

		public static MatlabExpression End()
		{
			return new MatlabStatement(
				ConvertName(Keyword.End),
				MatlabStatementFlags.Closing);
		}

        public static MatlabExpression CallFunction(object fname, params object[] args)
        {
            return new MatlabStatement(
                string.Format("{0}({1})", ConvertName(fname), string.Join(",", args.Select(arg => ConvertName(arg))))
                );
        }

        public static MatlabExpression CallFunctionInline(object fname, params object[] args)
        {
            return new MatlabStatement(
                string.Format("{0}({1})", ConvertName(fname), string.Join(",", args.Select(arg => ConvertName(arg))))
                );
        }

        public static MatlabExpression Assign(object varName, object value)
        {
            return new MatlabStatement(
                string.Format("{0} = {1};", ConvertName(varName), ConvertName(value))
                );
        }

        public static MatlabExpression AssignInline(object varName, object value)
        {
            return new MatlabStatement(
                string.Format("{0} = {1}", ConvertName(varName), ConvertName(value))
                );
        }

		public static MatlabExpression If(MatlabExpression condition)
		{
			return new MatlabStatement("if " + condition, MatlabStatementFlags.Opening);
		}

		public static MatlabExpression If(MatlabExpression condition, params MatlabExpression[] code)
		{
            var n = 2 + code.Length;
            var arr = new MatlabExpression[n];
            arr[0] = If(condition);
            for (var i = 1; i < n-1; ++i)
            {
                arr[i] = code[i - 1];
            }
            arr[n - 1] = End();
			return new MatlabStatementList(arr);
		}

		/// <summary>
		/// Creates a reference to a function (@fun)
		/// </summary>
        public static MatlabExpression RefFunction(object funName)
		{
			return "@" + ConvertName(funName);
        }

        public static MatlabRowVector RowVector<T>(List<T> vec)
        {
            return new MatlabRowVector(vec.Cast<object>().ToArray());
        }

        public static MatlabColVector ColVector<T>(List<T> vec)
        {
            return new MatlabColVector(vec.Cast<object>().ToArray());
        }

        public static MatlabRowVector RowVector<T>(params T[] vec)
        {
            return new MatlabRowVector(vec.Cast<object>().ToArray());
        }

        public static MatlabColVector ColVector<T>(params T[] vec)
        {
            return new MatlabColVector(vec.Cast<object>().ToArray());
        }

		public static MatlabExpression CellArray(MatlabExpression expr)
		{
			return "{" + expr + "}";
		}

		public static MatlabExpression CellArray(IEnumerable<string> expr)
		{
			return "{" + string.Join(", ", expr) + "}";
		}
		#endregion


        #region Common Operators
        public static string Add(object a, object b)
		{
			return string.Format("{0} + {1}", a, b);
		}

		public static string Subtract(object a, object b)
		{
			return string.Format("{0} - {1}", a, b);
		}

		public static string Mult(object a, object b)
		{
			return string.Format("{0} * {1}", a, b);
		}

		public static string Div(object a, object b)
		{
			return string.Format("({0}) / ({1})", a, b);
		}

        public static string Range(int from, int to)
        {
            if (from == 0)
                from.ToString();
            return (to - from) > 0 ? string.Format("{0}:{1}", from, to) : from.ToString();
        }
		#endregion


        #region Some Commonly Used Matlab Functions
        public static MatlabExpression Zeros(params int[] dims)
        {
            if (dims.Aggregate((a, b) => a * b) == 1)
            {
                return 0;
            }
            return new MatlabExpression(string.Format("zeros({0})", string.Join(", ", dims)));
        }

		public static MatlabExpression Cell(params int[] dims)
		{
			return new MatlabExpression(string.Format("cell({0})", string.Join(", ", dims)));
		}
		
		public static MatlabExpression Transpose(MatlabExpression expr)
        {
			return new MatlabExpression(string.Format("({0})'", expr));
        }

		public static MatlabExpression Sum(MatlabExpression expr)
        {
            return new MatlabExpression(string.Format("sum([{0}])", expr));
        }

		/// <summary>
		/// norm(to - from)
		/// </summary>
		public static MatlabExpression Norm(MatlabExpression to, MatlabExpression from)
		{
			return new MatlabExpression(string.Format("norm({0} - {1})", to, from));
		}

        /// <summary>
        /// Identity matrix
        /// </summary>
        public static MatlabExpression Eye(int n)
        {
            if (n == 1) return 1;
            return CallFunctionInline("eye", n);
        }

		/// <summary>
		/// Gradient of: norm(to - from) with respect to "to"
		/// </summary>
		public static MatlabExpression NormGrad(MatlabExpression to, MatlabExpression from)
		{
			return new MatlabExpression(string.Format("({0} - {1})/norm({0} - {1})", to, from));
		}

		/// <summary>
		/// Gradient of: norm(to - from) with respect to "to"
		/// </summary>
		public static MatlabExpression NormGrad(MatlabExpression vec)
		{
			return new MatlabExpression(string.Format("{0}/norm({0})", vec));
		}
        #endregion


        #region Name Management
        /// <summary>
        /// Convert contents of a NameType object to another form
        /// </summary>
        public static object ConvertName(object o)
		{
			if (o.GetType().IsEnum)
			{
				o = NameAttribute.GetName((Enum)o);
			}
			return o;
		}

		public class NameAttribute : Attribute
		{
			public NameAttribute(string name)
			{
				Name = name;
			}

			public string Name { get; set; }

			/// <summary>
			/// Enum names can use the NameAttribute to assign custom names
			/// </summary>
			public static string GetName(Enum symName)
			{
				FieldInfo fi = symName.GetType().GetField(symName.ToString());
				NameAttribute[] attrs = fi.GetCustomAttributes(typeof(NameAttribute), false) as NameAttribute[];
				if (attrs.Length > 0)
				{
					return attrs[0].Name;
				}
				return symName.ToString();
			}
		}
		#endregion

		#region Matlab Keywords
		public enum Keyword
		{
			[Name("if")]
			If,

			[Name("else")]
			Else,

			[Name("end")]
			End,

			[Name("function")]
			Function,

			[Name("nargout")]
			OutArgCount,

			[Name("Inf")]
			Inf,

			[Name("-Inf")]
			NegInf
		}

		public enum Operator
		{
			[Name("=")]
			Eq,
			[Name(">")]
			Gt,
			[Name("<")]
			Lt,
			[Name(">=")]
			Geq,
			[Name("<=")]
			Leq,
			[Name("")]
			Not
		}
		#endregion
	}

    /// <summary>
    /// We use this dummy-type, so we can call the symbol converter on all Matlab names before writing to file
    /// </summary>
    public struct NameType
    {
        public NameType(object value) : this()
        {
            this.Value = value;
        }

        public object Value { get; set; }

		//public static explicit operator NameType(object value)
		//{
		//	return new NameType(M.ConvertName(value));
		//}

        public override string ToString()
        {
            return Value.ToString();
        }
    }

	/// <summary>
	/// A dense row vector of matlab objects
	/// </summary>
	public class MatlabRowVector : MatlabExpression, IEnumerable
	{
        public MatlabRowVector(object[] vector)
            : base(vector)
        {
        }

        public object[] Array { get { return (object[])Value; } }

        public override string ToString()
        {
            if (Array.Length == 1) return Array[0].ToString();
            return string.Format("[{0}]", string.Join(", ", Array));
        }

        public IEnumerator<object> GetEnumerator()
        {
            foreach (var val in Array) yield return val;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return Array.GetEnumerator();
        }
	}

	/// <summary>
	/// A dense column vector of matlab objects
	/// </summary>
	public class MatlabColVector : MatlabExpression, IEnumerable<object>
    {
        public MatlabColVector(object[] vector)
            : base(vector)
        {
        }

        public object[] Array { get { return (object[])Value; } }

		public override string ToString()
		{
            if (Array.Length == 1) return Array[0].ToString();
            return string.Format("[{0}]", string.Join("; ", Array));
        }

        public IEnumerator<object> GetEnumerator()
        {
            foreach (var val in Array) yield return val;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return Array.GetEnumerator();
        }
	}
}
