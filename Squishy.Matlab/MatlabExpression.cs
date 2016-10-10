using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Squishy.Matlab
{
	/// <suNmmary>
	/// Statements can be either opening, closing, neither or both
	/// </summary>
	[Flags]
	public enum MatlabStatementFlags
	{
		None = 0,
		Opening = 1,
		Closing = 2
	}

	public class MatlabStatement : MatlabExpression
	{
		public MatlabStatement(object val, MatlabStatementFlags flags = MatlabStatementFlags.None) : base(val)
		{
			Flags = flags;
		}

		public MatlabStatementFlags Flags { get; private set; }

		public override void Write(MatlabWriter writer)
		{
			if ((Flags & MatlabStatementFlags.Closing) != 0)
			{
				--writer.IndentLevel;
			}
			base.Write(writer);
			if ((Flags & MatlabStatementFlags.Opening) != 0)
			{
				++writer.IndentLevel;
			}
		}
	}

    public class MatlabExpression
    {
        protected MatlabExpression()
        { }

        public MatlabExpression(object val)
        {
            Value = val;
        }

        public static implicit operator MatlabExpression(double value)
        {
            return new MatlabExpression(value);
        }
        public static implicit operator MatlabExpression(float value)
        {
            return new MatlabExpression(value);
        }

        public static implicit operator MatlabExpression(int value)
        {
            return new MatlabExpression(value);
        }

        public static implicit operator MatlabExpression(long value)
        {
            return new MatlabExpression(value);
        }

        public static implicit operator MatlabExpression(string value)
        {
            return new MatlabExpression(value);
        }

        public virtual int Dimensions { get { return 0; } }
        
        public object Value { get; set; }


		public virtual void Write(MatlabWriter writer)
		{
			var str = ToString();
			var stmts = str.Split(new char[] { '\n' });
			if (stmts.Length > 0)
			{
				++writer.IndentLevel;
				foreach (var stmt in stmts)
				{
					writer.WriteLine(stmt);
				}
				--writer.IndentLevel;
			}
			else
			{
				writer.WriteLine(str);
			}
		}

        public override string ToString()
        {
            return M.ConvertName(Value).ToString();
        }
    }

    public class MatlabStatementList : MatlabExpression
    {
        public MatlabStatementList(params MatlabExpression[] exprs) : base(exprs)
        {
        }

        public override string ToString()
        {
            return string.Join(Environment.NewLine, (IEnumerable<MatlabExpression>)Value);
        }

        public override void Write(MatlabWriter writer)
        {
            foreach (var expr in (IEnumerable<MatlabExpression>)Value)
            {
                expr.Write(writer);
            }
        }
    }
}
