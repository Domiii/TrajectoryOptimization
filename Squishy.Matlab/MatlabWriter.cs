using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using WCell.Util;

namespace Squishy.Matlab
{
    public class MatlabWriter : IndentTextWriter, IDisposable
	{	
		private int bracketCount;
		private string m_fileName;
		private bool raisedException;
        private string backup;

        /// <summary>
        /// Converts given symbols before appending it to the code
        /// </summary>
		protected static Func<object, object> SymbolConverter;

		public MatlabWriter(string fileName)
		{
			if (File.Exists(fileName))
			{
				backup = File.ReadAllText(fileName);
			}

			try
			{
				var writer = new StreamWriter(m_fileName = fileName);
				SetWriter(writer);
			}
			catch (Exception e)
			{
				OnException(e);
			}
			Indenter = "  ";
		}

		protected virtual void OnException(Exception e)
		{
            // try revert
            Revert();

            // throw exception
			throw e;
			//throw new NotImplementedException();
		}

		public MatlabWriter(TextWriter writer)
		{
			SetWriter(writer);
		}

        #region I/O Management
        /// <summary>
		/// Whether an Exception was raised during writing of the file.
		/// </summary>
		public bool RaisedException
		{
			get { return raisedException; }
		}

		/// <summary>
		/// The content of the file before (or null if there was none).
		/// </summary>
		public string Backup
		{
			get { return backup; }
		}

		/// <summary>
		/// Executes the given action. 
		/// If an Exception is raised, the Exception handler will be called and the file will be reverted.
		/// </summary>
		/// <param name="action"></param>
		public void ExecuteSafely(Action action)
		{
			try
			{
				action();
			}
			catch (Exception ex)
			{
				OnException(ex);
			}
		}

		public void Revert()
		{
			try
			{
				Dispose();
			}
			finally
			{
				if (backup != null)
				{
					File.WriteAllText(m_fileName, backup);
				}
				else
				{
					// File didn't exist before - Lets remove it.
					File.Delete(m_fileName);
				}
			}
		}
        #endregion

		public void Write(MatlabExpression expr)
		{
            ExecuteSafely(() =>
            {
                expr.Write(this);
                WriteLine();
            });
		}
    }
}
