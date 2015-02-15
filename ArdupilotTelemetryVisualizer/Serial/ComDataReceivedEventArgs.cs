using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial
{
	/// <summary>
	/// Provides data for the <see cref="ComController.DataReceived"/> event.
	/// </summary>
	public class ComDataReceivedEventArgs : EventArgs
	{
		public byte[] Data { get; private set; }

		internal ComDataReceivedEventArgs(byte[] data)
		{
			Data = data;
		}
	}
}
