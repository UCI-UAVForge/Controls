using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ArdupilotTelemetryVisualizer.Serial.Packets;

namespace ArdupilotTelemetryVisualizer.Serial
{
	/// <summary>
	/// Provides data for the <see cref="InputHandler.PacketReceived"/> event.
	/// </summary>
	internal class PacketReceivedEventArgs : EventArgs
	{
		public Packet Packet { get; private set; }
		public DateTime ReceivedTime { get; private set; }

		internal PacketReceivedEventArgs(Packet packet)
		{
			Packet = packet;
			ReceivedTime = DateTime.Now;
		}
	}
}
