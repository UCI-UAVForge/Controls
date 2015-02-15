using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ArdupilotTelemetryVisualizer.Serial.Packets;

namespace ArdupilotTelemetryVisualizer.Serial
{

	internal class OutputHandler
	{
		private readonly ComController controller;


		public OutputHandler(ComController controller)
		{
			this.controller = controller;
		}

		public void SendSSOutput(ushort flags)
		{
			SSOutputPacket p = new SSOutputPacket(0xFFFF);
			SendPacket(p.GetBytes());
		}

		private void SendPacket(byte[] packet)
		{
			if (this.controller == null)
			{
				throw new InvalidOperationException("No controller");
			}

			this.controller.Send(packet);
		}
	}
}
