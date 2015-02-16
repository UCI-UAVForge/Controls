using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal class Scalar16Packet : Packet
	{
		private readonly ushort value;

		public ushort Value { get { return value; } }

		public Scalar16Packet(ushort value)
			: base(PacketType.Scalar16)
		{
			this.value = value;
		}

		public override byte[] GetBytes()
		{
			throw new NotImplementedException();
		}
	}
}
