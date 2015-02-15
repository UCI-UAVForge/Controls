using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	class SSOutputPacket : Packet
	{
		private readonly ushort flags;

		public SSOutputPacket(ushort flags)
			: base(PacketType.SSOutput)
		{
			this.flags = flags;
		}

		public override byte[] GetBytes()
		{
			byte[] bytes = new byte[5];
			bytes[0] = (byte)((int)Type | 0x20);
			bytes[1] = (byte)flags;
			bytes[2] = (byte)(flags >> 8);
			AppendChecksum(bytes);
			return bytes;
		}
	}
}
