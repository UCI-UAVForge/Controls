using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal class Scalar32Packet : Packet
	{
		private readonly Scalar32 value;
		private readonly Scalar32Type scalarType;

		public Scalar32 Value { get { return value; } }
		public Scalar32Type ScalarType { get { return scalarType; } }

		public Scalar32Packet(Scalar32Type type, Scalar32 value)
			: base(PacketType.Scalar32)
		{
			this.scalarType = type;
			this.value = value;
		}

		public override byte[] GetBytes()
		{
			throw new NotImplementedException();
		}
	}

	internal enum Scalar32Type
	{
		Clock = 1,
		Heading,

		Debug = 255
	}
}
