using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	class Vector4Packet : Packet
	{
		private readonly Vector4 vect;
		private readonly Vector4Type type;

		public Vector4 Value { get { return vect; } }
		public Vector4Type VectorType { get { return type; } }

		public Vector4Packet(Vector4Type type, Vector4 vect)
			: base(PacketType.Vector4_16)
		{
			this.type = type;
			this.vect = vect;
		}

		public override byte[] GetBytes()
		{
			throw new NotImplementedException();
		}
	}

	internal enum Vector4Type
	{
		RCInput = 1,
		ControlInput,
		MotorOutput
	}
}
