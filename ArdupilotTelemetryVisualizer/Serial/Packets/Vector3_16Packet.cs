using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal class Vector3_16Packet : Packet
	{
		private readonly Vector3_16 vect;
		private readonly Vector3_16Type type;

		public Vector3_16 Value { get { return vect; } }
		public Vector3_16Type VectorType { get { return type; } }

		public Vector3_16Packet(Vector3_16Type type, Vector3_16 vect)
			: base(PacketType.Vector3_16)
		{
			this.type = type;
			this.vect = vect;
		}

		public override byte[] GetBytes()
		{
			throw new NotImplementedException();
		}
	}

	internal enum Vector3_16Type
	{
		RotPid = 1
	}
}
