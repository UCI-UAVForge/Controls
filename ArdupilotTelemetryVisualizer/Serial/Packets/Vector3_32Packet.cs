using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal class Vector3_32Packet : Packet
	{
		private readonly Vector3_32 vect;
		private Vector3_32Type type;

		public Vector3_32 Value { get { return vect; } }
		public Vector3_32Type VectorType { get { return type; } }

		public Vector3_32Packet(Vector3_32 vect, Vector3_32Type type)
			: base(PacketType.Vector3_32)
		{
			this.vect = vect;
			this.type = type;
		}

		public override byte[] GetBytes()
		{
			throw new NotImplementedException();
		}
	}

	internal enum Vector3_32Type
	{
		Gryo = 1,
		Attitude,
		Acceleration,
		Velocity,
		Position,
		Gps,
		AttitudePid,
		VelocityPid,
		AltRatePid,
		AltPid
	}
}
