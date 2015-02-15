using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal enum PacketType
	{
		Ping = 1,
		Pong,
		SSOutput,
		SSInput,
		Scalar16 = 8,
		Scalar32,
		Vector2_16,
		Vector2_32,
		Vector3_16,
		Vector3_32,
		Vector4_16
	}
}
