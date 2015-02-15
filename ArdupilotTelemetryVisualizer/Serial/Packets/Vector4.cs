using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal struct Vector4
	{
		public readonly ushort W;
		public readonly ushort X;
		public readonly ushort Y;
		public readonly ushort Z;

		public Vector4(ushort w, ushort x, ushort y, ushort z)
		{
			W = w;
			X = x;
			Y = y;
			Z = z;
		}
	}
}
