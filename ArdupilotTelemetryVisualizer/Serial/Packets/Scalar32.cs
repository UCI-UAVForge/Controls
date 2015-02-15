using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	[StructLayout(LayoutKind.Explicit)]
	struct Scalar32
	{
		[FieldOffset(0)]
		public readonly int Int32Value;
		[FieldOffset(0)]
		public readonly uint UInt32Value;
		[FieldOffset(0)]
		public readonly float SingleValue;

		public Scalar32(UInt32 data) : this()
		{
			UInt32Value = data;
		}
	}
}
