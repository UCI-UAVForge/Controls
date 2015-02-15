using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	[StructLayout(LayoutKind.Explicit)]
	internal struct Vector3_16
	{
		[FieldOffset(0)]
		public readonly Int16 Int16X;
		[FieldOffset(2)]
		public readonly Int16 Int16Y;
		[FieldOffset(4)]
		public readonly Int16 Int16Z;
		[FieldOffset(0)]
		public readonly UInt16 UInt16X;
		[FieldOffset(2)]
		public readonly UInt16 UInt16Y;
		[FieldOffset(4)]
		public readonly UInt16 UInt16Z;
 
		public Vector3_16(byte[] data, int offset)
			: this()
		{
			Int16X = BitConverter.ToInt16(data, offset);
			Int16Y = BitConverter.ToInt16(data, offset + 2);
			Int16Z = BitConverter.ToInt16(data, offset + 4);
		}
	}
}
