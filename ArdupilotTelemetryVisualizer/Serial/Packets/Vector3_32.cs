using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	[StructLayout(LayoutKind.Explicit)]
	internal struct Vector3_32
	{
		[FieldOffset(0)]
		public readonly Int32 Int32X;
		[FieldOffset(4)]
		public readonly Int32 Int32Y;
		[FieldOffset(8)]
		public readonly Int32 Int32Z;
		[FieldOffset(0)]
		public readonly UInt32 UInt32X;
		[FieldOffset(4)]
		public readonly UInt32 UInt32Y;
		[FieldOffset(8)]
		public readonly UInt32 UInt32Z;
		[FieldOffset(0)]
		public readonly float SingleX;
		[FieldOffset(4)]
		public readonly float SingleY;
		[FieldOffset(8)]
		public readonly float SingleZ;
 
		public Vector3_32(byte[] data, int offset)
			: this()
		{
			Int32X = BitConverter.ToInt32(data, offset);
			Int32Y = BitConverter.ToInt32(data, offset + 4);
			Int32Z = BitConverter.ToInt32(data, offset + 8);
		}
	}
}
