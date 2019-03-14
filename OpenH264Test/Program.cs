using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace OpenH264Test
{
    internal class Program
    {
        private const string OpenH264LibraryName = "openh264-1.8.0-win64.dll";

        [DllImport(OpenH264LibraryName)]
        public static extern int WelsCreateSVCEncoder(IntPtr ppEncoder);

        [DllImport(OpenH264LibraryName)]
        public static extern int WelsDestroySVCEncoder(IntPtr pEncoder);

        private static int Main(string[] args)
        {
            if (!Environment.Is64BitProcess) return -1;

            var pEncoder = IntPtr.Zero;
            var rv = 0;

            unsafe
            {
                rv = WelsCreateSVCEncoder(new IntPtr(&pEncoder));
            }

            var methods = new IntPtr[10];
            Marshal.Copy(Marshal.ReadIntPtr(pEncoder, 0), methods, 0, methods.Length);
            var _Initialize = Marshal.GetDelegateForFunctionPointer<Initialize>(methods[0]);
            var _SetOption = Marshal.GetDelegateForFunctionPointer<SetOption>(methods[7]);

            // Log: detail
            unsafe
            {
                var logLevel = 1 << 4;
                rv = _SetOption(pEncoder, 25, new IntPtr(&logLevel));
                Debug.Assert(rv == 0);
            }

            const int width = 320;
            const int height = 240;

            var param = new SEncParamBase();
            param.UsageType = 0;
            param.PicWidth = width;
            param.PicHeight = height;
            param.TargetBitrate = 500000;
            param.RCMode = 1;
            param.MaxFrameRate = 15.0f;

            unsafe
            {
                rv = _Initialize(pEncoder, new IntPtr(&param));
                Debug.Assert(rv == 0);
            }

            // I420
            unsafe
            {
                var videoFormat = 23;
                rv = _SetOption(pEncoder, 0, new IntPtr(&videoFormat));
                Debug.Assert(rv == 0);
            }

            var frameSize = width * height * 3 / 2;
            var data = new byte[frameSize];

            var info = new SFrameBSInfo();
            info.LayerInfo = new SLayerBSInfo[128];

            var pic = new SSourcePicture();
            pic.PicWidth = width;
            pic.PicHeight = height;
            pic.ColorFormat = 23;
            pic.Stride = new int[4];
            pic.Data = new IntPtr[4];
            pic.Stride[0] = pic.PicWidth;
            pic.Stride[1] = pic.Stride[2] = pic.PicWidth >> 1;

            unsafe
            {
                fixed (void* pData = data)
                {
                    pic.Data[0] = new IntPtr(pData);
                    pic.Data[1] = pic.Data[0] + width * height;
                    pic.Data[2] = pic.Data[1] + ((width * height) >> 2);
                }
            }

            WelsDestroySVCEncoder(pEncoder);

            return 0;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct SEncParamBase
        {
            public int UsageType;
            public int PicWidth;
            public int PicHeight;
            public int TargetBitrate;
            public int RCMode;
            public float MaxFrameRate;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct SSourcePicture
        {
            public int ColorFormat;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public int[] Stride;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public IntPtr[] Data;

            public int PicWidth;
            public int PicHeight;
            public long TimeStamp;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct SLayerBSInfo
        {
            public byte TemporalId;
            public byte SpatialId;
            public byte QualityId;
            public int FrameType;
            public byte LayerType;

            public int SubSeqId;
            public int NalCount;
            public IntPtr NalLengthInByte;
            public IntPtr BsBuf;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct SFrameBSInfo
        {
            public int LayerNum;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
            public SLayerBSInfo[] LayerInfo;

            public int FrameType;
            public int FrameSizeInBytes;
            public long TimeStamp;
        }

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int Initialize(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int SetOption(IntPtr self, int eOptionId, IntPtr pOption);
    }
}