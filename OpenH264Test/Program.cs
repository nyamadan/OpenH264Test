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
            var _Uninitialize = Marshal.GetDelegateForFunctionPointer<Uninitialize>(methods[3]);
            var _EncodeFrame = Marshal.GetDelegateForFunctionPointer<EncodeFrame>(methods[4]);
            var _SetOption = Marshal.GetDelegateForFunctionPointer<SetOption>(methods[7]);

            // Log: detail
            unsafe
            {
                var logLevel = 1 << 4;
                rv = _SetOption(pEncoder, 25, new IntPtr(&logLevel));
                Debug.Assert(rv == 0);
            }

            const int width = 640;
            const int height = 480;

            var param = new SEncParamBase();
            param.UsageType = 1;
            param.PicWidth = width;
            param.PicHeight = height;
            param.TargetBitrate = 500000;
            param.RCMode = 0;
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
            info.LayerNum = 0;
            info.FrameType = 0;
            info.FrameSizeInBytes = 0;
            info.TimeStamp = 0L;

            var pic = new SSourcePicture();
            pic.PicWidth = width;
            pic.PicHeight = height;
            pic.ColorFormat = 23;
            pic.TimeStamp = 0;
            unsafe
            {
                pic.Stride[0] = pic.PicWidth;
                pic.Stride[1] = pic.Stride[2] = pic.PicWidth >> 1;

                fixed (void* pData = data)
                {
                    pic.pData[0] = (long) pData;
                    pic.pData[1] = pic.pData[0] + width * height;
                    pic.pData[2] = pic.pData[1] + ((width * height) >> 2);

                    rv = _EncodeFrame(pEncoder, new IntPtr(&pic), new IntPtr(&info));
                    Debug.Assert(rv == 0);

                    var buf = info.pLayerInfo;
                    for(var i = 0; i < 40; i++)
                    {
                        Console.WriteLine($"{i}: {buf[i].ToString("X2")}");
                    }

                    var p = (SLayerBSInfo *)info.pLayerInfo;
                    Console.WriteLine(p->NalCount);
                }
            }

            if (pEncoder != IntPtr.Zero)
            {
                rv = _Uninitialize(pEncoder);
                Debug.Assert(rv == 0);
                WelsDestroySVCEncoder(pEncoder);
                pEncoder = IntPtr.Zero;
            }

            return 0;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SEncParamBase
        {
            public int UsageType;
            public int PicWidth;
            public int PicHeight;
            public int TargetBitrate;
            public int RCMode;
            public float MaxFrameRate;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SSourcePicture
        {
            public int ColorFormat;
            public unsafe fixed int Stride[4];
            public unsafe fixed long pData[4];
            public int PicWidth;
            public int PicHeight;
            public long TimeStamp;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SLayerBSInfo
        {
            public byte TemporalId;
            public byte SpatialId;
            public byte QualityId;
            public int FrameType;
            public byte LayerType;
            public int SubSeqId;
            public int NalCount;
            public IntPtr pNalLengthInByte;
            public IntPtr pBsBuf;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SFrameBSInfo
        {
            public int LayerNum;
            public unsafe fixed byte pLayerInfo[5120];
            public int FrameType;
            public int FrameSizeInBytes;
            public long TimeStamp;
        }

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int Initialize(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int Uninitialize(IntPtr self);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int EncodeFrame(IntPtr self, IntPtr picInfo, IntPtr bsInfo);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int SetOption(IntPtr self, int eOptionId, IntPtr pOption);
    }
}