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

        private static byte[] CreateSourcePicture()
        {
            var bytes = new byte[72];
            var offset = 0;

            // int iColorFormat;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // alignment
            offset += 4;

            // int iStride[4];
            for(var i = 0; i < 4; i++)
            {
                BitConverter.GetBytes(0).CopyTo(bytes, offset);
                offset += 4;
            }

            // unsigned char* pData[4];
            for(var i = 0; i < 4; i++)
            {
                BitConverter.GetBytes(0L).CopyTo(bytes, offset);
                offset += 8;
            }

            // int iPicWidth;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // int iPicHeight;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // long long uiTimeStamp;
            BitConverter.GetBytes(0L).CopyTo(bytes, offset);
            offset += 8;

            return bytes;
        }

        private static byte[] CreateFrameBSInfo()
        {
            var bytes = new byte[5144];
            var offset = 0;

            // int iLayerNum;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // alignment
            offset += 4;

            // SLayerBSInfo sLayerInfo[MAX_LAYER_NUM_OF_FRAME];
            for(var i = 0; i < 128; i++)
            {
                offset = ApplyLayerInfo(bytes, offset);
            }

            // EVideoFrameType eFrameType;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // int iFrameSizeInBytes;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // long long uiTimeStamp;
            BitConverter.GetBytes(0L).CopyTo(bytes, offset);
            offset += 8;

            return bytes;
        }

        private static int ApplyLayerInfo(byte[] bytes, int offset)
        {
            // unsigned char uiTemporalId;
            bytes[offset] = 0;
            offset += 1;

            // unsigned char uiSpatialId;
            bytes[offset] = 0;
            offset += 1;

            // unsigned char uiQualityId;
            bytes[offset] = 0;
            offset += 1;

            // alignment
            offset += 1;

            // EVideoFrameType eFrameType;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // unsigned char uiLayerType;
            bytes[offset] = 0;
            offset += 1;

            // alignment
            offset += 3;

            // int iSubSeqId;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // int iNalCount;
            BitConverter.GetBytes(0).CopyTo(bytes, offset);
            offset += 4;

            // alignment
            offset += 4;

            // int* pNalLengthInByte;
            BitConverter.GetBytes(0L).CopyTo(bytes, offset);
            offset += 8;

            // unsigned char* pBsBuf;
            BitConverter.GetBytes(0L).CopyTo(bytes, offset);
            offset += 8;

            return offset;
        }

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

            var param = new byte[24];
            BitConverter.GetBytes(0).CopyTo(param, 0);
            BitConverter.GetBytes(320).CopyTo(param, 4);
            BitConverter.GetBytes(240).CopyTo(param, 8);
            BitConverter.GetBytes(500000).CopyTo(param, 12);
            BitConverter.GetBytes(1).CopyTo(param, 16);
            BitConverter.GetBytes(15.0f).CopyTo(param, 20);

            unsafe
            {
                fixed (byte* p = param)
                {
                    rv = _Initialize(pEncoder, new IntPtr(p));
                    Debug.Assert(rv == 0);
                }
            }

            // I420
            unsafe
            {
                var videoFormat = 23;
                rv = _SetOption(pEncoder, 0, new IntPtr(&videoFormat));
                Debug.Assert(rv == 0);
            }

            WelsDestroySVCEncoder(pEncoder);

            return 0;
        }

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int Initialize(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int SetOption(IntPtr self, int eOptionId, IntPtr pOption);
    }
}