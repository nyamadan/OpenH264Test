using System;
using System.IO;
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
        public static extern void WelsDestroySVCEncoder(IntPtr pEncoder);

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
            var _InitializeExt = Marshal.GetDelegateForFunctionPointer<InitializeExt>(methods[1]);
            var _GetDefaultParams = Marshal.GetDelegateForFunctionPointer<GetDefaultParams>(methods[2]);
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

            const int VideoFormatI420 = 23;
            const int TotalFrame = 100;
            const int Width = 320;
            const int Height = 240;

            /*
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
            */

            var paramExt = new SEncParamExt();
            unsafe
            {
                rv = _GetDefaultParams(pEncoder, new IntPtr(&paramExt));
                Debug.Assert(rv == 0);
            }

            paramExt.UsageType = 1;
            paramExt.PicWidth = Width;
            paramExt.PicHeight = Height;
            paramExt.TargetBitrate = 500000;
            paramExt.RCMode = 0;
            paramExt.MaxFrameRate = 30001f / 1001f;

            paramExt.EnableAdaptiveQuant = false;
            paramExt.EnableBackgroundDetection = false;
            paramExt.MinQp = 25;
            paramExt.MaxQp = 36;

            unsafe
            {
                rv = _InitializeExt(pEncoder, new IntPtr(&paramExt));
                Debug.Assert(rv == 0);
            }

            // I420
            unsafe
            {
                var videoFormat = VideoFormatI420;
                rv = _SetOption(pEncoder, 0, new IntPtr(&videoFormat));
                Debug.Assert(rv == 0);
            }

            var info = new SFrameBSInfo();
            info.LayerNum = 0;
            info.FrameType = 0;
            info.FrameSizeInBytes = 0;
            info.TimeStamp = 0L;

            var pic = new SSourcePicture();
            pic.PicWidth = Width;
            pic.PicHeight = Height;
            pic.ColorFormat = VideoFormatI420;
            pic.TimeStamp = 0L;

            var bufferRGB = new float[Width * Height * 3];
            var bufferYUV = new byte[Width * Height * 3 / 2];
            unsafe
            {
                pic.Stride[0] = pic.PicWidth;
                pic.Stride[1] = pic.Stride[2] = pic.PicWidth >> 1;
            }

            using (var fs = new FileStream("test.264", FileMode.Create))
            using (var bw = new BinaryWriter(fs))
            {
                for (var frameIdx = 0; frameIdx < TotalFrame; frameIdx++)
                {
                    // render rgb
                    for (var y = 0; y < Height; y++)
                    {
                        for (var x = 0; x < Width; x++)
                        {
                            var rgbIdx = 3 * (y * Width + x);
                            bufferRGB[rgbIdx + 0] = (float)x / (Width - 1);
                            bufferRGB[rgbIdx + 1] = (float)y / (Height - 1);
                            bufferRGB[rgbIdx + 2] = (float)frameIdx / (TotalFrame - 1);
                        }
                    }

                    // encode yuv
                    for (var y = 0; y < Height; y++)
                    {
                        for (var x = 0; x < Width; x++)
                        {
                            var rgbIdx = 3 * (y * Width + x);
                            var red = 255f * bufferRGB[rgbIdx + 0];
                            var green = 255f * bufferRGB[rgbIdx + 1];
                            var blue = 255f * bufferRGB[rgbIdx + 2];
                            bufferYUV[y * Width + x] =
                                (byte)(0.2569f * red + 0.5044f * green + 0.0979f * blue + 16f);
                        }
                    }

                    for (var y = 0; y < Height >> 1; y++)
                    {
                        for (var x = 0; x < Width >> 1; x++)
                        {
                            var rgbIdx = 3 * (y * Width + x);
                            var red = 255f * bufferRGB[rgbIdx + 0];
                            var green = 255f * bufferRGB[rgbIdx + 1];
                            var blue = 255f * bufferRGB[rgbIdx + 2];

                            var yuvIdx = Width * Height;
                            bufferYUV[yuvIdx + (Width >> 1) * y + x] =
                                (byte)(-0.1483f * red - 0.2911f * green + 0.4394f * blue + 128f);

                            yuvIdx += (Width >> 1) * (Height >> 1);
                            bufferYUV[yuvIdx + (Width >> 1) * y + x] =
                                (byte)(0.4394f * red - 0.3679 * green - 0.0715 * blue + 128f);
                        }
                    }

                    pic.TimeStamp = (long)Math.Round(frameIdx * (1000 / paramExt.MaxFrameRate));

                    unsafe
                    {
                        fixed (void* pData = bufferYUV)
                        {
                            pic.pData[0] = (long)pData;
                            pic.pData[1] = pic.pData[0] + Width * Height;
                            pic.pData[2] = pic.pData[1] + ((Width * Height) >> 2);

                            rv = _EncodeFrame(pEncoder, new IntPtr(&pic), new IntPtr(&info));
                            Debug.Assert(rv == 0);
                        }
                    }

                    var frameSize = 0;
                    for (var layerIdx = 0; layerIdx < info.LayerNum; layerIdx++)
                    {
                        var layerSize = 0;

                        unsafe
                        {
                            var pLayerBsInfo = (SLayerBSInfo*)info.pLayerInfo + layerIdx;
                            for (var iNalIdx = 0; iNalIdx < pLayerBsInfo->NalCount; iNalIdx++)
                            {
                                var pNalLengthInByte = (int*)pLayerBsInfo->pNalLengthInByte;
                                layerSize += pNalLengthInByte[iNalIdx];
                            }

                            if (paramExt.SpatialLayerNum == 1)
                            {
                                var buf = new byte[layerSize];
                                Marshal.Copy(new IntPtr(pLayerBsInfo->pBsBuf), buf, 0, layerSize);
                                bw.Write(buf);
                            }
                            else
                            {
                                throw new NotImplementedException();
                            }
                        }

                        frameSize += layerSize;
                    }
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
            public long pNalLengthInByte;
            public long pBsBuf;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SFrameBSInfo
        {
            public int LayerNum;
            private int _pad0;
            public unsafe fixed byte pLayerInfo[5120];
            public int FrameType;
            public int FrameSizeInBytes;
            public long TimeStamp;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct SSliceArgument
        {
            public int SliceMode;
            public ushort SliceNum;
            public unsafe fixed uint SliceMbNum[35];
            public uint uiSliceSizeConstraint;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct SSpatialLayerConfig
        {
            public int VideoWidth;
            public int VideoHeight;
            public float FrameRate;
            public int SpatialBitrate;
            public int MaxSpatialBitrate;
            public int ProfileIdc;
            public int LevelIdc;
            public int DLayerQp;

            public unsafe fixed byte SliceArgument[152];

            [MarshalAs(UnmanagedType.U1)] public bool VideoSignalTypePresent;
            public byte uiVideoFormat;
            [MarshalAs(UnmanagedType.U1)] public bool FullRange;
            [MarshalAs(UnmanagedType.U1)] public bool ColorDescriptionPresent;
            public byte ColorPrimaries;
            public byte TransferCharacteristics;
            public byte ColorMatrix;
            [MarshalAs(UnmanagedType.U1)] public bool AspectRatioPresent;
            public int AspectRatio;
            public ushort AspectRatioExtWidth;
            public ushort AspectRatioExtHeight;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct SEncParamExt
        {
            public int UsageType;

            public int PicWidth;
            public int PicHeight;
            public int TargetBitrate;
            public int RCMode;
            public float MaxFrameRate;

            public int TemporalLayerNum;
            public int SpatialLayerNum;

            public unsafe fixed byte SpatialLayers[800];

            public int ComplexityMode;
            public uint IntraPeriod;
            public int NumRefFrame;
            public int SpsPpsIdStrategy;
            [MarshalAs(UnmanagedType.U1)] public bool PrefixNalAddingCtrl;
            [MarshalAs(UnmanagedType.U1)] public bool EnableSSEI;
            [MarshalAs(UnmanagedType.U1)] public bool SimulcastAVC;
            public int PaddingFlag;
            public int EntropyCodingModeFlag;

            [MarshalAs(UnmanagedType.U1)] public bool EnableFrameSkip;
            public int MaxBitrate;
            public int MaxQp;
            public int MinQp;
            public uint MaxNalSize;

            [MarshalAs(UnmanagedType.U1)] public bool EnableLongTermReference;
            public int LTRRefNum;
            public uint LtrMarkPeriod;

            public ushort MultipleThreadIdc;
            [MarshalAs(UnmanagedType.U1)] public bool UseLoadBalancing;

            public int LoopFilterDisableIdc;
            public int LoopFilterAlphaC0Offset;
            public int LoopFilterBetaOffset;

            [MarshalAs(UnmanagedType.U1)] public bool EnableDenoise;
            [MarshalAs(UnmanagedType.U1)] public bool EnableBackgroundDetection;
            [MarshalAs(UnmanagedType.U1)] public bool EnableAdaptiveQuant;
            [MarshalAs(UnmanagedType.U1)] public bool EnableFrameCroppingFlag;
            [MarshalAs(UnmanagedType.U1)] public bool EnableSceneChangeDetect;

            [MarshalAs(UnmanagedType.U1)] public bool IsLosslessLink;
        }

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int Initialize(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int InitializeExt(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int GetDefaultParams(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int Uninitialize(IntPtr self);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int EncodeFrame(IntPtr self, IntPtr picInfo, IntPtr bsInfo);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        private delegate int SetOption(IntPtr self, int eOptionId, IntPtr pOption);
    }
}