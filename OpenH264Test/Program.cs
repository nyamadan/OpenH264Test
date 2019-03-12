using System;
using System.Runtime.InteropServices;

namespace OpenH264Test
{
    internal class Program
    {
        private const string OpenH264LibraryPath = "openh264-1.8.0-win64.dll";

        [DllImport(OpenH264LibraryPath)]
        public static extern int WelsCreateSVCEncoder(IntPtr ppEncoder);

        [DllImport(OpenH264LibraryPath)]
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

            // I420
            unsafe
            {
                var videoFormat = 23;
                rv = _SetOption(pEncoder, 0, new IntPtr(&videoFormat));
            }

            // Log: detail
            unsafe
            {
                var logLevel = 1 << 4;
                rv = _SetOption(pEncoder, 25, new IntPtr(&logLevel));
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
                }
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