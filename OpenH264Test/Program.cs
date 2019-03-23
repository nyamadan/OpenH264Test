using System;
using System.IO;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace OpenH264Test
{
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

    internal class Program
    {
        static readonly byte[] BoxFTYP = { 0x00, 0x00, 0x00, 0x18, 0x66, 0x74, 0x79, 0x70, 0x6d, 0x70, 0x34, 0x32, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x70, 0x34, 0x32, 0x69, 0x73, 0x6f, 0x6d };

        private const string OpenH264LibraryName = "openh264-1.8.0-win64.dll";

        [DllImport(OpenH264LibraryName)]
        public static extern int WelsCreateSVCEncoder(IntPtr ppEncoder);

        [DllImport(OpenH264LibraryName)]
        public static extern void WelsDestroySVCEncoder(IntPtr pEncoder);

        const int MINIMP4_MAX_SPS = 32;
        const int MINIMP4_MAX_PPS = 256;

        private static unsafe int MP4E__set_sps(MP4E_mux_t mux, int track_id, void* sps, int bytes)
        {
            track_t tr = mux.tracks.data + track_id;
            Debug.Assert(tr.info.track_media_kind == track_media_kind.e_video);
            return append_mem(tr.vsps, sps, bytes) ? MP4E_STATUS_OK : MP4E_STATUS_NO_MEMORY;
        }

        private static unsafe int Mp4H26xWriteNal(mp4_h26x_writer_t h, byte* nal, int length, uint timeStamp90kHz_next)
        {
            const int MP4E_SAMPLE_DEFAULT = 0;   // (beginning of) audio or video frame
            const int MP4E_SAMPLE_RANDOM_ACCESS = 1;   // mark sample as random access point (key frame)
            const int MP4E_SAMPLE_CONTINUATION = 2;   // Not a sample, but continuation of previous sample (new slice)

            var eof = nal + length;
            var sizeof_nal = 0;
            var prev_payload_type = -1;

            for (; ; nal++)
            {
                var payload_type = 0;
                nal = FindNalUnit(nal, (int)(eof - nal), out sizeof_nal);

                if (sizeof_nal == 0)
                    break;

                payload_type = nal[0] & 31;
                var _nal1 = new byte[sizeof_nal * 17 / 16 + 32];
                var _nal2 = new byte[sizeof_nal * 17 / 16 + 32];

                fixed (byte* nal1 = _nal1)
                fixed (byte* nal2 = _nal2)
                {
                    sizeof_nal = remove_nal_escapes(nal1, nal, sizeof_nal);
                    sizeof_nal = transcode_nalu(h.sps_patcher, nal2, sizeof_nal, nal1);

                    if (sizeof_nal == 0)
                    {
                        return 0;
                    }

                    switch (payload_type)
                    {
                        case 7:
                            MP4E__set_sps(h.mux, h.mux_track_id, nal2 + 4, sizeof_nal - 4);
                            h.need_sps = 0;
                            break;

                        case 8:
                            if (h.need_sps != 0)
                                return 0;
                            MP4E__set_pps(h.mux, h.mux_track_id, nal2 + 4, sizeof_nal - 4);
                            h.need_pps = 0;
                            break;

                        case 5:
                            if (h.need_sps != 0)
                                return 0;

                            h.need_idr = 0;

                            if (h.need_sps != 0)
                                return 0;

                            if (h.need_pps == 0 && h.need_idr == 0)
                            {
                                int sample_kind = MP4E_SAMPLE_DEFAULT;
                                nal2[0] = (byte)((sizeof_nal - 4) >> 24);
                                nal2[1] = (byte)((sizeof_nal - 4) >> 16);
                                nal2[2] = (byte)((sizeof_nal - 4) >> 8);
                                nal2[3] = (byte)((sizeof_nal - 4));
                                if (payload_type == prev_payload_type)
                                {
                                    sample_kind = MP4E_SAMPLE_CONTINUATION;
                                }
                                else if (payload_type == 5)
                                {
                                    sample_kind = MP4E_SAMPLE_RANDOM_ACCESS;
                                }
                                prev_payload_type = payload_type;
                                MP4E__put_sample(h.mux, h.mux_track_id, nal2, sizeof_nal, timeStamp90kHz_next, sample_kind);
                            }
                            break;

                        default:
                            if (h.need_sps != 0)
                                return 0;
                            if (h.need_pps == 0 && h.need_idr == 0)
                            {
                                int sample_kind = MP4E_SAMPLE_DEFAULT;
                                nal2[0] = (byte)((sizeof_nal - 4) >> 24);
                                nal2[1] = (byte)((sizeof_nal - 4) >> 16);
                                nal2[2] = (byte)((sizeof_nal - 4) >> 8);
                                nal2[3] = (byte)((sizeof_nal - 4));
                                if (payload_type == prev_payload_type)
                                {
                                    sample_kind = MP4E_SAMPLE_CONTINUATION;
                                }
                                else if (payload_type == 5)
                                {
                                    sample_kind = MP4E_SAMPLE_RANDOM_ACCESS;
                                }
                                prev_payload_type = payload_type;
                                MP4E__put_sample(h.mux, h.mux_track_id, nal2, sizeof_nal, timeStamp90kHz_next, sample_kind);
                            }
                            break;
                    }
                }
            }

            return 1;
        }

        public enum track_media_kind
        {
            e_audio,
            e_video,
            e_private
        }


        public class minimp4_vector_t
        {
            public byte[] data;
            public int bytes;
            public int capacity;
        }

        public class track_t
        {
            public MP4E_track_t info;
            public minimp4_vector_t smpl;  // sample descriptor
            public minimp4_vector_t pending_sample;

            public minimp4_vector_t vsps;  // or dsi for audio
            public minimp4_vector_t vpps;  // not used for audio
            public minimp4_vector_t vvps;  // used for HEVC
        }

        public class MP4E_track_t
        {
            // MP4 object type code, which defined codec class for the track.
            // See MP4E_OBJECT_TYPE_* values for some codecs
            public uint object_type_indication;

            // Track language: 3-char ISO 639-2T code: "und", "eng", "rus", "jpn" etc...
            public byte[] language = new byte[4];

            public track_media_kind track_media_kind;

            // 90000 for video, sample rate for audio
            public uint time_scale;
            public uint default_duration;

            public uint channelcount
            {
                get { return (uint)(0xffffffff & u); }
                set { u = (height << 32) | (int)value; }
            }

            public int width
            {
                get { return (int)(0xffffffff & u); }
                set { u = (height << 32) | value; }
            }

            public int height
            {
                get { return (int)(0xffffffff & (u >> 32)); }
                set { u = (value << 32) | width; }
            }

            public long u;
        }

        public class MP4E_mux_t
        {
            public minimp4_vector_t tracks;

            public long write_pos;
            public long mdat_pos;

            public string text_comment;

            public int sequential_mode_flag;
        }

        public class mp4_h26x_writer_t
        {
            public h264_sps_id_patcher_t sps_patcher;
            public MP4E_mux_t mux;
            public int mux_track_id, is_hevc, need_vps, need_sps, need_pps, need_idr;
        }

        public class h264_sps_id_patcher_t
        {
            public byte[][] sps_cache = new byte[MINIMP4_MAX_SPS][];
            public byte[][] pps_cache = new byte[MINIMP4_MAX_PPS][];
            public int[] sps_bytes = new int[MINIMP4_MAX_SPS];
            public int[] pps_bytes = new int[MINIMP4_MAX_PPS];

            public int[] map_sps = new int[MINIMP4_MAX_SPS];
            public int[] map_pps = new int[MINIMP4_MAX_SPS];
        }

        public class bit_reader_t
        {
            public uint cache;
            public int cache_free_bits;
            public unsafe ushort* buf;
            public unsafe ushort* origin;
            public uint origin_bytes;
        }

        public class bs_t
        {
            public int shift;         // bit position in the cache
            public int cache;    // bit cache
            public unsafe uint* buf;    // current position
            public unsafe uint* origin; // initial position
        }

        private static unsafe void h264e_bs_init_bits(bs_t bs, void* data)
        {
            bs.origin = (uint*)data;
            bs.buf = bs.origin;
            bs.shift = 32;
            bs.cache = 0;
        }

        static uint show_bits(bit_reader_t bs, int n)
        {
            int retval;
            Debug.Assert(n > 0 && n <= 16);
            retval = (int)bs.cache >> (32 - n);
            return (uint)retval;
        }

        static uint get_bits(bit_reader_t bs, int n)
        {
            uint retval = show_bits(bs, n);
            flush_bits(bs, n);
            return retval;
        }

        static unsafe void h264e_bs_put_bits(bs_t bs, uint n, uint val)
        {
            Debug.Assert(((int)val >> (int)n) == 0);
            bs.shift -= (int)n;
            Debug.Assert((int)n <= 32);
            if (bs.shift < 0)
            {
                Debug.Assert(-bs.shift < 32);
                bs.cache |= (int)val >> -bs.shift;
                *bs.buf++ = SWAP32(bs.cache);
                bs.shift = 32 + bs.shift;
                bs.cache = 0;
            }
            bs.cache |= (int)val << bs.shift;
        }

        static int ue_bits(bit_reader_t bs)
        {
            int clz;
            int val;
            for (clz = 0; get_bits(bs, 1) == 0; clz++)
            {
            }
            //get_bits(bs, clz + 1);
            val = (1 << clz) - 1 + (clz != 0 ? (int)get_bits(bs, clz) : 0);
            return val;
        }

        static void h264e_bs_put_golomb(bs_t bs, uint val)
        {
            int size = 0;
            uint t = val + 1;

            do
            {
                size++;
                t = t >> 1;
            } while (t != 0);

            h264e_bs_put_bits(bs, (uint)(2 * size - 1), val + 1);
        }

        static unsafe uint get_pos_bits(bit_reader_t bs)
        {
            // Current bitbuffer position =
            // position of next wobits in the internal buffer
            // minus bs, available in bit cache wobits
            uint pos_bits = (uint)(bs.buf - bs.origin) * 16;
            pos_bits -= (uint)(16 - bs.cache_free_bits);
            Debug.Assert((int)pos_bits >= 0);
            return pos_bits;
        }

        static int remaining_bits(bit_reader_t bs)
        {
            return (int)bs.origin_bytes * 8 - (int)get_pos_bits(bs);
        }

        static void copy_bits(bit_reader_t bs, bs_t bd)
        {
            uint cb, bits;
            int bit_count = remaining_bits(bs);
            while (bit_count > 7)
            {
                cb = (uint)Math.Min(bit_count - 7, 8);
                bits = get_bits(bs, (int)cb);
                h264e_bs_put_bits(bd, cb, bits);
                bit_count -= (int)cb;
            }

            // cut extra zeros after stop-bit
            bits = get_bits(bs, bit_count);
            for (; (bit_count != 0) && ((~bits & 1) != 0); bit_count--)
            {
                bits >>= 1;
            }
            if (bit_count != 0)
            {
                h264e_bs_put_bits(bd, (uint)bit_count, bits);
            }
        }

        static unsafe uint h264e_bs_get_pos_bits(bs_t bs)
        {
            uint pos_bits = (uint)((bs.buf - bs.origin) * 32);
            pos_bits += (uint)(32 - bs.shift);
            Debug.Assert((int)pos_bits >= 0);
            return pos_bits;
        }

        static uint h264e_bs_byte_align(bs_t bs)
        {
            int pos = (int)h264e_bs_get_pos_bits(bs);
            h264e_bs_put_bits(bs, (uint)(-pos & 7), 0);
            return (uint)(pos + (-pos & 7));
        }

        static unsafe int change_sps_id(bit_reader_t bs, bs_t bd, int new_id, int* old_id)
        {
            uint bits, sps_id, i, bytes;
            for (i = 0; i < 3; i++)
            {
                bits = get_bits(bs, 8);
                h264e_bs_put_bits(bd, 8, bits);
            }
            sps_id = (uint)ue_bits(bs); // max = 31

            *old_id = (int)sps_id;
            sps_id = (uint)new_id;
            Debug.Assert(sps_id <= 31);

            h264e_bs_put_golomb(bd, sps_id);
            copy_bits(bs, bd);

            bytes = h264e_bs_byte_align(bd) / 8;
            h264e_bs_flush(bd);
            return (int)bytes;
        }

        static unsafe void h264e_bs_flush(bs_t bs)
        {
            *bs.buf = SWAP32(bs.cache);
        }

        static uint SWAP32(int x)
        {
            return (uint)((((x) >> 24) & 0xFF) | (((x) >> 8) & 0xFF00) | (((x) << 8) & 0xFF0000) | ((x & 0xFF) << 24));
        }

        static uint LOAD_SHORT(ushort x)
        {
            return (uint)((ushort)(x << 8) | (x >> 8));
        }

        static unsafe void flush_bits(bit_reader_t bs, int n)
        {
            Debug.Assert(n >= 0 && n <= 16);
            bs.cache <<= n;
            bs.cache_free_bits += n;
            if (bs.cache_free_bits >= 0)
            {
                bs.cache |= (LOAD_SHORT(*bs.buf)) << bs.cache_free_bits;
                bs.buf++;
                bs.cache_free_bits -= 16;
            }
        }

        static unsafe void set_pos_bits(bit_reader_t bs, uint pos_bits)
        {
            Debug.Assert((int)pos_bits >= 0);

            bs.buf = bs.origin + pos_bits / 16;
            bs.cache = 0;
            bs.cache_free_bits = 16;
            flush_bits(bs, 0);
            flush_bits(bs, (int)pos_bits & 15);
        }

        private static unsafe void init_bits(bit_reader_t bs, void* data, uint data_bytes)
        {
            bs.origin = (ushort*)data;
            bs.origin_bytes = data_bytes;
            set_pos_bits(bs, 0);
        }

        static unsafe bool memcmp(void* a, byte[] b, int length)
        {
            for (var i = 0; i < length; i++)
            {
                if (((byte*)a)[i] != b[i])
                {
                    return false;
                }
            }

            return true;
        }

        static unsafe void memcpy(byte[] dst, void* src, int length)
        {
            for (var i = 0; i < length; i++)
            {
                dst[i] = ((byte*)src)[i];
            }
        }

        static unsafe void memcpy(void* dst, void* src, int length)
        {
            for (var i = 0; i < length; i++)
            {
                ((byte*)dst)[i] = ((byte*)src)[i];
            }
        }

        static unsafe int find_mem_cache(byte[][] cache, int[] cache_bytes, int cache_size, void* mem, int bytes)
        {
            int i;

            if (bytes == 0)
            {
                return -1;
            }

            for (i = 0; i < cache_size; i++)
            {
                if (cache_bytes[i] == bytes && !memcmp(mem, cache[i], bytes))
                {
                    return i; // found
                }
            }
            for (i = 0; i < cache_size; i++)
            {
                if (cache_bytes[i] == 0)
                {
                    cache[i] = new byte[bytes];
                    if (cache[i] != null)
                    {
                        memcpy(cache[i], mem, bytes);
                        cache_bytes[i] = bytes;
                    }
                    return i; // put in
                }
            }
            return -1; // no room
        }

        private static unsafe int patch_pps(h264_sps_id_patcher_t h, bit_reader_t bs, bs_t bd, int new_pps_id, int* old_id)
        {
            int bytes;
            uint pps_id = (uint)ue_bits(bs);  // max = 255
            uint sps_id = (uint)ue_bits(bs);  // max = 31

            *old_id = (int)pps_id;
            sps_id = (uint)h.map_sps[sps_id];
            pps_id = (uint)new_pps_id;

            Debug.Assert(sps_id <= 31);
            Debug.Assert(pps_id <= 255);

            h264e_bs_put_golomb(bd, pps_id);
            h264e_bs_put_golomb(bd, sps_id);
            copy_bits(bs, bd);

            bytes = (int)h264e_bs_byte_align(bd) / 8;
            h264e_bs_flush(bd);
            return bytes;
        }

        private static unsafe void patch_slice_header(h264_sps_id_patcher_t h, bit_reader_t bs, bs_t bd)
        {
            uint first_mb_in_slice = (uint)ue_bits(bs);
            uint slice_type = (uint)ue_bits(bs);
            uint pps_id = (uint)ue_bits(bs);

            pps_id = (uint)h.map_pps[pps_id];

            Debug.Assert(pps_id <= 255);

            h264e_bs_put_golomb(bd, first_mb_in_slice);
            h264e_bs_put_golomb(bd, slice_type);
            h264e_bs_put_golomb(bd, pps_id);
            copy_bits(bs, bd);
        }

        private static unsafe int transcode_nalu(h264_sps_id_patcher_t h, byte* src, int nalu_bytes, byte* dst)
        {
            int old_id;

            bit_reader_t bst = new bit_reader_t();
            bs_t bdt = new bs_t();

            bit_reader_t bs = new bit_reader_t();
            bs_t bd = new bs_t();
            int payload_type = src[0] & 31;

            *dst = *src;
            h264e_bs_init_bits(bd, dst + 1);
            init_bits(bs, src + 1, (uint)nalu_bytes - 1);
            h264e_bs_init_bits(bdt, dst + 1);
            init_bits(bst, src + 1, (uint)nalu_bytes - 1);

            switch (payload_type)
            {
                case 7:
                    {
                        int cb = change_sps_id(bst, bdt, 0, &old_id);
                        int id = find_mem_cache(h.sps_cache, h.sps_bytes, MINIMP4_MAX_SPS, dst + 1, cb);
                        if (id == -1)
                        {
                            return 0;
                        }
                        h.map_sps[old_id] = id;
                        change_sps_id(bs, bd, id, &old_id);
                    }
                    break;
                case 8:
                    {
                        int cb = patch_pps(h, bst, bdt, 0, &old_id);
                        int id = find_mem_cache(h.pps_cache, h.pps_bytes, MINIMP4_MAX_PPS, dst + 1, cb);
                        if (id == -1)
                        {
                            return 0;
                        }
                        h.map_pps[old_id] = id;
                        patch_pps(h, bs, bd, id, &old_id);
                    }
                    break;
                case 1:
                case 2:
                case 5:
                    patch_slice_header(h, bs, bd);
                    break;
                default:
                    memcpy(dst, src, nalu_bytes);
                    return nalu_bytes;
            }

            nalu_bytes = (int)(1 + h264e_bs_byte_align(bd) / 8);
            h264e_bs_flush(bd);

            return nalu_bytes;
        }

        private static unsafe int remove_nal_escapes(byte* dst, byte* src, int h264_data_bytes)
        {
            int i = 0, j = 0, zero_cnt = 0;
            for (j = 0; j < h264_data_bytes; j++)
            {
                if (zero_cnt == 2 && src[j] <= 3)
                {
                    if (src[j] == 3)
                    {
                        if (j == h264_data_bytes - 1)
                        {
                            // cabac_zero_word: no action
                        }
                        else if (src[j + 1] <= 3)
                        {
                            j++;
                            zero_cnt = 0;
                        }
                        else
                        {
                            // TODO: assume end-of-nal
                            //return 0;
                        }
                    }
                    else
                    {
                        return 0;
                    }
                }
                dst[i++] = src[j];
                if (src[j] != 0)
                    zero_cnt = 0;
                else
                    zero_cnt++;
            }
            //while (--j > i) src[j] = 0;
            return i;
        }

        private static unsafe byte* FindNalUnit(byte* h264_data, int h264_data_bytes, out int pnal_unit_bytes)
        {
            var eof = h264_data + h264_data_bytes;
            var zcount = 0;
            var start = FindStartCode(h264_data, h264_data_bytes, out zcount);
            var stop = start;
            if (start != null)
            {
                stop = FindStartCode(start, (int)(eof - start), out zcount);
                while (stop > start && stop[-1] == 0)
                {
                    stop--;
                }
            }

            pnal_unit_bytes = (int)(stop - start - zcount);
            return start;
        }

        private static unsafe byte* FindStartCode(byte* h264_data, int h264_data_bytes, out int zcount)
        {
            var eof = h264_data + h264_data_bytes;
            var p = h264_data;
            do
            {
                int zero_cnt = 1;
                while (p < eof && *p != 0)
                    p++;
                while (p + zero_cnt < eof && p[zero_cnt] == 0)
                    zero_cnt++;
                if (zero_cnt >= 2 && p[zero_cnt] == 1)
                {
                    zcount = zero_cnt + 1;
                    return p + zero_cnt + 1;
                }
                p += zero_cnt;
            } while (p < eof);
            zcount = 0;
            return eof;
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

            using (var f264 = new FileStream("test.264", FileMode.Create))
            using (var bw264 = new BinaryWriter(f264))
            using (var fMP4 = new FileStream("test.mp4", FileMode.Create))
            using (var bwMP4 = new BinaryWriter(fMP4))
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
                                var nalSize = pNalLengthInByte[iNalIdx];

                                int nal_unit_bytes = 0;
                                var nal = FindNalUnit((byte*)pLayerBsInfo->pBsBuf + layerSize, nalSize, out nal_unit_bytes);

                                // WIP

                                layerSize += nalSize;
                            }

                            if (paramExt.SpatialLayerNum == 1)
                            {
                                var buf = new byte[layerSize];
                                Marshal.Copy(new IntPtr(pLayerBsInfo->pBsBuf), buf, 0, layerSize);
                                bw264.Write(buf);
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