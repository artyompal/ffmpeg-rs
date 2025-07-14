//! This is a Rust port of the FFmpeg command line tool.

#![allow(non_camel_case_types)]

mod bindings {
    pub mod avdevice;
    pub mod avformat;
    pub mod avutil;
    pub mod avutil_bprint;
    pub mod avutil_error;
    pub mod avutil_log;
    pub mod avutil_mem;
    pub mod avutil_time;
    pub mod ffmpeg_h;
    pub mod ffmpeg_sched;
    pub mod ffmpeg_utils;
}

use bindings::avdevice::avdevice_register_all;
use bindings::ffmpeg_h::{avio_closep, avio_flush, avio_write, avformat_network_deinit, avformat_network_init};
use bindings::avutil::{
    AV_LOG_DEBUG, AV_LOG_ERROR, AV_LOG_INFO, AV_LOG_QUIET, AV_LOG_WARNING, AV_TIME_BASE, AV_LOG_SKIP_REPEATED,
};
use bindings::ffmpeg_h::{
    AVIOInterruptCB,  ffmpeg_parse_options, fg_free, fg_send_command, filtergraph_is_simple,
    hw_device_free_all, ifile_close, of_enc_stats_close, of_free, of_filesize, of_write_trailer,
    parse_loglevel, show_banner, show_usage, uninit_opts,
    InputStream, OutputStream,
    AVCodecParameters, avcodec_parameters_alloc, avcodec_parameters_copy, avcodec_parameters_free,
    avcodec_descriptor_get, dec_free,
    AVBufferRef, AVFrame, AVPacket,
    av_buffer_is_writable, av_buffer_create, av_buffer_unref,
    FF_QP2LAMBDA, 
    nb_input_files, input_files, nb_output_files, output_files, nb_filtergraphs, filtergraphs, nb_decoders, decoders, progress_avio,
};
use bindings::ffmpeg_h::{sch_alloc, sch_free, sch_start, sch_stop, sch_wait, Scheduler};
use bindings::avutil_bprint::{av_bprint_finalize, av_bprint_init, av_bprintf, AV_BPRINT_SIZE_AUTOMATIC};
use bindings::avutil_time::av_gettime_relative;
use bindings::ffmpeg_h::{
    av_log, av_log_get_level, av_log_set_flags, av_log_set_level,
};
use bindings::ffmpeg_utils::err_merge;
use bindings::ffmpeg_h::{va_list, vsnprintf};
use bindings::avutil::{AVMEDIA_TYPE_VIDEO, AV_NOPTS_VALUE, AV_LOG_FATAL};
use bindings::avutil_error::{AVERROR, AVERROR_EXIT, FFMPEG_ERROR_RATE_EXCEEDED};
use bindings::ffmpeg_h::{av_free, av_freep, av_mallocz};

use libc::{c_int, c_void, SIGINT, SIGPIPE, SIGQUIT, SIGTERM, SIGXCPU};
use std::ffi::{CStr, CString};
use std::io::{self, Write};
use std::ptr;
use std::sync::atomic::{self, AtomicI32, AtomicU64};
use std::sync::OnceLock;

// Macro to create CString literals for safety and convenience.
macro_rules! c_str {
    ($s:expr) => {
        unsafe {
            CStr::from_bytes_with_nul_unchecked(concat!($s, "\0").as_bytes())
        }
    };
}

// External C variables
unsafe extern "C" {
    pub static program_name: *const libc::c_char;
    pub static program_birth_year: libc::c_int;
    pub static mut vstats_file: *mut libc::FILE;
    pub static mut stdin_interaction: libc::c_int;
    pub static mut do_benchmark: libc::c_int;
    pub static mut do_benchmark_all: libc::c_int;
    pub static mut print_stats: libc::c_int;
    pub static mut stats_period: libc::c_long;
    pub static mut copy_ts: libc::c_int;
    pub static mut filter_nbthreads: *mut libc::c_int;
}

// Global variables, made safe with OnceLock or atomic types
static NB_OUTPUT_DUMPED: AtomicU64 = AtomicU64::new(0);
static mut CURRENT_TIME: OnceLock<BenchmarkTimeStamps> = OnceLock::new();

static mut RESTORE_TTY: c_int = 0;
static mut OLD_TTY: libc::termios =
    libc::termios {
        c_iflag: 0,
        c_oflag: 0,
        c_cflag: 0,
        c_lflag: 0,
        c_line: 0,
        c_cc: [0; libc::NCCS],
        c_ispeed: 0,
        c_ospeed: 0,
    };

static RECEIVED_SIGTERM: AtomicI32 = AtomicI32::new(0);
static RECEIVED_NB_SIGNALS: AtomicI32 = AtomicI32::new(0);
static TRANSCODE_INIT_DONE: AtomicI32 = AtomicI32::new(0);
static FFMPEG_EXITED: AtomicI32 = AtomicI32::new(0);
static mut COPY_TS_FIRST_PTS: i64 = AV_NOPTS_VALUE;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct BenchmarkTimeStamps {
    real_usec: i64,
    user_usec: i64,
    sys_usec: i64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct FrameData {
    dec: FrameDataDec,
    enc: FrameDataEnc,
    wallclock: [i64; 2],
    par_enc: *mut AVCodecParameters,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct FrameDataDec {
    frame_num: u64,
    pts: i64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct FrameDataEnc {
    frame_num: u64,
}

/// Helper function to convert C-style string pointer to Rust `&str`.
unsafe fn c_str_to_rust_str(c_ptr: *const libc::c_char) -> &'static str {
    if c_ptr.is_null() {
        ""
    } else {
        CStr::from_ptr(c_ptr).to_str().unwrap_or("")
    }
}

/// Helper function to convert C-style string pointer to Rust `Option<&str>`.
unsafe fn c_str_to_opt_rust_str(c_ptr: *const libc::c_char) -> Option<&'static str> {
    if c_ptr.is_null() {
        None
    } else {
        CStr::from_ptr(c_ptr).to_str().ok()
    }
}

unsafe extern "C" fn term_exit_sigsafe() {
    if RESTORE_TTY != 0 {
        libc::tcsetattr(0, libc::TCSANOW, &OLD_TTY);
    }
}

/// Public function for `term_exit` used in cleanup.
unsafe extern "C" fn term_exit() {
    av_log(ptr::null_mut(), AV_LOG_QUIET, c_str!("").as_ptr());
    term_exit_sigsafe();
}

unsafe extern "C" fn sigterm_handler(sig: c_int) {
    RECEIVED_SIGTERM.store(sig, atomic::Ordering::SeqCst);
    RECEIVED_NB_SIGNALS.fetch_add(1, atomic::Ordering::SeqCst);
    term_exit_sigsafe();
    if RECEIVED_NB_SIGNALS.load(atomic::Ordering::SeqCst) > 3 {
        let msg = CString::new("Received > 3 system signals, hard exiting\n").unwrap();
        let ret = libc::write(libc::STDERR_FILENO, msg.as_ptr() as *const c_void, msg.as_bytes().len());
        if ret < 0 { /* Do nothing */ };
        libc::exit(123);
    }
}

/// Public function for `term_init`.
unsafe extern "C" fn term_init() {
    let mut action: libc::sigaction = std::mem::zeroed();
    action.sa_sigaction = sigterm_handler as usize;
    libc::sigfillset(&mut action.sa_mask);
    action.sa_flags = libc::SA_RESTART;

    if stdin_interaction != 0 {
        let mut tty: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(0, &mut tty) == 0 {
            OLD_TTY = tty;
            RESTORE_TTY = 1;

            tty.c_iflag &= !(libc::IGNBRK | libc::BRKINT | libc::PARMRK | libc::ISTRIP
                | libc::INLCR | libc::IGNCR | libc::ICRNL | libc::IXON);
            tty.c_oflag |= libc::OPOST;
            tty.c_lflag &= !(libc::ECHO | libc::ECHONL | libc::ICANON | libc::IEXTEN);
            tty.c_cflag &= !(libc::CSIZE | libc::PARENB);
            tty.c_cflag |= libc::CS8;
            tty.c_cc[libc::VMIN] = 1;
            tty.c_cc[libc::VTIME] = 0;

            libc::tcsetattr(0, libc::TCSANOW, &tty);
        }
        libc::sigaction(SIGQUIT, &action, ptr::null_mut());
    }

    libc::sigaction(SIGINT, &action, ptr::null_mut());
    libc::sigaction(SIGTERM, &action, ptr::null_mut());
    libc::sigaction(SIGXCPU, &action, ptr::null_mut());
    libc::signal(SIGPIPE, libc::SIG_IGN);
}

unsafe fn read_key() -> c_int {
    let mut ch: u8 = 0;
    let mut tv = libc::timeval {
        tv_sec: 0,
        tv_usec: 0,
    };
    let mut rfds: libc::fd_set = std::mem::zeroed();

    libc::FD_ZERO(&mut rfds);
    libc::FD_SET(0, &mut rfds);

    let n = libc::select(1, &mut rfds, ptr::null_mut(), ptr::null_mut(), &mut tv);
    if n > 0 {
        let n = libc::read(0, &mut ch as *mut _ as *mut c_void, 1);
        if n == 1 {
            return ch as c_int;
        }
        return n as c_int;
    }

    -1
}

unsafe extern "C" fn decode_interrupt_cb(_ctx: *mut c_void) -> c_int {
    (RECEIVED_NB_SIGNALS.load(atomic::Ordering::SeqCst)
        > TRANSCODE_INIT_DONE.load(atomic::Ordering::SeqCst)) as c_int
}

const INT_CB: AVIOInterruptCB = AVIOInterruptCB {
    callback: Some(decode_interrupt_cb),
    opaque: ptr::null_mut(),
};

unsafe fn ffmpeg_cleanup(ret: c_int) {
    if do_benchmark != 0 {
        let maxrss = getmaxrss() / 1024;
        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("bench: maxrss=%llKiB\n").as_ptr(),
            maxrss,
        );
    }

    for i in 0..nb_filtergraphs {
        fg_free(&mut *filtergraphs.add(i as usize));
    }
    av_freep(&mut filtergraphs as *mut _ as *mut c_void);

    for i in 0..nb_output_files {
        of_free(&mut *output_files.add(i as usize));
    }

    for i in 0..nb_input_files {
        ifile_close(&mut *input_files.add(i as usize));
    }

    for i in 0..nb_decoders {
        dec_free(&mut *decoders.add(i as usize));
    }
    av_freep(&mut decoders as *mut _ as *mut c_void);

    if !vstats_file.is_null() {
        if libc::fclose(vstats_file) != 0 {
            let err_str = av_err2str(AVERROR(io::Error::last_os_error().raw_os_error().unwrap_or(0)));
            let err_cstr = std::ffi::CString::new(err_str).unwrap();
            av_log(
                ptr::null_mut(),
                AV_LOG_ERROR,
                c_str!("Error closing vstats file, loss of information possible: %s\n").as_ptr(),
                err_cstr.as_ptr(),
            );
        }
    }
    // Assuming vstats_filename is a global C pointer that needs to be freed
    let mut vstats_filename_ptr: *mut libc::c_char = ptr::null_mut();
    av_freep(&mut vstats_filename_ptr as *mut _ as *mut c_void);
    of_enc_stats_close();

    hw_device_free_all();

    av_freep(&mut filter_nbthreads as *mut _ as *mut c_void);

    av_freep(&mut input_files as *mut _ as *mut c_void);
    av_freep(&mut output_files as *mut _ as *mut c_void);

    uninit_opts();

    avformat_network_deinit();

    if RECEIVED_SIGTERM.load(atomic::Ordering::SeqCst) != 0 {
        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("Exiting normally, received signal %d.\n").as_ptr(),
            RECEIVED_SIGTERM.load(atomic::Ordering::SeqCst),
        );
    } else if ret != 0 && TRANSCODE_INIT_DONE.load(atomic::Ordering::SeqCst) != 0 {
        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("Conversion failed!\n").as_ptr(),
        );
    }
    term_exit();
    FFMPEG_EXITED.store(1, atomic::Ordering::SeqCst);
}

unsafe fn ost_iter(prev: *mut OutputStream) -> *mut OutputStream {
    let mut of_idx = if prev.is_null() {
        0
    } else {
        (*(*prev).file).index
    };
    let mut ost_idx = if prev.is_null() {
        0
    } else {
        (*prev).index + 1
    };

    while of_idx < nb_output_files {
        let of = *output_files.add(of_idx as usize);
        if ost_idx < (*of).nb_streams {
            return *(*of).streams.add(ost_idx as usize);
        }
        of_idx += 1;
        ost_idx = 0;
    }
    ptr::null_mut()
}

unsafe fn ist_iter(prev: *mut InputStream) -> *mut InputStream {
    let mut if_idx = if prev.is_null() {
        0
    } else {
        (*(*prev).file).index
    };
    let mut ist_idx = if prev.is_null() {
        0
    } else {
        (*prev).index + 1
    };

    while if_idx < nb_input_files {
        let f = *input_files.add(if_idx as usize);
        if ist_idx < (*f).nb_streams {
            return *(*f).streams.add(ist_idx as usize);
        }
        if_idx += 1;
        ist_idx = 0;
    }
    ptr::null_mut()
}

unsafe extern "C" fn frame_data_free(_opaque: *mut c_void, data: *mut u8) {
    let fd = data as *mut FrameData;
    avcodec_parameters_free(&mut (*fd).par_enc);
    av_free(data as *mut c_void);
}

unsafe fn frame_data_ensure(dst: *mut *mut AVBufferRef, writable: c_int) -> c_int {
    let mut src = *dst;

    if src.is_null() || (writable != 0 && av_buffer_is_writable(src) == 0) {
        let mut fd_ptr = av_mallocz(std::mem::size_of::<FrameData>()) as *mut FrameData;
        if fd_ptr.is_null() {
            return AVERROR(libc::ENOMEM);
        }

        *dst = av_buffer_create(
            fd_ptr as *mut u8,
            std::mem::size_of::<FrameData>(),
            Some(frame_data_free),
            ptr::null_mut(),
            0,
        );
        if (*dst).is_null() {
            av_buffer_unref(&mut src);
            av_freep(&mut fd_ptr as *mut _ as *mut c_void);
            return AVERROR(libc::ENOMEM);
        }

        if !src.is_null() {
            let fd_src = (*src).data as *const FrameData;
            ptr::copy_nonoverlapping(fd_src, fd_ptr, 1);
            (*fd_ptr).par_enc = ptr::null_mut();

            if !(*fd_src).par_enc.is_null() {
                let mut ret = 0;
                (*fd_ptr).par_enc = avcodec_parameters_alloc();
                ret = if !(*fd_ptr).par_enc.is_null() {
                    avcodec_parameters_copy((*fd_ptr).par_enc, (*fd_src).par_enc)
                } else {
                    AVERROR(libc::ENOMEM)
                };
                if ret < 0 {
                    av_buffer_unref(dst);
                    av_buffer_unref(&mut src);
                    return ret;
                }
            }
            av_buffer_unref(&mut src);
        } else {
            (*fd_ptr).dec.frame_num = u64::MAX;
            (*fd_ptr).dec.pts = AV_NOPTS_VALUE;

            for i in 0..2 {
                (*fd_ptr).wallclock[i] = i64::MIN;
            }
        }
    }
    0
}

/// Public function for `frame_data`.
unsafe extern "C" fn frame_data(frame: *mut AVFrame) -> *mut FrameData {
    let ret = frame_data_ensure(&mut (*frame).opaque_ref, 1);
    if ret < 0 {
        ptr::null_mut()
    } else {
        (*(*frame).opaque_ref).data as *mut FrameData
    }
}

/// Public function for `frame_data_c`.
unsafe extern "C" fn frame_data_c(frame: *mut AVFrame) -> *const FrameData {
    let ret = frame_data_ensure(&mut (*frame).opaque_ref, 0);
    if ret < 0 {
        ptr::null_mut()
    } else {
        (*(*frame).opaque_ref).data as *const FrameData
    }
}

/// Public function for `packet_data`.
unsafe extern "C" fn packet_data(pkt: *mut AVPacket) -> *mut FrameData {
    let ret = frame_data_ensure(&mut (*pkt).opaque_ref, 1);
    if ret < 0 {
        ptr::null_mut()
    } else {
        (*(*pkt).opaque_ref).data as *mut FrameData
    }
}

/// Public function for `packet_data_c`.
unsafe extern "C" fn packet_data_c(pkt: *mut AVPacket) -> *const FrameData {
    let ret = frame_data_ensure(&mut (*pkt).opaque_ref, 0);
    if ret < 0 {
        ptr::null_mut()
    } else {
        (*(*pkt).opaque_ref).data as *const FrameData
    }
}

unsafe fn update_benchmark(fmt: Option<&CStr>, args: va_list) {
    if do_benchmark_all != 0 {
        let t = get_benchmark_time_stamps();
        if let Some(fmt_cstr) = fmt {
            let mut buf = [0; 1024];
            let _ = vsnprintf(buf.as_mut_ptr() as *mut libc::c_char, buf.len().try_into().unwrap(), fmt_cstr.as_ptr(), args);

            av_log(
                ptr::null_mut(),
                AV_LOG_INFO,
                c_str!("bench: %8llu user %8llu sys %8llu real %s \n").as_ptr(),
                t.user_usec - CURRENT_TIME.get().unwrap().user_usec,
                t.sys_usec - CURRENT_TIME.get().unwrap().sys_usec,
                t.real_usec - CURRENT_TIME.get().unwrap().real_usec,
                buf.as_ptr(),
            );
        }
        CURRENT_TIME.set(t).unwrap(); // Update the global static
    }
}

unsafe fn print_report(is_last_report: c_int, timer_start: i64, cur_time: i64, pts: i64) {
    let mut buf = std::mem::zeroed();
    let mut buf_script = std::mem::zeroed();
    let total_size = of_filesize(*output_files.add(0)); // Assuming output_files[0] exists
    let mut vid = 0;
    let mut last_time: i64 = -1;
    static mut FIRST_REPORT: c_int = 1;
    let mut nb_frames_dup: u64 = 0;
    let mut nb_frames_drop: u64 = 0;

    if print_stats == 0 && is_last_report == 0 && progress_avio.is_null() {
        return;
    }

    if is_last_report == 0 {
        if last_time == -1 {
            last_time = cur_time;
        }
        if ((cur_time - last_time) < stats_period && FIRST_REPORT != 1) ||
            (FIRST_REPORT != 0 && NB_OUTPUT_DUMPED.load(atomic::Ordering::SeqCst) < nb_output_files as u64) {
            return;
        }
        last_time = cur_time;
    }

    let t_elapsed = (cur_time - timer_start) as f64 / 1_000_000.0;

    av_bprint_init(&mut buf, 0, AV_BPRINT_SIZE_AUTOMATIC);
    av_bprint_init(&mut buf_script, 0, AV_BPRINT_SIZE_AUTOMATIC);

    let mut ost = ost_iter(ptr::null_mut());
    while !ost.is_null() {
        let q = if !(*ost).enc.is_null() {
            (*(*ost).quality).load(atomic::Ordering::SeqCst) as f32 / (*FF_QP2LAMBDA) as f32
        } else {
            -1.0
        };

        if vid != 0 && (*ost).type_ == AVMEDIA_TYPE_VIDEO {
            av_bprintf(&mut buf, c_str!("q=%.1f ").as_ptr(), q);
            av_bprintf(
                &mut buf_script,
                c_str!("stream_%d_%d_q=%.1f\n").as_ptr(),
                (*(*ost).file).index,
                (*ost).index,
                q,
            );
        }
        if vid == 0 && (*ost).type_ == AVMEDIA_TYPE_VIDEO {
            let frame_number = (*(*ost).packets_written).load(atomic::Ordering::SeqCst);
            let fps = if t_elapsed > 1.0 { frame_number as f64 / t_elapsed } else { 0.0 };
            let fps_precision = if fps < 9.95 { 1 } else { 0 }; // Similar logic to C's %3.*f

            av_bprintf(
                &mut buf,
                c_str!("frame=%5ll fps=%3.*f q=%3.1f ").as_ptr(),
                frame_number,
                fps_precision,
                fps,
                q,
            );
            av_bprintf(&mut buf_script, c_str!("frame=%ll\n").as_ptr(), frame_number);
            av_bprintf(&mut buf_script, c_str!("fps=%.2f\n").as_ptr(), fps);
            av_bprintf(
                &mut buf_script,
                c_str!("stream_%d_%d_q=%.1f\n").as_ptr(),
                (*(*ost).file).index,
                (*ost).index,
                q,
            );
            if is_last_report != 0 {
                av_bprintf(&mut buf, c_str!("L").as_ptr());
            }

            if !(*ost).filter.is_null() {
                nb_frames_dup = (*(*(*ost).filter).nb_frames_dup).load(atomic::Ordering::SeqCst);
                nb_frames_drop = (*(*(*ost).filter).nb_frames_drop).load(atomic::Ordering::SeqCst);
            }
            vid = 1;
        }
        ost = ost_iter(ost);
    }

    let mut current_pts = pts;
    if copy_ts != 0 {
        if COPY_TS_FIRST_PTS == AV_NOPTS_VALUE && current_pts > 1 {
            COPY_TS_FIRST_PTS = current_pts;
        }
        if COPY_TS_FIRST_PTS != AV_NOPTS_VALUE {
            current_pts -= COPY_TS_FIRST_PTS;
        }
    }

    let us = (current_pts.abs() % AV_TIME_BASE as i64) as c_int;
    let secs = (current_pts.abs() / AV_TIME_BASE as i64 % 60) as c_int;
    let mins = (current_pts.abs() / AV_TIME_BASE as i64 / 60 % 60) as c_int;
    let hours = current_pts.abs() / AV_TIME_BASE as i64 / 3600;
    let hours_sign = if current_pts < 0 { "-" } else { "" };

    let bitrate = if current_pts != AV_NOPTS_VALUE && current_pts != 0 && total_size >= 0 {
        total_size as f64 * 8.0 / (current_pts as f64 / 1000.0)
    } else {
        -1.0
    };
    let speed = if current_pts != AV_NOPTS_VALUE && t_elapsed != 0.0 {
        current_pts as f64 / AV_TIME_BASE as f64 / t_elapsed
    } else {
        -1.0
    };

    if total_size < 0 {
        av_bprintf(&mut buf, c_str!("size=N/A time=").as_ptr());
    } else {
        av_bprintf(&mut buf, c_str!("size=%8.0fKiB time=").as_ptr(), total_size as f64 / 1024.0);
    }
    if current_pts == AV_NOPTS_VALUE {
        av_bprintf(&mut buf, c_str!("N/A ").as_ptr());
    } else {
        av_bprintf(
            &mut buf,
            c_str!("%s%02ll:%02d:%02d.%02d ").as_ptr(),
            hours_sign.as_ptr(),
            hours,
            mins,
            secs,
            (100 * us) / AV_TIME_BASE as c_int,
        );
    }

    if bitrate < 0.0 {
        av_bprintf(&mut buf, c_str!("bitrate=N/A").as_ptr());
        av_bprintf(&mut buf_script, c_str!("bitrate=N/A\n").as_ptr());
    } else {
        av_bprintf(&mut buf, c_str!("bitrate=%6.1fkbits/s").as_ptr(), bitrate);
        av_bprintf(&mut buf_script, c_str!("bitrate=%6.1fkbits/s\n").as_ptr(), bitrate);
    }

    if total_size < 0 {
        av_bprintf(&mut buf_script, c_str!("total_size=N/A\n").as_ptr());
    } else {
        av_bprintf(&mut buf_script, c_str!("total_size=%ll\n").as_ptr(), total_size);
    }
    if current_pts == AV_NOPTS_VALUE {
        av_bprintf(&mut buf_script, c_str!("out_time_us=N/A\n").as_ptr());
        av_bprintf(&mut buf_script, c_str!("out_time_ms=N/A\n").as_ptr());
        av_bprintf(&mut buf_script, c_str!("out_time=N/A\n").as_ptr());
    } else {
        av_bprintf(&mut buf_script, c_str!("out_time_us=%ll\n").as_ptr(), current_pts);
        av_bprintf(&mut buf_script, c_str!("out_time_ms=%ll\n").as_ptr(), current_pts);
        av_bprintf(
            &mut buf_script,
            c_str!("out_time=%s%02ll:%02d:%02d.%06d\n").as_ptr(),
            hours_sign.as_ptr(),
            hours,
            mins,
            secs,
            us,
        );
    }

    if nb_frames_dup != 0 || nb_frames_drop != 0 {
        av_bprintf(
            &mut buf,
            c_str!(" dup=%ll drop=%ll").as_ptr(),
            nb_frames_dup,
            nb_frames_drop,
        );
    }
    av_bprintf(&mut buf_script, c_str!("dup_frames=%ll\n").as_ptr(), nb_frames_dup);
    av_bprintf(&mut buf_script, c_str!("drop_frames=%ll\n").as_ptr(), nb_frames_drop);

    if speed < 0.0 {
        av_bprintf(&mut buf, c_str!(" speed=N/A").as_ptr());
        av_bprintf(&mut buf_script, c_str!("speed=N/A\n").as_ptr());
    } else {
        av_bprintf(&mut buf, c_str!(" speed=%4.3gx").as_ptr(), speed);
        av_bprintf(&mut buf_script, c_str!("speed=%4.3gx\n").as_ptr(), speed);
    }

    if print_stats != 0 || is_last_report != 0 {
        let end = if is_last_report != 0 { b'\n' } else { b'\r' };
        if print_stats == 1 && AV_LOG_INFO > av_log_get_level() {
            let buf_str = CStr::from_ptr(buf.str_).to_string_lossy();
            let _ = io::stderr().write_all(format!("{}    {}\n", buf_str, end as char).as_bytes());
        } else {
            av_log(
                ptr::null_mut(),
                AV_LOG_INFO,
                c_str!("%s    %c").as_ptr(),
                buf.str_,
                end as c_int,
            );
        }
        let _ = io::stderr().flush();
    }
    av_bprint_finalize(&mut buf, ptr::null_mut());

    if !progress_avio.is_null() {
        av_bprintf(
            &mut buf_script,
            c_str!("progress=%s\n").as_ptr(),
            if is_last_report != 0 {
                c_str!("end").as_ptr()
            } else {
                c_str!("continue").as_ptr()
            },
        );
        avio_write(
            progress_avio,
            buf_script.str_ as *const u8,
            buf_script.len.min(buf_script.size as usize - 1).try_into().unwrap(),
        );
        avio_flush(progress_avio);
        av_bprint_finalize(&mut buf_script, ptr::null_mut());
        if is_last_report != 0 {
            let ret = avio_closep(&mut progress_avio);
            if ret < 0 {
                let err_str = av_err2str(ret);
                let err_cstr = std::ffi::CString::new(err_str).unwrap();
                av_log(
                    ptr::null_mut(),
                    AV_LOG_ERROR,
                    c_str!("Error closing progress log, loss of information possible: %s\n").as_ptr(),
                    err_cstr.as_ptr(),
                );
            }
        }
    }
    FIRST_REPORT = 0;
}

unsafe fn print_stream_maps() {
    av_log(ptr::null_mut(), AV_LOG_INFO, c_str!("Stream mapping:\n").as_ptr());

    let mut ist = ist_iter(ptr::null_mut());
    while !ist.is_null() {
        for j in 0..(*ist).nb_filters {
            let filter = *(*ist).filters.add(j as usize);
            if filtergraph_is_simple((*filter).graph) == 0 {
                av_log(
                    ptr::null_mut(),
                    AV_LOG_INFO,
                    c_str!("  Stream #%d:%d (%s) -> %s").as_ptr(),
                    (*(*ist).file).index,
                    (*ist).index,
                    c_str_to_rust_str(if !(*ist).dec.is_null() {
                        (*(*ist).dec).name
                    } else {
                        c_str!("?").as_ptr()
                    }),
                    c_str_to_rust_str((*filter).name),
                );
                if nb_filtergraphs > 1 {
                    av_log(
                        ptr::null_mut(),
                        AV_LOG_INFO,
                        c_str!(" (graph %d)").as_ptr(),
                        (*(*filter).graph).index,
                    );
                }
                av_log(ptr::null_mut(), AV_LOG_INFO, c_str!("\n").as_ptr());
            }
        }
        ist = ist_iter(ist);
    }

    let mut ost = ost_iter(ptr::null_mut());
    while !ost.is_null() {
        if !(*ost).attachment_filename.is_null() {
            av_log(
                ptr::null_mut(),
                AV_LOG_INFO,
                c_str!("  File %s -> Stream #%d:%d\n").as_ptr(),
                (*ost).attachment_filename,
                (*(*ost).file).index,
                (*ost).index,
            );
            ost = ost_iter(ost);
            continue;
        }

        if !(*ost).filter.is_null() && filtergraph_is_simple((*(*ost).filter).graph) == 0 {
            av_log(
                ptr::null_mut(),
                AV_LOG_INFO,
                c_str!("  %s").as_ptr(),
                (*(*ost).filter).name,
            );
            if nb_filtergraphs > 1 {
                av_log(
                    ptr::null_mut(),
                    AV_LOG_INFO,
                    c_str!(" (graph %d)").as_ptr(),
                    (*(*(*ost).filter).graph).index,
                );
            }
            av_log(
                ptr::null_mut(),
                AV_LOG_INFO,
                c_str!(" -> Stream #%d:%d (%s)\n").as_ptr(),
                (*(*ost).file).index,
                (*ost).index,
                c_str_to_rust_str((*(*(*ost).enc_ctx).codec).name),
            );
            ost = ost_iter(ost);
            continue;
        }

        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("  Stream #%d:%d -> #%d:%d").as_ptr(),
            (*(*(*ost).ist).file).index,
            (*(*ost).ist).index,
            (*(*ost).file).index,
            (*ost).index,
        );
        if !(*ost).enc_ctx.is_null() {
            let in_codec = (*(*ost).ist).dec;
            let out_codec = (*(*ost).enc_ctx).codec;
            let mut decoder_name: *const libc::c_char = c_str!("?").as_ptr();
            let mut in_codec_name: *const libc::c_char = c_str!("?").as_ptr();
            let mut encoder_name: *const libc::c_char = c_str!("?").as_ptr();
            let mut out_codec_name: *const libc::c_char = c_str!("?").as_ptr();

            if !in_codec.is_null() {
                decoder_name = (*in_codec).name;
                let desc = avcodec_descriptor_get((*in_codec).id);
                if !desc.is_null() {
                    in_codec_name = (*desc).name;
                }
                if CStr::from_ptr(decoder_name) == CStr::from_ptr(in_codec_name) {
                    decoder_name = c_str!("native").as_ptr();
                }
            }

            if !out_codec.is_null() {
                encoder_name = (*out_codec).name;
                let desc = avcodec_descriptor_get((*out_codec).id);
                if !desc.is_null() {
                    out_codec_name = (*desc).name;
                }
                if CStr::from_ptr(encoder_name) == CStr::from_ptr(out_codec_name) {
                    encoder_name = c_str!("native").as_ptr();
                }
            }

            av_log(
                ptr::null_mut(),
                AV_LOG_INFO,
                c_str!(" (%s (%s) -> %s (%s))").as_ptr(),
                in_codec_name,
                decoder_name,
                out_codec_name,
                encoder_name,
            );
        } else {
            av_log(ptr::null_mut(), AV_LOG_INFO, c_str!(" (copy)").as_ptr());
        }
        av_log(ptr::null_mut(), AV_LOG_INFO, c_str!("\n").as_ptr());
        ost = ost_iter(ost);
    }
}

unsafe fn set_tty_echo(on: c_int) {
    let mut tty: libc::termios = std::mem::zeroed();
    if libc::tcgetattr(0, &mut tty) == 0 {
        if on != 0 {
            tty.c_lflag |= libc::ECHO;
        } else {
            tty.c_lflag &= !libc::ECHO;
        }
        libc::tcsetattr(0, libc::TCSANOW, &tty);
    }
}

unsafe fn check_keyboard_interaction(cur_time: i64) -> c_int {
    static mut LAST_TIME: i64 = 0; // Initialize to avoid uninitialized read
    if RECEIVED_NB_SIGNALS.load(atomic::Ordering::SeqCst) != 0 {
        return AVERROR_EXIT;
    }

    let key = if cur_time - LAST_TIME >= 100000 {
        let k = read_key();
        LAST_TIME = cur_time;
        k
    } else {
        -1
    };

    if key == b'q' as c_int {
        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("\n\n[q] command received. Exiting.\n\n").as_ptr(),
        );
        return AVERROR_EXIT;
    }
    if key == b'+' as c_int {
        av_log_set_level(av_log_get_level() + 10);
    }
    if key == b'-' as c_int {
        av_log_set_level(av_log_get_level() - 10);
    }
    if key == b'c' as c_int || key == b'C' as c_int {
        let mut buf = [0; 4096];
        let mut target = [0; 64];
        let mut command = [0; 256];
        let mut arg = [0; 256];
        let mut time: f64 = 0.0;
        let mut n_parsed: c_int = 0;

        eprintln!("Enter command: <target>|all <time>|-1 <command>[ <argument>]");
        let mut i = 0;
        set_tty_echo(1);
        loop {
            let k = read_key();
            if k == b'\n' as c_int || k == b'\r' as c_int || i >= buf.len() - 1 {
                break;
            }
            if k > 0 {
                buf[i] = k as u8;
                i += 1;
            }
        }
        buf[i] = 0;
        set_tty_echo(0);
        eprintln!();

        let buf_cstr = CStr::from_bytes_until_nul(&buf).unwrap();
        // Manually parse similar to sscanf for safety and control
        let buf_str = buf_cstr.to_str().unwrap_or("");
        let parts: Vec<&str> = buf_str.splitn(4, ' ').collect();

        if parts.len() >= 3 {
            n_parsed = parts.len() as c_int;
            let target_str = parts[0];
            let time_str = parts[1];
            let command_str = parts[2];
            let arg_str = if parts.len() > 3 { parts[3] } else { "" };

            // Copy to fixed-size arrays
            ptr::copy_nonoverlapping(
                target_str.as_ptr() as *const u8,
                target.as_mut_ptr(),
                target_str.len().min(target.len() - 1),
            );
            target[target_str.len().min(target.len() - 1)] = 0;

            ptr::copy_nonoverlapping(
                command_str.as_ptr() as *const u8,
                command.as_mut_ptr(),
                command_str.len().min(command.len() - 1),
            );
            command[command_str.len().min(command.len() - 1)] = 0;

            ptr::copy_nonoverlapping(
                arg_str.as_ptr() as *const u8,
                arg.as_mut_ptr(),
                arg_str.len().min(arg.len() - 1),
            );
            arg[arg_str.len().min(arg.len() - 1)] = 0;

            if let Ok(t) = time_str.parse::<f64>() {
                time = t;
            } else {
                n_parsed = 0; // Indicate parsing failure if time is not a valid float
            }
        }


        if n_parsed >= 3 {
            av_log(
                ptr::null_mut(),
                AV_LOG_DEBUG,
                c_str!("Processing command target:%s time:%f command:%s arg:%s").as_ptr(),
                target.as_ptr(),
                time,
                command.as_ptr(),
                arg.as_ptr(),
            );
            let mut ost = ost_iter(ptr::null_mut());
            while !ost.is_null() {
                if !(*ost).fg_simple.is_null() {
                    fg_send_command(
                        (*ost).fg_simple,
                        time,
                        target.as_ptr() as *mut libc::c_char,
                        command.as_ptr() as *mut libc::c_char,
                        arg.as_ptr() as *mut libc::c_char,
                        (key == b'C' as c_int) as c_int,
                    );
                }
                ost = ost_iter(ost);
            }
            for i in 0..nb_filtergraphs {
                fg_send_command(
                    *filtergraphs.add(i as usize),
                    time,
                    target.as_ptr() as *mut libc::c_char,
                    command.as_ptr() as *mut libc::c_char,
                    arg.as_ptr() as *mut libc::c_char,
                    (key == b'C' as c_int) as c_int,
                );
            }
        } else {
            av_log(
                ptr::null_mut(),
                AV_LOG_ERROR,
                c_str!(
                    "Parse error, at least 3 arguments were expected, only %d given in string '%s'\n"
                )
                .as_ptr(),
                n_parsed,
                buf.as_ptr(),
            );
        }
    }
    if key == b'?' as c_int {
        eprint!(
            "key    function\n\
             ?      show this help\n\
             +      increase verbosity\n\
             -      decrease verbosity\n\
             c      Send command to first matching filter supporting it\n\
             C      Send/Queue command to all matching filters\n\
             h      dump packets/hex press to cycle through the 3 states\n\
             q      quit\n\
             s      Show QP histogram\n"
        );
    }
    0
}

unsafe fn transcode(sch: *mut Scheduler) -> c_int {
    let mut ret = 0;
    print_stream_maps();

    TRANSCODE_INIT_DONE.store(1, atomic::Ordering::SeqCst);

    ret = sch_start(sch);
    if ret < 0 {
        return ret;
    }

    if stdin_interaction != 0 {
        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("Press [q] to stop, [?] for help\n").as_ptr(),
        );
    }

    let timer_start = av_gettime_relative();
    let mut transcode_ts: i64 = 0;

    loop {
        let wait_ret = sch_wait(sch, stats_period.try_into().unwrap(), &mut transcode_ts);
        if wait_ret != 0 {
            break;
        }

        let cur_time = av_gettime_relative();
        if stdin_interaction != 0 {
            if check_keyboard_interaction(cur_time) < 0 {
                break;
            }
        }
        print_report(0, timer_start, cur_time, transcode_ts);
    }

    ret = sch_stop(sch, &mut transcode_ts);

    for i in 0..nb_output_files {
        let err = of_write_trailer(*output_files.add(i as usize));
        ret = err_merge(ret, err);
    }

    term_exit();

    print_report(1, timer_start, av_gettime_relative(), transcode_ts);

    ret
}

unsafe fn get_benchmark_time_stamps() -> BenchmarkTimeStamps {
    let mut time_stamps = BenchmarkTimeStamps {
        real_usec: av_gettime_relative(),
        user_usec: 0,
        sys_usec: 0,
    };

    let mut rusage: libc::rusage = std::mem::zeroed();
    libc::getrusage(libc::RUSAGE_SELF, &mut rusage);
    time_stamps.user_usec =
        (rusage.ru_utime.tv_sec * 1_000_000_000i64) / 1000 + rusage.ru_utime.tv_usec;
    time_stamps.sys_usec =
        (rusage.ru_stime.tv_sec * 1_000_000_000i64) / 1000 + rusage.ru_stime.tv_usec;
    
    time_stamps
}

unsafe fn getmaxrss() -> i64 {
    let mut rusage: libc::rusage = std::mem::zeroed();
    libc::getrusage(libc::RUSAGE_SELF, &mut rusage);
    // Assuming ru_maxrss is in kilobytes on Linux/macOS
    return rusage.ru_maxrss * 1024;
}

/// Safe Rust implementation of av_err2str.
pub fn av_err2str(errnum: i32) -> String {
    use std::ffi::CStr;
    use bindings::avutil::av_strerror;
    let mut buf = [0u8; 4096];
    unsafe {
        let ret = av_strerror(errnum, buf.as_mut_ptr() as *mut i8, buf.len());
        if ret == 0 {
            // Find the first null byte to avoid overrun
            let cstr = CStr::from_ptr(buf.as_ptr() as *const i8);
            cstr.to_string_lossy().into_owned()
        } else {
            format!("Unknown error code: {}", errnum)
        }
    }
}

// Original ffmpeg.c main() function
unsafe extern "C" fn ffmpeg_main(argc: c_int, argv: *mut *mut libc::c_char) -> c_int {
    let mut sch: *mut Scheduler = ptr::null_mut();
    let mut ret: c_int;

    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    parse_loglevel(argc, argv, ptr::null_mut()); // Assuming 'options' global is handled by parse_loglevel internally or not needed

    // CONFIG_AVDEVICE is assumed to be true since we include avdevice.rs
    avdevice_register_all();

    avformat_network_init();

    show_banner(argc, argv, ptr::null_mut()); // Assuming 'options' global is handled by show_banner internally or not needed

    // This macro simulates a C 'goto finish;' for error handling.
    // It cleans up and returns.
    macro_rules! goto_finish {
        () => {{
            if ret == AVERROR_EXIT {
                ret = 0;
            }
            ffmpeg_cleanup(ret);
            sch_free(&mut sch);
            return ret;
        }};
    }

    sch = sch_alloc();
    if sch.is_null() {
        ret = AVERROR(libc::ENOMEM);
        goto_finish!();
    }

    ret = ffmpeg_parse_options(argc, argv, sch);
    if ret < 0 {
        goto_finish!();
    }

    if nb_output_files <= 0 && nb_input_files == 0 {
        show_usage();
        av_log(
            ptr::null_mut(),
            AV_LOG_WARNING,
            c_str!("Use -h to get full help or, even better, run 'man %s'\n").as_ptr(),
            program_name,
        );
        ret = 1;
        goto_finish!();
    }

    if nb_output_files <= 0 {
        av_log(
            ptr::null_mut(),
            AV_LOG_FATAL,
            c_str!("At least one output file must be specified\n").as_ptr(),
        );
        ret = 1;
        goto_finish!();
    }

    let ti = get_benchmark_time_stamps();
    let _ = CURRENT_TIME.set(ti); // Initialize CURRENT_TIME OnceLock
    ret = transcode(sch);
    if ret >= 0 && do_benchmark != 0 {
        let current_time_val = get_benchmark_time_stamps();
        let utime = current_time_val.user_usec - ti.user_usec;
        let stime = current_time_val.sys_usec - ti.sys_usec;
        let rtime = current_time_val.real_usec - ti.real_usec;
        av_log(
            ptr::null_mut(),
            AV_LOG_INFO,
            c_str!("bench: utime=%0.3fs stime=%0.3fs rtime=%0.3fs\n").as_ptr(),
            utime as f64 / 1_000_000.0,
            stime as f64 / 1_000_000.0,
            rtime as f64 / 1_000_000.0,
        );
    }

    ret = if RECEIVED_NB_SIGNALS.load(atomic::Ordering::SeqCst) != 0 {
        255
    } else if ret == FFMPEG_ERROR_RATE_EXCEEDED {
        69
    } else {
        ret
    };

    goto_finish!(); // This will only be reached if an error happened before the main loop or if
                    // the loop completes and needs final cleanup.
}

fn main() {
    use std::env;
    use std::ffi::CString;
    use std::os::raw::c_char;

    let args: Vec<String> = env::args().collect();
    let c_args: Vec<CString> = args.iter().map(|arg| CString::new(arg.as_str()).unwrap()).collect();
    let mut c_ptrs: Vec<*mut c_char> = c_args.iter().map(|arg| arg.as_ptr() as *mut c_char).collect();
    c_ptrs.push(std::ptr::null_mut()); // argv must be null-terminated for C
    let argc = c_args.len() as c_int;
    let argv = c_ptrs.as_mut_ptr();
    unsafe {
        ffmpeg_main(argc, argv);
    }
}
