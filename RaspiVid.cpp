
/*
Copyright (c) 2018, Raspberry Pi (Trading) Ltd.
Copyright (c) 2013, Broadcom Europe Ltd.
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiVid.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * Description
 *
 * 3 components are created; camera, preview and video encoder.
 * Camera component has three ports, preview, video and stills.
 * This program connects preview and video to the preview and video
 * encoder. Using mmal we don't need to worry about buffers between these
 * components, but we do need to handle buffers from the encoder, which
 * are simply written straight to the file in the requisite buffer callback.
 *
 * If raw option is selected, a video splitter component is connected between
 * camera and preview. This allows us to set up callback for raw camera data
 * (in YUV420 or RGB format) which might be useful for further image processing.
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 * We use the RaspiPreview code to handle the (generic) preview window
 */



/*

raspivid -v -hf -vf -md 2 -br 50 -awb off -awbg 1.2,1.0 -ex auto -fps 15 -a 12 -a 1024 -ae 32,0xfff,0x808000 --qp 25 -t 0 -mo -o /dev/null

raspivid -v -hf -vf -md 2 -br 50 -awb off -awbg 1.2,1.0 -ex auto -fps 15 -a 12 -a 1024 -ae 32,0xfff,0x808000 --qp 25 -t 0 -m4v -mo -o vid.mp4

*/


// We use some GNU extensions (basename)
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define MINIMP4_IMPLEMENTATION

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>


#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiHelpers.h"
#include "RaspiGPS.h"
#include "DispManX.h"


#include "MP4Wrapper.h"
#include "CNN.h"
//#include "AVCodecHelper.h"

#define VIDEO_FPS 15

#include <semaphore.h>

#include <stdbool.h>

#include <iostream>
#include <vector>


#include <condition_variable>
#include <thread>
#include <mutex>
//#include <wiringPi.h>
//#include <GLES2/gl2ext.h>
//#include <GLES2/gl2.h>

#ifndef OPEN_CV2
#include <opencv2/core/core_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#else
#include <opencv4/core/core_c.h>
#include <opencv4/imgcodecs.hpp>
#include <opencv4/objdetect/objdetect.hpp>
#include <opencv4/imgproc/imgproc.hpp>
#endif


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

#define RESIZER_OUTPUT_PORT 0

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms


/// Capture/Pause switch method
/// Simply capture for time specified
enum
{
   WAIT_METHOD_NONE,       /// Simply capture for time specified
   WAIT_METHOD_TIMED,      /// Cycle between capture and pause for times specified
   WAIT_METHOD_KEYPRESS,   /// Switch between capture and pause on keypress
   WAIT_METHOD_SIGNAL,     /// Switch between capture and pause on signal
   WAIT_METHOD_FOREVER     /// Run/record forever
};

// Forward
typedef struct RASPIVID_STATE_S RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   char *cb_buff;                       /// Circular buffer
   int   cb_len;                        /// Length of buffer
   int   cb_wptr;                       /// Current write pointer
   int   cb_wrap;                       /// Has buffer wrapped at least once?
   int   cb_data;                       /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60*1000)
   int   iframe_buff[IFRAME_BUFSIZE];          /// buffer of iframe pointers
   int   iframe_buff_wpos;
   int   iframe_buff_rpos;
   char  header_bytes[29];
   int  header_wptr;
   FILE *imv_file_handle;               /// File handle to write inline motion vectors to.
   FILE *raw_file_handle;               /// File handle to write raw data to.
   int  flush_buffers;
   FILE *pts_file_handle;         

	FILE* fp;
	int frameno;
	
	

} PORT_USERDATA;

/** Possible raw output formats
 */
typedef enum
{
   RAW_OUTPUT_FMT_YUV = 0,
   RAW_OUTPUT_FMT_RGB,
   RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;




/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE_S
{
   RASPICOMMONSETTINGS_PARAMETERS common_settings;     /// Common settings
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   MMAL_FOURCC_T encoding;             /// Requested codec video encoding (MJPEG or H264)
   int bitrate;                        /// Requested bitrate
   int framerate;                      /// Requested frame rate (fps)
   int intraperiod;                    /// Intra-refresh period (key frame rate)
   int quantisationParameter;          /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
   int bInlineHeaders;                  /// Insert inline headers to stream (SPS, PPS)
   int demoMode;                       /// Run app in demo mode
   int demoInterval;                   /// Interval between camera settings changes
   int immutableInput;                 /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
   /// the camera output or the encoder output (with compression artifacts)
   int profile;                        /// H264 profile to use for encoding
   int level;                          /// H264 level to use for encoding
   int waitMethod;                     /// Method for switching between pause and capture

   int onTime;                         /// In timed cycle mode, the amount of time the capture is on per cycle
   int offTime;                        /// In timed cycle mode, the amount of time the capture is off per cycle

   int segmentSize;                    /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
   int segmentWrap;                    /// Point at which to wrap segment counter
   int segmentNumber;                  /// Current segment counter
   int splitNow;                       /// Split at next possible i-frame if set to 1.
   int splitWait;                      /// Switch if user wants splited files

   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *splitter_component;  /// Pointer to the splitter component
   MMAL_COMPONENT_T *resizer_component;
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera or splitter to preview
   MMAL_CONNECTION_T *splitter_connection;/// Pointer to the connection from camera to splitter
   MMAL_CONNECTION_T *resizer_connection;
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *resizer_pool;	
   MMAL_POOL_T *splitter_pool; /// Pointer to the pool of buffers used by splitter output port 0
   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

   PORT_USERDATA callback_data;        /// Used to move data to the encoder callback

   int bCapturing;                     /// State of capture/pause
   int bCircularBuffer;                /// Whether we are writing to a circular buffer

   int inlineMotionVectors;             /// Encoder outputs inline Motion Vectors
   char *imv_filename;                  /// filename of inline Motion Vectors output
   int raw_output;                      /// Output raw video from camera as well
   RAW_OUTPUT_FMT raw_output_fmt;       /// The raw video format
   char *raw_filename;                  /// Filename for raw video output
   int intra_refresh_type;              /// What intra refresh type to use. -1 to not set.
   int frame;
   char *pts_filename;
   int save_pts;
   int64_t starttime;
   int64_t lasttime;

   bool netListen;
   MMAL_BOOL_T addSPSTiming;
   int slices;

// open cv related vars
   
   int running;
   int mp4_video;
   int mp4_logging;
   int use_resizer;
   int detect_motion;
   int file_open_delegated;
   char file_name[512];
   char* audio_card;		
   int opencv_width;
   int opencv_height;
   int skip_frame;
   int motion_thres;
   time_t opencv_next_update;
   VCOS_SEMAPHORE_T frame_complete;
   cv::CascadeClassifier cascade;
   cv::Mat opencv_image;
   cv::Mat opencv_image2;
   cv::Mat opencv_image3;	
   cv::Mat diff;
   cv::Mat kernal;
   rp::MP4Wrapper* mp4w;	
   rp::DispManX* disp;
   rp::CNN* cnn;
   
   std::condition_variable con;
   std::mutex mutex;		
	
	//rp::AVCodecHelper* avc;
   //CvMemStorage* storage;
   //IplImage* opencv_image;
   //IplImage* opencv_image2;
};


/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T  profile_map[] =
{
   {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
   {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
   {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T  level_map[] =
{
   {"4",           MMAL_VIDEO_LEVEL_H264_4},
   {"4.1",         MMAL_VIDEO_LEVEL_H264_41},
   {"4.2",         MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);

static XREF_T  initial_map[] =
{
   {"record",     0},
   {"pause",      1},
};

static int initial_map_size = sizeof(initial_map) / sizeof(initial_map[0]);

static XREF_T  intra_refresh_map[] =
{
   {"cyclic",       MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
   {"adaptive",     MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
   {"both",         MMAL_VIDEO_INTRA_REFRESH_BOTH},
   {"cyclicrows",   MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
//   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

static XREF_T  raw_output_fmt_map[] =
{
   {"yuv",  RAW_OUTPUT_FMT_YUV},
   {"rgb",  RAW_OUTPUT_FMT_RGB},
   {"gray", RAW_OUTPUT_FMT_GRAY},
};

static int raw_output_fmt_map_size = sizeof(raw_output_fmt_map) / sizeof(raw_output_fmt_map[0]);

/// Command ID's and Structure defining our command line options
enum
{
   CommandBitrate,
   CommandTimeout,
   CommandDemoMode,
   CommandFramerate,
   CommandPreviewEnc,
   CommandIntraPeriod,
   CommandProfile,
   CommandTimed,
   CommandSignal,
   CommandKeypress,
   CommandInitialState,
   CommandQP,
   CommandInlineHeaders,
   CommandSegmentFile,
   CommandSegmentWrap,
   CommandSegmentStart,
   CommandSplitWait,
   CommandCircular,
   CommandIMV,
   CommandIntraRefreshType,
   CommandFlush,
   CommandSavePTS,
   CommandCodec,
   CommandLevel,
   CommandRaw,
   CommandRawFormat,
   CommandNetListen,
   CommandSPSTimings,
   CommandSlices,

	CommandMP4Video,
	CommandMP4Audio,
	CommandMP4Logging,
	CommandMotion	
};


static COMMAND_LIST cmdline_commands[] =
{
   { CommandBitrate,       "-bitrate",    "b",  "Set bitrate. Use bits per second (e.g. 10MBits/s would be -b 10000000)", 1 },
   { CommandTimeout,       "-timeout",    "t",  "Time (in ms) to capture for. If not specified, set to 5s. Zero to disable", 1 },
   { CommandDemoMode,      "-demo",       "d",  "Run a demo mode (cycle through range of camera options, no capture)", 1},
   { CommandFramerate,     "-framerate",  "fps","Specify the frames per second to record", 1},
   { CommandPreviewEnc,    "-penc",       "e",  "Display preview image *after* encoding (shows compression artifacts)", 0},
   { CommandIntraPeriod,   "-intra",      "g",  "Specify the intra refresh period (key frame rate/GoP size). Zero to produce an initial I-frame and then just P-frames.", 1},
   { CommandProfile,       "-profile",    "pf", "Specify H264 profile to use for encoding", 1},
   { CommandTimed,         "-timed",      "td", "Cycle between capture and pause. -cycle on,off where on is record time and off is pause time in ms", 0},
   { CommandSignal,        "-signal",     "s",  "Cycle between capture and pause on Signal", 0},
   { CommandKeypress,      "-keypress",   "k",  "Cycle between capture and pause on ENTER", 0},
   { CommandInitialState,  "-initial",    "i",  "Initial state. Use 'record' or 'pause'. Default 'record'", 1},
   { CommandQP,            "-qp",         "qp", "Quantisation parameter. Use approximately 10-40. Default 0 (off)", 1},
   { CommandInlineHeaders, "-inline",     "ih", "Insert inline headers (SPS, PPS) to stream", 0},
   { CommandSegmentFile,   "-segment",    "sg", "Segment output file in to multiple files at specified interval <ms>", 1},
   { CommandSegmentWrap,   "-wrap",       "wr", "In segment mode, wrap any numbered filename back to 1 when reach number", 1},
   { CommandSegmentStart,  "-start",      "sn", "In segment mode, start with specified segment number", 1},
   { CommandSplitWait,     "-split",      "sp", "In wait mode, create new output file for each start event", 0},
   { CommandCircular,      "-circular",   "c",  "Run encoded data through circular buffer until triggered then save", 0},
   { CommandIMV,           "-vectors",    "x",  "Output filename <filename> for inline motion vectors", 1 },
   { CommandIntraRefreshType,"-irefresh", "if", "Set intra refresh type", 1},
   { CommandFlush,         "-flush",      "fl",  "Flush buffers in order to decrease latency", 0 },
   { CommandSavePTS,       "-save-pts",   "pts","Save Timestamps to file for mkvmerge", 1 },
   { CommandCodec,         "-codec",      "cd", "Specify the codec to use - H264 (default) or MJPEG", 1 },
   { CommandLevel,         "-level",      "lev","Specify H264 level to use for encoding", 1},
   { CommandRaw,           "-raw",        "r",  "Output filename <filename> for raw video", 1 },
   { CommandRawFormat,     "-raw-format", "rf", "Specify output format for raw video. Default is yuv", 1},
   { CommandNetListen,     "-listen",     "l", "Listen on a TCP socket", 0},
   { CommandSPSTimings,    "-spstimings",    "stm", "Add in h.264 sps timings", 0},
   { CommandSlices   ,     "-slices",     "sl", "Horizontal slices per frame. Default 1 (off)", 1},

	{CommandMP4Video,       "-mp4video",	"m4v", "Write mp4 video instead of raw h264", 0},
	{CommandMP4Audio,       "-mp4audio",	"m4a", "Capture audio from mic", 1},
	{CommandMP4Logging,	"-mp4logging",	"m4l", "Enable ffmpeg logging", 0},
	{CommandMotion,		"-motion",	"mo", "Motion detection(recording) enabled", 0},	
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);


static struct
{
   char *description;
   int nextWaitMethod;
} wait_method_description[] =
{
   {"Simple capture",         WAIT_METHOD_NONE},
   {"Capture forever",        WAIT_METHOD_FOREVER},
   {"Cycle on time",          WAIT_METHOD_TIMED},
   {"Cycle on keypress",      WAIT_METHOD_KEYPRESS},
   {"Cycle on signal",        WAIT_METHOD_SIGNAL},
};

static int frame_diff(cv::Mat& image, cv::Mat& old, RASPIVID_STATE* state);

static void FillRect( VC_IMAGE_TYPE_T type, void *image, int pitch, int aligned_height, int x, int y, int w, int h, int val )
{
    int         row;
    int         col;

    uint16_t *line = (uint16_t *)image + y * (pitch>>1) + x;

    for ( row = 0; row < h; row++ )
    {
        for ( col = 0; col < w; col++ )
        {
            line[col] = val;
        }
        line += (pitch>>1);
    }
}


static int wait_method_description_size = sizeof(wait_method_description) / sizeof(wait_method_description[0]);



/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPIVID_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));

   raspicommonsettings_set_defaults(&state->common_settings);

   // Now set anything non-zero
   state->timeout = -1; // replaced with 5000ms later if unset
   state->common_settings.width = 1920;       // Default to 1080p
   state->common_settings.height = 1080;
   state->encoding = MMAL_ENCODING_H264;
   state->bitrate = 17000000; // This is a decent default bitrate for 1080p
   state->framerate = VIDEO_FRAME_RATE_NUM;
   state->intraperiod = -1;    // Not set
   state->quantisationParameter = 0;
   state->demoMode = 0;
   state->demoInterval = 250; // ms
   state->immutableInput = 1;
   state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   state->level = MMAL_VIDEO_LEVEL_H264_4;
   state->waitMethod = WAIT_METHOD_NONE;
   state->onTime = 5000;
   state->offTime = 5000;
   state->bCapturing = 0;
   state->bInlineHeaders = 0;
   state->segmentSize = 0;  // 0 = not segmenting the file.
   state->segmentNumber = 1;
   state->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
   state->splitNow = 0;
   state->splitWait = 0;
   state->inlineMotionVectors = 0;
   state->intra_refresh_type = -1;
   state->frame = 0;
   state->save_pts = 0;
   state->netListen = false;
   state->addSPSTiming = MMAL_FALSE;
   state->slices = 1;


   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

static void check_camera_model(int cam_num)
{
   MMAL_COMPONENT_T *camera_info;
   MMAL_STATUS_T status;

   // Try to get the camera name
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
   if (status == MMAL_SUCCESS)
   {
      MMAL_PARAMETER_CAMERA_INFO_T param;
      param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
      param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware version
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);

      if (status != MMAL_SUCCESS)
      {
         // Running on newer firmware
         param.hdr.size = sizeof(param);
         status = mmal_port_parameter_get(camera_info->control, &param.hdr);
         if (status == MMAL_SUCCESS && param.num_cameras > cam_num)
         {
            if (!strncmp(param.cameras[cam_num].camera_name, "toshh2c", 7))
            {
               fprintf(stderr, "The driver for the TC358743 HDMI to CSI2 chip you are using is NOT supported.\n");
               fprintf(stderr, "They were written for a demo purposes only, and are in the firmware on an as-is\n");
               fprintf(stderr, "basis and therefore requests for support or changes will not be acted on.\n\n");
            }
         }
      }

      mmal_component_destroy(camera_info);
   }
}

/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPIVID_STATE *state)
{
   int i;

   if (!state)
   {
      vcos_assert(0);
      return;
   }

   raspicommonsettings_dump_parameters(&state->common_settings);

   fprintf(stderr, "bitrate %d, framerate %d, time delay %d\n", state->bitrate, state->framerate, state->timeout);
   fprintf(stderr, "H264 Profile %s\n", raspicli_unmap_xref(state->profile, profile_map, profile_map_size));
   fprintf(stderr, "H264 Level %s\n", raspicli_unmap_xref(state->level, level_map, level_map_size));
   fprintf(stderr, "H264 Quantisation level %d, Inline headers %s\n", state->quantisationParameter, state->bInlineHeaders ? "Yes" : "No");
   fprintf(stderr, "H264 Fill SPS Timings %s\n", state->addSPSTiming ? "Yes" : "No");
   fprintf(stderr, "H264 Intra refresh type %s, period %d\n", raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size), state->intraperiod);
   fprintf(stderr, "H264 Slices %d\n", state->slices);

   // Not going to display segment data unless asked for it.
   if (state->segmentSize)
      fprintf(stderr, "Segment size %d, segment wrap value %d, initial segment number %d\n", state->segmentSize, state->segmentWrap, state->segmentNumber);

   if (state->raw_output)
      fprintf(stderr, "Raw output enabled, format %s\n", raspicli_unmap_xref(state->raw_output_fmt, raw_output_fmt_map, raw_output_fmt_map_size));

   fprintf(stderr, "Wait method : ");
   for (i=0; i<wait_method_description_size; i++)
   {
      if (state->waitMethod == wait_method_description[i].nextWaitMethod)
         fprintf(stderr, "%s", wait_method_description[i].description);
   }
   fprintf(stderr, "\nInitial state '%s'\n", raspicli_unmap_xref(state->bCapturing, initial_map, initial_map_size));
   fprintf(stderr, "\n\n");

   raspipreview_dump_parameters(&state->preview_parameters);
   raspicamcontrol_dump_parameters(&state->camera_parameters);
}

/**
 * Display usage information for the application to stdout
 *
 * @param app_name String to display as the application name
 */
static void application_help_message(char *app_name)
{
   int i;

   fprintf(stdout, "Display camera output to display, and optionally saves an H264 capture at requested bitrate\n\n");
   fprintf(stdout, "\nusage: %s [options]\n\n", app_name);

   fprintf(stdout, "Image parameter commands\n\n");

   raspicli_display_help(cmdline_commands, cmdline_commands_size);

   // Profile options
   fprintf(stdout, "\n\nH264 Profile options :\n%s", profile_map[0].mode );

   for (i=1; i<profile_map_size; i++)
   {
      fprintf(stdout, ",%s", profile_map[i].mode);
   }

   // Level options
   fprintf(stdout, "\n\nH264 Level options :\n%s", level_map[0].mode );

   for (i=1; i<level_map_size; i++)
   {
      fprintf(stdout, ",%s", level_map[i].mode);
   }

   // Intra refresh options
   fprintf(stdout, "\n\nH264 Intra refresh options :\n%s", intra_refresh_map[0].mode );

   for (i=1; i<intra_refresh_map_size; i++)
   {
      fprintf(stdout, ",%s", intra_refresh_map[i].mode);
   }

   // Raw output format options
   fprintf(stdout, "\n\nRaw output format options :\n%s", raw_output_fmt_map[0].mode );

   for (i=1; i<raw_output_fmt_map_size; i++)
   {
      fprintf(stdout, ",%s", raw_output_fmt_map[i].mode);
   }

   fprintf(stdout, "\n\n");

   fprintf(stdout, "Raspivid allows output to a remote IPv4 host e.g. -o tcp://192.168.1.2:1234"
           "or -o udp://192.168.1.2:1234\n"
           "To listen on a TCP port (IPv4) and wait for an incoming connection use the -l option\n"
           "e.g. raspivid -l -o tcp://0.0.0.0:3333 -> bind to all network interfaces,\n"
           "raspivid -l -o tcp://192.168.1.1:3333 -> bind to a certain local IPv4 port\n");

   return;
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters to
 * @return Non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, const char **argv, RASPIVID_STATE *state)
{
   // Parse the command line arguments.
   // We are looking for --<something> or -<abbreviation of something>


   int valid = 1;
   int i;

   for (i = 1; i < argc && valid; i++)
   {
      int command_id, num_parameters;

      if (!argv[i])
         continue;

      if (argv[i][0] != '-')
      {
         valid = 0;
         continue;
      }

      // Assume parameter is valid until proven otherwise
      valid = 1;

      command_id = raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

	//printf("%s	%d\n", argv[i], command_id);

      // If we found a command but are missing a parameter, continue (and we will drop out of the loop)
      if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc) )
         continue;

      //  We are now dealing with a command line option
      switch (command_id)
      {
	case CommandMP4Video:
		state->mp4_video = 1;
		break;
	case CommandMP4Audio:
		if(argv[i+1][0] != '-') {
			state->audio_card = (char*)argv[i + 1];
			i++;
		} else {
			valid = 0;
		}
		break;
	case CommandMP4Logging:
		state->mp4_logging = 1;
		break;
	case CommandMotion:
		state->raw_output_fmt = RAW_OUTPUT_FMT_GRAY;
		state->raw_filename = (char*)malloc(4);
		strcpy(state->raw_filename, "raw");		
		state->detect_motion = 1;
		state->raw_output = 1;

		if( argc > i+1 && argv[i+1][0] != '-' && sscanf(argv[i + 1], "%d", &state->use_resizer) == 1) {
			i++;
		} else {
			valid = 0;
		}

		break;
	
      case CommandBitrate: // 1-100
         if (sscanf(argv[i + 1], "%u", &state->bitrate) == 1)
         {
            i++;
         }
         else
            valid = 0;

         break;

      case CommandTimeout: // Time to run viewfinder/capture
      {
         if (sscanf(argv[i + 1], "%d", &state->timeout) == 1)
         {
            // Ensure that if previously selected a waitMethod we don't overwrite it
            if (state->timeout == 0 && state->waitMethod == WAIT_METHOD_NONE)
               state->waitMethod = WAIT_METHOD_FOREVER;

            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandDemoMode: // Run in demo mode - no capture
      {
         // Demo mode might have a timing parameter
         // so check if a) we have another parameter, b) its not the start of the next option
         if (i + 1 < argc  && argv[i+1][0] != '-')
         {
            if (sscanf(argv[i + 1], "%u", &state->demoInterval) == 1)
            {
               // TODO : What limits do we need for timeout?
               if (state->demoInterval == 0)
                  state->demoInterval = 250; // ms

               state->demoMode = 1;
               i++;
            }
            else
               valid = 0;
         }
         else
         {
            state->demoMode = 1;
         }

         break;
      }

      case CommandFramerate: // fps to record
      {
         if (sscanf(argv[i + 1], "%u", &state->framerate) == 1)
         {
            // TODO : What limits do we need for fps 1 - 30 - 120??
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandPreviewEnc:
         state->immutableInput = 0;
         break;

      case CommandIntraPeriod: // key frame rate
      {
         if (sscanf(argv[i + 1], "%u", &state->intraperiod) == 1)
            i++;
         else
            valid = 0;
         break;
      }

      case CommandQP: // quantisation parameter
      {
         if (sscanf(argv[i + 1], "%u", &state->quantisationParameter) == 1)
            i++;
         else
            valid = 0;
         break;
      }

      case CommandProfile: // H264 profile
      {
         state->profile = raspicli_map_xref(argv[i + 1], profile_map, profile_map_size);

         if( state->profile == -1)
            state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;

         i++;
         break;
      }

      case CommandInlineHeaders: // H264 inline headers
      {
         state->bInlineHeaders = 1;
         break;
      }

      case CommandTimed:
      {
         if (sscanf(argv[i + 1], "%u,%u", &state->onTime, &state->offTime) == 2)
         {
            i++;

            if (state->onTime < 1000)
               state->onTime = 1000;

            if (state->offTime < 1000)
               state->offTime = 1000;

            state->waitMethod = WAIT_METHOD_TIMED;

            if (state->timeout == -1)
               state->timeout = 0;
         }
         else
            valid = 0;
         break;
      }

      case CommandKeypress:
         state->waitMethod = WAIT_METHOD_KEYPRESS;

         if (state->timeout == -1)
            state->timeout = 0;

         break;

      case CommandSignal:
         state->waitMethod = WAIT_METHOD_SIGNAL;
         // Reenable the signal
         signal(SIGUSR1, default_signal_handler);

         if (state->timeout == -1)
            state->timeout = 0;

         break;

      case CommandInitialState:
      {
         state->bCapturing = raspicli_map_xref(argv[i + 1], initial_map, initial_map_size);

         if( state->bCapturing == -1)
            state->bCapturing = 0;

         i++;
         break;
      }

      case CommandSegmentFile: // Segment file in to chunks of specified time
      {
         if (sscanf(argv[i + 1], "%u", &state->segmentSize) == 1)
         {
            // Must enable inline headers for this to work
            state->bInlineHeaders = 1;
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandSegmentWrap: // segment wrap value
      {
         if (sscanf(argv[i + 1], "%u", &state->segmentWrap) == 1)
            i++;
         else
            valid = 0;
         break;
      }

      case CommandSegmentStart: // initial segment number
      {
         if((sscanf(argv[i + 1], "%u", &state->segmentNumber) == 1) && (!state->segmentWrap || (state->segmentNumber <= state->segmentWrap)))
            i++;
         else
            valid = 0;
         break;
      }

      case CommandSplitWait: // split files on restart
      {
         // Must enable inline headers for this to work
         state->bInlineHeaders = 1;
         state->splitWait = 1;
         break;
      }

      case CommandCircular:
      {
         state->bCircularBuffer = 1;
         break;
      }

      case CommandIMV:  // output filename
      {
         state->inlineMotionVectors = 1;
         int len = strlen(argv[i + 1]);
         if (len)
         {
            state->imv_filename = (char*)malloc(len + 1);
            vcos_assert(state->imv_filename);
            if (state->imv_filename)
               strncpy(state->imv_filename, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandIntraRefreshType:
      {
         state->intra_refresh_type = raspicli_map_xref(argv[i + 1], intra_refresh_map, intra_refresh_map_size);
         i++;
         break;
      }

      case CommandFlush:
      {
         state->callback_data.flush_buffers = 1;
         break;
      }
      case CommandSavePTS:  // output filename
      {
         state->save_pts = 1;
         int len = strlen(argv[i + 1]);
         if (len)
         {
            state->pts_filename = (char*)malloc(len + 1);
            vcos_assert(state->pts_filename);
            if (state->pts_filename)
               strncpy(state->pts_filename, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;
      }
      case CommandCodec:  // codec type
      {
         int len = strlen(argv[i + 1]);
         if (len)
         {
            if (len==4 && !strncmp("H264", argv[i+1], 4))
               state->encoding = MMAL_ENCODING_H264;
            else  if (len==5 && !strncmp("MJPEG", argv[i+1], 5))
               state->encoding = MMAL_ENCODING_MJPEG;
            else
               valid = 0;
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandLevel: // H264 level
      {
         state->level = raspicli_map_xref(argv[i + 1], level_map, level_map_size);

         if( state->level == -1)
            state->level = MMAL_VIDEO_LEVEL_H264_4;

         i++;
         break;
      }

      case CommandRaw:  // output filename
      {
         state->raw_output = 1;
         //state->raw_output_fmt defaults to 0 / yuv
         int len = strlen(argv[i + 1]);
         if (len)
         {
            state->raw_filename = (char*)malloc(len + 1);
            vcos_assert(state->raw_filename);
            if (state->raw_filename)
               strncpy(state->raw_filename, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandRawFormat:
      {
         state->raw_output_fmt = (RAW_OUTPUT_FMT)raspicli_map_xref(argv[i + 1], raw_output_fmt_map, raw_output_fmt_map_size);
	
         if (state->raw_output_fmt == -1)
            valid = 0;

         i++;
         break;
      }

      case CommandNetListen:
      {
         state->netListen = true;

         break;
      }
      case CommandSlices:
      {
         if ((sscanf(argv[i + 1], "%d", &state->slices) == 1) && (state->slices > 0))
            i++;
         else
            valid = 0;
         break;
      }

      case CommandSPSTimings:
      {
         state->addSPSTiming = MMAL_TRUE;

         break;
      }

      default:
      {
         // Try parsing for any image specific parameters
         // result indicates how many parameters were used up, 0,1,2
         // but we adjust by -1 as we have used one already
         const char *second_arg = (i + 1 < argc) ? argv[i + 1] : NULL;
         int parms_used = (raspicamcontrol_parse_cmdline(&state->camera_parameters, &argv[i][1], second_arg));

         // Still unused, try common settings
         if (!parms_used)
            parms_used = raspicommonsettings_parse_cmdline(&state->common_settings, &argv[i][1], second_arg, (void(*)())&application_help_message);

         // Still unused, try preview options
         if (!parms_used)
            parms_used = raspipreview_parse_cmdline(&state->preview_parameters, &argv[i][1], second_arg);

         // If no parms were used, this must be a bad parameter
         if (!parms_used)
            valid = 0;
         else
            i += parms_used - 1;

         break;
      }
      }
   }

   if (!valid)
   {
      fprintf(stderr, "Invalid command line option (%s)\n", argv[i-1]);
      return 1;
   }

   return 0;
}

/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
static FILE *open_filename(RASPIVID_STATE *pState, char *filename)
{
   FILE *new_handle = NULL;
   char *tempname = NULL;

   if (pState->segmentSize || pState->splitWait)
   {
      // Create a new filename string

      //If %d/%u or any valid combination e.g. %04d is specified, assume segment number.
      bool bSegmentNumber = false;
      const char* pPercent = strchr(filename, '%');
      if (pPercent)
      {
         pPercent++;
         while (isdigit(*pPercent))
            pPercent++;
         if (*pPercent == 'u' || *pPercent == 'd')
            bSegmentNumber = true;
      }

      if (bSegmentNumber)
      {
         asprintf(&tempname, filename, pState->segmentNumber);
      }
      else
      {
         char temp_ts_str[100];
         time_t t = time(NULL);
         struct tm *tm = localtime(&t);
         strftime(temp_ts_str, 100, filename, tm);
         asprintf(&tempname, "%s", temp_ts_str);
      }

      filename = tempname;
   }

   if (filename)
   {
      bool bNetwork = false;
      int sfd = -1, socktype;

      if(!strncmp("tcp://", filename, 6))
      {
         bNetwork = true;
         socktype = SOCK_STREAM;
      }
      else if(!strncmp("udp://", filename, 6))
      {
         if (pState->netListen)
         {
            fprintf(stderr, "No support for listening in UDP mode\n");
            exit(131);
         }
         bNetwork = true;
         socktype = SOCK_DGRAM;
      }

      if(bNetwork)
      {
         unsigned short port;
         filename += 6;
         char *colon;
         if(NULL == (colon = strchr(filename, ':')))
         {
            fprintf(stderr, "%s is not a valid IPv4:port, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234\n",
                    filename);
            exit(132);
         }
         if(1 != sscanf(colon + 1, "%hu", &port))
         {
            fprintf(stderr,
                    "Port parse failed. %s is not a valid network file name, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234\n",
                    filename);
            exit(133);
         }
         char chTmp = *colon;
         *colon = 0;

         struct sockaddr_in saddr= {};
         saddr.sin_family = AF_INET;
         saddr.sin_port = htons(port);
         if(0 == inet_aton(filename, &saddr.sin_addr))
         {
            fprintf(stderr, "inet_aton failed. %s is not a valid IPv4 address\n",
                    filename);
            exit(134);
         }
         *colon = chTmp;

         if (pState->netListen)
         {
            int sockListen = socket(AF_INET, SOCK_STREAM, 0);
            if (sockListen >= 0)
            {
               int iTmp = 1;
               setsockopt(sockListen, SOL_SOCKET, SO_REUSEADDR, &iTmp, sizeof(int));//no error handling, just go on
               if (bind(sockListen, (struct sockaddr *) &saddr, sizeof(saddr)) >= 0)
               {
                  while ((-1 == (iTmp = listen(sockListen, 0))) && (EINTR == errno))
                     ;
                  if (-1 != iTmp)
                  {
                     fprintf(stderr, "Waiting for a TCP connection on %s:%"SCNu16"...",
                             inet_ntoa(saddr.sin_addr), ntohs(saddr.sin_port));
                     struct sockaddr_in cli_addr;
                     socklen_t clilen = sizeof(cli_addr);
                     while ((-1 == (sfd = accept(sockListen, (struct sockaddr *) &cli_addr, &clilen))) && (EINTR == errno))
                        ;
                     if (sfd >= 0)
                        fprintf(stderr, "Client connected from %s:%"SCNu16"\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));
                     else
                        fprintf(stderr, "Error on accept: %s\n", strerror(errno));
                  }
                  else//if (-1 != iTmp)
                  {
                     fprintf(stderr, "Error trying to listen on a socket: %s\n", strerror(errno));
                  }
               }
               else//if (bind(sockListen, (struct sockaddr *) &saddr, sizeof(saddr)) >= 0)
               {
                  fprintf(stderr, "Error on binding socket: %s\n", strerror(errno));
               }
            }
            else//if (sockListen >= 0)
            {
               fprintf(stderr, "Error creating socket: %s\n", strerror(errno));
            }

            if (sockListen >= 0)//regardless success or error
               close(sockListen);//do not listen on a given port anymore
         }
         else//if (pState->netListen)
         {
            if(0 <= (sfd = socket(AF_INET, socktype, 0)))
            {
               fprintf(stderr, "Connecting to %s:%hu...", inet_ntoa(saddr.sin_addr), port);

               int iTmp = 1;
               while ((-1 == (iTmp = connect(sfd, (struct sockaddr *) &saddr, sizeof(struct sockaddr_in)))) && (EINTR == errno))
                  ;
               if (iTmp < 0)
                  fprintf(stderr, "error: %s\n", strerror(errno));
               else
                  fprintf(stderr, "connected, sending video...\n");
            }
            else
               fprintf(stderr, "Error creating socket: %s\n", strerror(errno));
         }

         if (sfd >= 0)
            new_handle = fdopen(sfd, "w");
      }
      else if(!pState->file_open_delegated)
      {
		new_handle = fopen(filename, "wb");
      } else {

		strcpy(pState->file_name, filename);
		pState->file_open_delegated = 0;
		new_handle = stdout;
	}
   }

	
   if (pState->common_settings.verbose)
   {
      if (new_handle)
         fprintf(stderr, "Opening output file \"%s\"\n", filename);
      else
         fprintf(stderr, "Failed to open new file \"%s\"\n", filename);
   }

   if (tempname)
      free(tempname);

   return new_handle;
}

/**
 * Update any annotation data specific to the video.
 * This simply passes on the setting from cli, or
 * if application defined annotate requested, updates
 * with the H264 parameters
 *
 * @param state Pointer to state control struct
 *
 */
static void update_annotation_data(RASPIVID_STATE *state)
{
   // So, if we have asked for a application supplied string, set it to the H264 or GPS parameters
   if (state->camera_parameters.enable_annotate & ANNOTATE_APP_TEXT)
   {
      char *text;

      if (state->common_settings.gps)
      {
         text = raspi_gps_location_string();
      }
      else
      {
         const char *refresh = raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size);

         asprintf(&text,  "%dk,%df,%s,%d,%s,%s",
                  state->bitrate / 1000,  state->framerate,
                  refresh ? refresh : "(none)",
                  state->intraperiod,
                  raspicli_unmap_xref(state->profile, profile_map, profile_map_size),
                  raspicli_unmap_xref(state->level, level_map, level_map_size));
      }


      raspicamcontrol_set_annotate(state->camera_component, state->camera_parameters.enable_annotate, state->framerate, text,
                                   state->camera_parameters.annotate_text_size,
                                   state->camera_parameters.annotate_text_colour,
                                   state->camera_parameters.annotate_bg_colour,
                                   state->camera_parameters.annotate_justify,
                                   state->camera_parameters.annotate_x,
                                   state->camera_parameters.annotate_y
                                  );

      free(text);
   }
   else
   {
	
      raspicamcontrol_set_annotate(state->camera_component, state->camera_parameters.enable_annotate, state->framerate, state->camera_parameters.annotate_string,
                                   state->camera_parameters.annotate_text_size,
                                   state->camera_parameters.annotate_text_colour,
                                   state->camera_parameters.annotate_bg_colour,
                                   state->camera_parameters.annotate_justify,
                                   state->camera_parameters.annotate_x,
                                   state->camera_parameters.annotate_y
                                  );
   }
}





/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	//printf("encoder_buffer_callback\n");

   MMAL_BUFFER_HEADER_T *new_buffer;
   static int64_t base_time =  -1;
   static int64_t last_second = -1;

   // All our segment times based on the receipt of the first encoder callback
   if (base_time == -1)
      base_time = get_microseconds64()/1000;

   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
   RASPIVID_STATE *state = pData->pstate;
	

   if (pData)
   {
      int bytes_written = buffer->length;
      int64_t current_time = get_microseconds64()/1000;

      vcos_assert(pData->file_handle);
      if(pData->pstate->inlineMotionVectors) vcos_assert(pData->imv_file_handle);

      if (pData->cb_buff)
      {
         int space_in_buff = pData->cb_len - pData->cb_wptr;
         int copy_to_end = space_in_buff > buffer->length ? buffer->length : space_in_buff;
         int copy_to_start = buffer->length - copy_to_end;

         if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG)
         {
            if(pData->header_wptr + buffer->length > sizeof(pData->header_bytes))
            {
               vcos_log_error("Error in header bytes\n");
            }
            else
            {
               // These are the header bytes, save them for final output
               mmal_buffer_header_mem_lock(buffer);
               memcpy(pData->header_bytes + pData->header_wptr, buffer->data, buffer->length);
               mmal_buffer_header_mem_unlock(buffer);
               pData->header_wptr += buffer->length;
            }
         }
         else if((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
         {
            // Do something with the inline motion vectors...
         }
         else
         {
            static int frame_start = -1;
            int i;

            if(frame_start == -1)
               frame_start = pData->cb_wptr;

            if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME)
            {
               pData->iframe_buff[pData->iframe_buff_wpos] = frame_start;
               pData->iframe_buff_wpos = (pData->iframe_buff_wpos + 1) % IFRAME_BUFSIZE;
            }

            if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
               frame_start = -1;

            // If we overtake the iframe rptr then move the rptr along
            if((pData->iframe_buff_rpos + 1) % IFRAME_BUFSIZE != pData->iframe_buff_wpos)
            {
               while(
                  (
                     pData->cb_wptr <= pData->iframe_buff[pData->iframe_buff_rpos] &&
                     (pData->cb_wptr + buffer->length) > pData->iframe_buff[pData->iframe_buff_rpos]
                  ) ||
                  (
                     (pData->cb_wptr > pData->iframe_buff[pData->iframe_buff_rpos]) &&
                     (pData->cb_wptr + buffer->length) > (pData->iframe_buff[pData->iframe_buff_rpos] + pData->cb_len)
                  )
               )
                  pData->iframe_buff_rpos = (pData->iframe_buff_rpos + 1) % IFRAME_BUFSIZE;
            }

            mmal_buffer_header_mem_lock(buffer);
            // We are pushing data into a circular buffer
            memcpy(pData->cb_buff + pData->cb_wptr, buffer->data, copy_to_end);
            memcpy(pData->cb_buff, buffer->data + copy_to_end, copy_to_start);
            mmal_buffer_header_mem_unlock(buffer);

            if((pData->cb_wptr + buffer->length) > pData->cb_len)
               pData->cb_wrap = 1;

            pData->cb_wptr = (pData->cb_wptr + buffer->length) % pData->cb_len;

            for(i = pData->iframe_buff_rpos; i != pData->iframe_buff_wpos; i = (i + 1) % IFRAME_BUFSIZE)
            {
               int p = pData->iframe_buff[i];
               if(pData->cb_buff[p] != 0 || pData->cb_buff[p+1] != 0 || pData->cb_buff[p+2] != 0 || pData->cb_buff[p+3] != 1)
               {
                  vcos_log_error("Error in iframe list\n");
               }
            }
         }
      }
      else
      {


	
         // For segmented record mode, we need to see if we have exceeded our time/size,
         // but also since we have inline headers turned on we need to break when we get one to
         // ensure that the new stream has the header in it. If we break on an I-frame, the
         // SPS/PPS header is actually in the previous chunk.
         if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
               ((pData->pstate->segmentSize && current_time > base_time + pData->pstate->segmentSize) ||
                (pData->pstate->splitWait && pData->pstate->splitNow)))
         {
            FILE *new_handle;

            base_time = current_time;

            pData->pstate->splitNow = 0;
            pData->pstate->segmentNumber++;

            // Only wrap if we have a wrap point set
            if (pData->pstate->segmentWrap && pData->pstate->segmentNumber > pData->pstate->segmentWrap)
               pData->pstate->segmentNumber = 1;

            if (pData->pstate->common_settings.filename && pData->pstate->common_settings.filename[0] != '-')
            {
		pData->pstate->file_open_delegated = pData->pstate->mp4_video;
               new_handle = open_filename(pData->pstate, pData->pstate->common_settings.filename);

               if (new_handle)
               {

			if(state->mp4_video) {
				state->mp4w->endMovie();
			}

			if(new_handle != stdout) {
			  	fclose(pData->file_handle);
			}

		  pData->file_handle = new_handle;
	       }	
            }

            if (pData->pstate->imv_filename && pData->pstate->imv_filename[0] != '-')
            {
               new_handle = open_filename(pData->pstate, pData->pstate->imv_filename);

               if (new_handle)
               {
                  fclose(pData->imv_file_handle);
                  pData->imv_file_handle = new_handle;
               }
            }

            if (pData->pstate->pts_filename && pData->pstate->pts_filename[0] != '-')
            {
               new_handle = open_filename(pData->pstate, pData->pstate->pts_filename);

               if (new_handle)
               {
                  fclose(pData->pts_file_handle);
                  pData->pts_file_handle = new_handle;
               }
            }
         }
         if (buffer->length)
         {
	
            mmal_buffer_header_mem_lock(buffer);

		//printf("buffer->length=%lld\n", (long long) buffer->length);
		/*if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_NAL_END) {
			printf("NAL_END	%d\n", buffer->length + pData->bufsize);	
			pData->bufsize = 0;	
		} else {
			pData->bufsize += buffer->length;
		}*/

            if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
            {
               if(pData->pstate->inlineMotionVectors)
               {
                  bytes_written = fwrite(buffer->data, 1, buffer->length, pData->imv_file_handle);
                  if(pData->flush_buffers) fflush(pData->imv_file_handle);
               }
               else
               {
                  //We do not want to save inlineMotionVectors...
                  bytes_written = buffer->length;
               }
            }
            else
            {

		

		// TODO it gets actual fps 
		/*
		MMAL_PARAMETER_FRAME_RATE_T param = { 0 };
		param.hdr.id = MMAL_PARAMETER_VIDEO_FRAME_RATE;
	      	param.hdr.size = sizeof(param);  

		MMAL_PORT_T *camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
		if(MMAL_SUCCESS == mmal_port_parameter_get(camera_video_port, &param.hdr)) {
			vcos_log_error("MMAL_PARAMETER_FRAME_RATE_T = %d / %d", param.frame_rate.num, param.frame_rate.num);
			// 1966080 for 30 fps
			// 983040 for 15 fps
			// 655360 for 10 fps
			// 65536 for 1 fps

			state->framerate = param.frame_rate.num / 65536;

		} else {
			vcos_log_error("cam_config=error");
		} */

		/*
		if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_START) {
			printf("FRAME_START	");		
		}

		if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME) {
			printf("MMAL_BUFFER_HEADER_FLAG_KEYFRAME\n");		
		}
		
		if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
			printf("MMAL_BUFFER_HEADER_FLAG_CONFIG\n");		
		}
		*/


		/*
		if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
			for(int i=i; i < buffer->length; ++i)
				printf("%02x ", buffer->data[i]);	
			printf("\n\n");
		}*/
		
		if(state->mp4_video) {
			
			state->mp4w->copyBuffer(buffer->data, buffer->length);

			if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) { // SPS AND PPS NAL

				if(!state->mp4w->should_allow_addmovie()) {
					state->mp4w->writeNAL();

				} else if(!state->mp4w->addMovie(pData->pstate->file_name)) {
					vcos_log_error("error: openFile failed\n");

				} else if(!state->mp4w->alsa_good()) {
					state->mp4w->writeHeader(pData->pstate->file_name);

				} else if(!state->mp4w->addAudio(pData->pstate->file_name)) {
					vcos_log_error("error: addAudio failed\n");
				} else {
					state->mp4w->writeHeader(pData->pstate->file_name);
				}
			
			} else if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) { // IFRAME AND IPFRAME NULS

				if(!state->mp4w->writeNAL()) {
					vcos_log_error("error: writeNAL2 failed\n");
				}
			}

			while(state->mp4w->compare_time()) {
				state->mp4w->alsa_capture(0);
			}

		} else {

			bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
		       if(pData->flush_buffers)
		       {
		           fflush(pData->file_handle);
		           fdatasync(fileno(pData->file_handle));
		       }

		       if (pData->pstate->save_pts &&
		          !(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
		          buffer->pts != MMAL_TIME_UNKNOWN &&
		          buffer->pts != pData->pstate->lasttime)
		       {
		          int64_t pts;
		          if (pData->pstate->frame == 0)
		             pData->pstate->starttime = buffer->pts;
		          pData->pstate->lasttime = buffer->pts;
		          pts = buffer->pts - pData->pstate->starttime;
		          fprintf(pData->pts_file_handle, "%lld.%03lld\n", pts/1000, pts%1000);
		          pData->pstate->frame++;
		       }
		}
               
            }

            mmal_buffer_header_mem_unlock(buffer);

            if (bytes_written != buffer->length)
            {
               vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
               pData->abort = 1;
            }
         }
      }

      // See if the second count has changed and we need to update any annotation
      if (current_time/1000 != last_second)
      {
         update_annotation_data(pData->pstate);
         last_second = current_time/1000;
      }
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}

/**
 *  buffer header callback function for splitter
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void splitter_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
  	printf("splitter_buffer_callback\n");
	//vcos_log_error("splitter_buffer_callback");

	/*
   if (pData)
   {

      int bytes_written = 0;
      int bytes_to_write = buffer->length;

      // Write only luma component to get grayscale image: 
      if (buffer->length && pData->pstate->raw_output_fmt == RAW_OUTPUT_FMT_GRAY)
         bytes_to_write = port->format->es->video.width * port->format->es->video.height;
	
	//printf("%d	%d\n", port->format->es->video.width, port->format->es->video.height);

	//int current_time = get_microseconds64()/1000000;
	//printf("%d	%dx%d / %d	- %d \n", pData->pstate->raw_output_fmt, port->format->es->video.width, 
	//	port->format->es->video.height, bytes_to_write, current_time);
	//FILE* fp = fopen("./xxxxxx.bmp", "w");

	cv::Mat& image = pData->pstate->opencv_image;
	if(image.empty()) {
		image.create(pData->pstate->opencv_height, pData->pstate->opencv_width, CV_8UC1);
	}

	mmal_buffer_header_mem_lock(buffer);
	memcpy(image.data, buffer->data, image.cols * image.rows);
	mmal_buffer_header_mem_unlock(buffer);

	cv::imwrite("/mnt/usb/motion/split.bmp", image);

	int64_t diff = get_microseconds64() - pData->pstate->opencv_next_update;
	if(diff >= 0 && vcos_semaphore_trywait(&(pData->pstate->frame_complete)) != VCOS_SUCCESS) {
		vcos_semaphore_post(&(pData->pstate->frame_complete));
	}

	sleep(1);
	
	
	//if(pData->pstate->running) {
	//	//printf("wait....1\n");
	//	std::unique_lock<std::mutex> lock(pData->pstate->mutex);
	//	pData->pstate->con.wait(lock);
	//	//printf("wait....2\n");
	//}

	
	//if(diff < 0) {
	//	usleep(1000000);
	//} else if(diff > 5000000) {
	//	usleep(1000000);
	//} else {
	//	usleep(diff);
	//}
   }
   else
   {
      vcos_log_error("Received a camera buffer callback with no state");
   }
*/
	
sleep(1);
   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->splitter_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the splitter port");
   }
}

static void resizer_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{

	MMAL_BUFFER_HEADER_T *new_buffer;
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
	//vcos_log_error("resizer_buffer_callback");
	//printf("resizer_buffer_callback\n");

	if (pData)
	{
		//printf("%d	%d\n", port->format->es->video.width, port->format->es->video.height);

		RASPIVID_STATE* pstate = pData->pstate;
		cv::Mat& image = pstate->opencv_image;
		cv::Mat& image2 = pstate->opencv_image2;
		//cv::Mat& image3 = pstate->opencv_image3;
		

		if(image.empty()) {
			image.create(port->format->es->video.crop.height, port->format->es->video.crop.width, CV_8UC1);
			pstate->motion_thres = 200;
		}

		mmal_buffer_header_mem_lock(buffer);
		memcpy(image.data, buffer->data, image.cols * image.rows);
		mmal_buffer_header_mem_unlock(buffer);

		if(image2.empty()) {
			image.copyTo(image2);
		}

		//cv::imwrite("/mnt/usb/motion/resize2.bmp", image);
	
		switch(frame_diff(image, image2, pstate)) {
			case 2: // motion detected
				pstate->motion_thres = 20;
				if(++pstate->skip_frame > 50) {
					pstate->skip_frame = 0;
					image.copyTo(image2);
				}
				break;
			case 1: // frames are un-equal	
				pstate->skip_frame = 0;
				pstate->motion_thres = 200;
				pstate->opencv_next_update = time(NULL);
				//fprintf(stderr, "copy_1=%d\n", pstate->opencv_next_update);
				image.copyTo(image2);
				break;
			default: // frames are equal
				pstate->skip_frame = 0;
				pstate->motion_thres = 200;
				pstate->opencv_next_update = time(NULL);
				//fprintf(stderr, "copy_0=%d\n", pstate->opencv_next_update);
				image.copyTo(image2);
		} 

	} 

	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;

		new_buffer = mmal_queue_get(pData->pstate->resizer_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			vcos_log_error("Unable to return a buffer to the resizer port");
	}

	//sleep(1);	

}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */

//#define ENCODING 

#define USE_ENCODING MMAL_ENCODING_I420
//#define USE_ENCODING MMAL_ENCODING_OPAQUE
static MMAL_STATUS_T create_camera_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
   MMAL_STATUS_T status;

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create camera component");
       if (camera)
       mmal_component_destroy(camera);
   }

   status = (MMAL_STATUS_T)raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
   status = (MMAL_STATUS_T)((int)status + 
raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode));
   status = (MMAL_STATUS_T)((int)status + raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode));

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set stereo mode : error %d", status);
       if (camera)
       mmal_component_destroy(camera);
   }

   MMAL_PARAMETER_INT32_T camera_num =
   {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

   status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", status);
       if (camera)
       mmal_component_destroy(camera);
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera doesn't have output ports");
       if (camera)
       mmal_component_destroy(camera);
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
       if (camera)
       mmal_component_destroy(camera);
   }

   preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, default_camera_control_callback);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable control port : error %d", status);
       if (camera)
       mmal_component_destroy(camera);
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->common_settings.width,
         .max_stills_h = state->common_settings.height,
         .stills_yuv422 = 0,
         .one_shot_stills = 0,
         .max_preview_video_w = state->common_settings.width,
         .max_preview_video_h = state->common_settings.height,
         .num_preview_video_frames = 3 + vcos_max(0, (state->framerate-30)/10),
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 0,
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
      };
      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   // Now set up the port formats

   // Set the encode format on the Preview port
   // HW limitations mean we need the preview to be the same size as the required recorded output

   format = preview_port->format;
   format->encoding = USE_ENCODING;//MMAL_ENCODING_I420;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 5, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 166, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }

   //enable dynamic framerate if necessary
   if (state->camera_parameters.shutter_speed)
   {
      if (state->framerate > 1000000./state->camera_parameters.shutter_speed)
      {
         state->framerate=0;
         if (state->common_settings.verbose)
            fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
      }
   }

   format->encoding = USE_ENCODING;//MMAL_ENCODING_I420; //MMAL_ENCODING_RGB24
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   //format->es->video.width = VCOS_ALIGN_UP(state->opencv_width, 32);
   //format->es->video.height = VCOS_ALIGN_UP(state->opencv_height, 16);
  
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.crop.width = state->opencv_width;
   format->es->video.crop.height = state->opencv_height;
  
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(preview_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera viewfinder format couldn't be set");
       if (camera)
       mmal_component_destroy(camera);
   }

   // Set the encode format on the video  port

   format = video_port->format;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 5, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 167, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }

   format->encoding = MMAL_ENCODING_I420;//MMAL_ENCODING_OPAQUE;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(video_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera video format couldn't be set");
       if (camera)
       mmal_component_destroy(camera);
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


   // Set the encode format on the still  port

   format = still_port->format;

   format->encoding = USE_ENCODING;
   format->encoding_variant = MMAL_ENCODING_I420;

   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;


   status = mmal_port_format_commit(still_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera still format couldn't be set");
      if (camera)
       mmal_component_destroy(camera);
   }

   /* Ensure there are enough buffers to avoid dropping frames */
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera component couldn't be enabled");
      if (camera)
       mmal_component_destroy(camera);
   }

   // Note: this sets lots of parameters that were not individually addressed before.
   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   state->camera_component = camera;

   update_annotation_data(state);

   if (state->common_settings.verbose)
      fprintf(stderr, "Camera component done\n");

   return status;

//error:

 //  if (camera)
//      mmal_component_destroy(camera);

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *splitter = 0;
   MMAL_PORT_T *splitter_output = NULL;
   MMAL_ES_FORMAT_T *format;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;
   int i;

   if (state->camera_component == NULL)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera component must be created before splitter");
      goto error;
   }

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create splitter component");
      goto error;
   }

   if (!splitter->input_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Splitter doesn't have any input port");
      goto error;
   }

   if (splitter->output_num < 2)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Splitter doesn't have enough output ports");
      goto error;
   }

   /* Ensure there are enough buffers to avoid dropping frames: */
   mmal_format_copy(splitter->input[0]->format, state->camera_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);

   if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   status = mmal_port_format_commit(splitter->input[0]);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on splitter input port");
      goto error;
   }

   /* Splitter can do format conversions, configure format for its output port: */

   for (i = 0; i < splitter->output_num; i++)
   {
	//splitter->input[0]->format->es->video.crop.height = VCOS_ALIGN_UP(state->common_settings.height/6, 16);
	//splitter->input[0]->format->es->video.crop.width = VCOS_ALIGN_UP(state->common_settings.width/6, 32);

      	mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);
	MMAL_ES_FORMAT_T *format = splitter->output[i]->format;
	
	//format->es->video.height = VCOS_ALIGN_UP(480, 16);	
   	//format->es->video.width = VCOS_ALIGN_UP(640, 32);
   	
   	
   	//format->es->video.crop.width = 640;
   	//format->es->video.crop.height = 480;
   	//format->es->video.frame_rate.num = 1;
   	//format->es->video.frame_rate.den = 1;


      if (i == SPLITTER_OUTPUT_PORT)
      {
         format = splitter->output[i]->format;	

         switch (state->raw_output_fmt)
         {
         case RAW_OUTPUT_FMT_YUV:
         case RAW_OUTPUT_FMT_GRAY: /* Grayscale image contains only luma (Y) component */
            format->encoding = MMAL_ENCODING_I420;
            format->encoding_variant = MMAL_ENCODING_I420;
            break;
         case RAW_OUTPUT_FMT_RGB:
            if (mmal_util_rgb_order_fixed(state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT]))
               format->encoding = MMAL_ENCODING_RGB24;
            else
               format->encoding = MMAL_ENCODING_BGR24;
            format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */
            break;
         default:
            status = MMAL_EINVAL;
            vcos_log_error("unknown raw output format");
            goto error;
         }
      } /*else {
		format = splitter->output[i]->format;
		format->es->video.width = VCOS_ALIGN_UP(640, 32);
         format->es->video.height = VCOS_ALIGN_UP(480, 16);

         format->es->video.crop.width = 640;
         format->es->video.crop.height = 480;
         format->es->video.frame_rate.num = 1;
         format->es->video.frame_rate.den = 1;

	}*/

      status = mmal_port_format_commit(splitter->output[i]);

      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set format on splitter output port %d", i);
         goto error;
      }
   }

   /* Enable component */
   status = mmal_component_enable(splitter);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("splitter component couldn't be enabled");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   splitter_output = splitter->output[SPLITTER_OUTPUT_PORT];
   pool = mmal_port_pool_create(splitter_output, splitter_output->buffer_num, splitter_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for splitter output port %s", splitter_output->name);
   }

   state->splitter_pool = pool;
   state->splitter_component = splitter;

   if (state->common_settings.verbose)
      fprintf(stderr, "Splitter component done\n");

   return status;

error:

   if (splitter)
      mmal_component_destroy(splitter);

   return status;
}

static MMAL_STATUS_T create_resizer_component(RASPIVID_STATE *state) {

	MMAL_COMPONENT_T *resizer = 0;
	MMAL_PORT_T *resizer_input = NULL;
	MMAL_PORT_T *resizer_output = NULL;
	MMAL_ES_FORMAT_T *format;
	MMAL_STATUS_T status;
	MMAL_POOL_T *pool = 0;

	if (state->camera_component == NULL) {
		status = MMAL_ENOSYS;
		vcos_log_error("Camera component must be created before resizer");
		goto error;
	}


	/* Create the component */
	status = mmal_component_create("vc.ril.isp", &resizer);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create resizer component");
		goto error;
	}

	if (!resizer->input_num || !resizer->output_num)
	{
		status = MMAL_ENOSYS;
		vcos_log_error("Video encoder doesn't have input/output ports");
		goto error;
	}

	resizer_input = resizer->input[0];
	resizer_output = resizer->output[RESIZER_OUTPUT_PORT];

	// We want same format on input and output
	mmal_format_copy(resizer_input->format, state->camera_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);
		

	resizer_input->format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height/6, 32);
   	resizer_input->format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width/6, 128); // 3d require width = 128x

	resizer_input->format->es->video.crop.height = resizer_input->format->es->video.height;
	resizer_input->format->es->video.crop.width = resizer_input->format->es->video.width;
	
	//vcos_log_error("video_size=%d x %d\n", resizer_input->format->es->video.crop.width, resizer_input->format->es->video.crop.height);

	//resizer_input->format->es->video.crop.height = state->common_settings.height;
	//resizer_input->format->es->video.crop.width = state->common_settings.width; // 3d require width = 128x

	mmal_format_copy(resizer_output->format, resizer_input->format);
	
	//vcos_log_error("VIDEO_OUTPUT_BUFFERS_NUM=%d\n", VIDEO_OUTPUT_BUFFERS_NUM);

	if (resizer->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		resizer->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	status = mmal_port_format_commit(resizer->input[0]);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to set format on resizer input port");
		goto error;
	}

	mmal_format_copy(resizer_output->format, resizer->input[0]->format);
	format = resizer_output->format;	
   	

	switch (state->raw_output_fmt) {
         case RAW_OUTPUT_FMT_YUV:
		//vcos_log_error("RAW_OUTPUT_FMT_YUV\n");
         case RAW_OUTPUT_FMT_GRAY: /* Grayscale image contains only luma (Y) component */
            format->encoding = MMAL_ENCODING_I420;
            format->encoding_variant = MMAL_ENCODING_I420;
		//vcos_log_error("RAW_OUTPUT_FMT_GRAY\n");	
            break;
         case RAW_OUTPUT_FMT_RGB:
		//vcos_log_error("RAW_OUTPUT_FMT_RGB\n");	
            if (mmal_util_rgb_order_fixed(state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT]))
               format->encoding = MMAL_ENCODING_RGB24;
            else
               format->encoding = MMAL_ENCODING_BGR24;
            format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */
            break;
         default:
		//vcos_log_error("MMAL_EINVAL\n");	
            status = MMAL_EINVAL;
            vcos_log_error("unknown raw output format");
            goto error;
	}

	status = mmal_port_format_commit(resizer_output);
	//return status;

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to set format on resizer output port 0");
		goto error;
	}

	/* Enable component */
	status = mmal_component_enable(resizer);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("resizer component couldn't be enabled");
		goto error;
	}

	/* Create pool of buffer headers for the output port to consume */
	//resizer_output = resizer->output[0];
	pool = mmal_port_pool_create(resizer_output, resizer_output->buffer_num, resizer_output->buffer_size);

	if (!pool)
	{
		vcos_log_error("Failed to create buffer header pool for resizer output port %s", resizer_output->name);
	}

	state->resizer_pool = pool;
	state->resizer_component = resizer;

	if (state->common_settings.verbose)
		fprintf(stderr, "Resizer component done\n");

	return status;

error:
	 if (resizer)
		mmal_component_destroy(resizer);

	return status;
}

/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_splitter_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->splitter_pool)
   {
      mmal_port_pool_destroy(state->splitter_component->output[SPLITTER_OUTPUT_PORT], state->splitter_pool);
   }

   if (state->splitter_component)
   {
      mmal_component_destroy(state->splitter_component);
      state->splitter_component = NULL;
   }
}

/**
 * Destroy the resizer component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_resizer_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->resizer_pool)
   {
      mmal_port_pool_destroy(state->resizer_component->output[0], state->resizer_pool);
   }

   if (state->resizer_component)
   {
      mmal_component_destroy(state->resizer_component);
      state->resizer_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Only supporting H264 at the moment
   encoder_output->format->encoding = state->encoding;

   if(state->encoding == MMAL_ENCODING_H264)
   {
      if(state->level == MMAL_VIDEO_LEVEL_H264_4)
      {
         if(state->bitrate > MAX_BITRATE_LEVEL4)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            state->bitrate = MAX_BITRATE_LEVEL4;
         }
      }
      else
      {
         if(state->bitrate > MAX_BITRATE_LEVEL42)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
            state->bitrate = MAX_BITRATE_LEVEL42;
         }
      }
   }
   else if(state->encoding == MMAL_ENCODING_MJPEG)
   {
      if(state->bitrate > MAX_BITRATE_MJPEG)
      {
         fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
         state->bitrate = MAX_BITRATE_MJPEG;
      }
   }

   encoder_output->format->bitrate = state->bitrate;

   if (state->encoding == MMAL_ENCODING_H264)
      encoder_output->buffer_size = encoder_output->buffer_size_recommended;
   else
      encoder_output->buffer_size = 256<<10;


   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // We need to set the frame rate on output to 0, to ensure it gets
   // updated correctly from the input framerate when port connected
   encoder_output->format->es->video.frame_rate.num = 0;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      goto error;
   }

   // Set the rate control parameter
   if (0)
   {
      MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set ratecontrol");
         goto error;
      }

   }

   if (state->encoding == MMAL_ENCODING_H264 &&
         state->intraperiod != -1)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set intraperiod");
         goto error;
      }
   }

   if (state->encoding == MMAL_ENCODING_H264 && state->slices > 1 && state->common_settings.width <= 1280)
   {
      int frame_mb_rows = VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4;

      if (state->slices > frame_mb_rows) //warn user if too many slices selected
      {
         fprintf(stderr,"H264 Slice count (%d) exceeds number of macroblock rows (%d). Setting slices to %d.\n", state->slices, frame_mb_rows, frame_mb_rows);
         // Continue rather than abort..
      }
      int slice_row_mb = frame_mb_rows/state->slices;
      if (frame_mb_rows - state->slices*slice_row_mb)
         slice_row_mb++; //must round up to avoid extra slice if not evenly divided

      status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set number of slices");
         goto error;
      }
   }

   if (state->encoding == MMAL_ENCODING_H264 &&
       state->quantisationParameter)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set initial QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param2.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set min QP");
         goto error;
      }

      MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param3.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set max QP");
         goto error;
      }
   }

   if (state->encoding == MMAL_ENCODING_H264)
   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = (MMAL_VIDEO_PROFILE_T)state->profile;

      if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate > 245760)
      {
         if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate <= 522240)
         {
            fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
            state->level=MMAL_VIDEO_LEVEL_H264_42;
         }
         else
         {
            vcos_log_error("Too many macroblocks/s requested");
            status = MMAL_EINVAL;
            goto error;
         }
      }

      param.profile[0].level = (MMAL_VIDEO_LEVEL_T)state->level;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
         goto error;
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   if (state->encoding == MMAL_ENCODING_H264)
   {
      //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE HEADER FLAG parameters");
         // Continue rather than abort..
      }

      //set flag for add SPS TIMING
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
         // Continue rather than abort..
      }

      //set INLINE VECTORS flag to request motion vector estimates
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE VECTORS parameters");
         // Continue rather than abort..
      }

      // Adaptive intra refresh settings
      if ( state->intra_refresh_type != -1)
      {
         MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
         param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
         param.hdr.size = sizeof(param);

         // Get first so we don't overwrite anything unexpectedly
         status = mmal_port_parameter_get(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
            // Set some defaults, don't just pass random stack data
            param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
         }

         param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T) state->intra_refresh_type;

         //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
         //   param.cir_mbs = 10;

         status = mmal_port_parameter_set(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Unable to set H264 intra-refresh values");
            goto error;
         }
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   state->encoder_pool = pool;
   state->encoder_component = encoder;

   if (state->common_settings.verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;

error:
   if (encoder)
      mmal_component_destroy(encoder);

   state->encoder_component = NULL;

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->encoder_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}

//FILE *g_debug = NULL;

/**
 * Pause for specified time, but return early if detect an abort request
 *
 * @param state Pointer to state control struct
 * @param pause Time in ms to pause
 * @param callback Struct contain an abort flag tested for early termination
 *
 */
static int pause_and_test_abort(RASPIVID_STATE *state, int pause)
{
   int wait;

   if (!pause)
      return 0;

   // Going to check every ABORT_INTERVAL milliseconds
   for (wait = 0; wait < pause; wait+= ABORT_INTERVAL)
   {
      vcos_sleep(ABORT_INTERVAL);
      if (state->callback_data.abort) 
         return 1;

   }

   return 0;
}


/**
 * Function to wait in various ways (depending on settings)
 *
 * @param state Pointer to the state data
 *
 * @return !0 if to continue, 0 if reached end of run
 */
static int wait_for_next_change(RASPIVID_STATE *state)
{
   int keep_running = 1;
   static int64_t complete_time = -1;

   // Have we actually exceeded our timeout?
   int64_t current_time =  get_microseconds64()/1000;

   if (complete_time == -1)
      complete_time =  current_time + state->timeout;

   // if we have run out of time, flag we need to exit
   if (current_time >= complete_time && state->timeout != 0)
      keep_running = 0;

	

   switch (state->waitMethod)
   {
   case WAIT_METHOD_NONE:
      (void)pause_and_test_abort(state, state->timeout);
      return 0;

   case WAIT_METHOD_FOREVER:
   {
      // We never return from this. Expect a ctrl-c to exit or abort.
      while (!state->callback_data.abort)
         // Have a sleep so we don't hog the CPU.
         vcos_sleep(ABORT_INTERVAL);

      return 0;
   }

   case WAIT_METHOD_TIMED:
   {
      int abort;
      if (state->bCapturing)
         abort = pause_and_test_abort(state, state->onTime);
      else
         abort = pause_and_test_abort(state, state->offTime);

      if (abort)
         return 0;
      else
         return keep_running;
   }

   case WAIT_METHOD_KEYPRESS:
   {
      char ch;

      if (state->common_settings.verbose)
         fprintf(stderr, "Press Enter to %s, X then ENTER to exit, [i,o,r] then ENTER to change zoom\n", state->bCapturing ? "pause" : "capture");

      ch = getchar();
      if (ch == 'x' || ch == 'X')
         return 0;
      else if (ch == 'i' || ch == 'I')
      {
         if (state->common_settings.verbose)
            fprintf(stderr, "Starting zoom in\n");

         raspicamcontrol_zoom_in_zoom_out(state->camera_component, ZOOM_IN, &(state->camera_parameters).roi);

         if (state->common_settings.verbose)
            dump_status(state);
      }
      else if (ch == 'o' || ch == 'O')
      {
         if (state->common_settings.verbose)
            fprintf(stderr, "Starting zoom out\n");

         raspicamcontrol_zoom_in_zoom_out(state->camera_component, ZOOM_OUT, &(state->camera_parameters).roi);

         if (state->common_settings.verbose)
            dump_status(state);
      }
      else if (ch == 'r' || ch == 'R')
      {
         if (state->common_settings.verbose)
            fprintf(stderr, "starting reset zoom\n");

         raspicamcontrol_zoom_in_zoom_out(state->camera_component, ZOOM_RESET, &(state->camera_parameters).roi);

         if (state->common_settings.verbose)
            dump_status(state);
      }

      return keep_running;
   }


   case WAIT_METHOD_SIGNAL:
   {
      // Need to wait for a SIGUSR1 signal
      sigset_t waitset;
      int sig;
      int result = 0;
      sigemptyset( &waitset );
      sigaddset( &waitset, SIGUSR1 );

      // We are multi threaded because we use mmal, so need to use the pthread
      // variant of procmask to block SIGUSR1 so we can wait on it.
      pthread_sigmask( SIG_BLOCK, &waitset, NULL );

      if (state->common_settings.verbose)
      {
         fprintf(stderr, "Waiting for SIGUSR1 to %s\n", state->bCapturing ? "pause" : "capture");
      }

      result = sigwait( &waitset, &sig );

      if (state->common_settings.verbose && result != 0)
         fprintf(stderr, "Bad signal received - error %d\n", errno);

      return keep_running;
   }

   } // switch

   return keep_running;
}

void soundAlert(int n, int d) {

	/*for(int i=0; i < n; ++i ) {
		digitalWrite(36, HIGH);
		delay(d);
		digitalWrite(36, LOW);
		delay(d);
	}*/
}

static int frame_diff(cv::Mat& image, cv::Mat& image2, RASPIVID_STATE* state) {
	//printf("image %dx%d\n", image.cols, image.rows);
	//cv::GaussianBlur(m2, dst, ksize, 0.0);

	//fprintf(stderr, "frame_diff: contours=%d\n", 111);
	//return 0;
	
	cv::Mat& diff = state->diff;
	cv::absdiff(image, image2, diff);	
	cv::dilate(diff, diff, state->kernal);
	cv::threshold(diff, diff, 20, 255, cv::THRESH_BINARY); //THRESH_BINARY

	//cv::adaptiveThreshold(diff, diff, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2); //THRESH_BINARY
	//if(cv::countNonZero(diff)*100.0/diff.total() < 0.5) return 0;
			
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	//state->motion_thres = 200;
	//int64_t time5 = get_microseconds64();
	//fprintf(stderr, "contours=%d\n", contours.size());
	
	int detected = 0;
	for(size_t i = 0; i < contours.size(); ++i) {
		std::vector<cv::Point>& contour = contours.at(i);
		if(cv::contourArea(contour) > state->motion_thres) {
			//cv::Rect r = cv::boundingRect(contour);
			//cv::rectangle(image, cv::Point(r.x, r.y), cv::Point(r.x+r.width, r.y+r.height), 0xFFFFFF, 2);
			detected++;
			break;
		}
	}

	MMAL_BOOL_T capturing = 0;
	MMAL_PORT_T *camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
	mmal_port_parameter_get_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, &capturing);
	MMAL_ES_FORMAT_T *format = camera_video_port->format;


	//fprintf(stderr, ">>> contours=%d\n", contours.size());
	//fprintf(stderr, ">>>: contours=%d, detected=%d\n", contours.size(), detected);

	if(detected) {
		//copy = false;

		//format->es->video.frame_rate.num = 5;
		//mmal_port_format_commit(camera_video_port);

		if(!capturing) {
			mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1);
			fprintf(stderr, "OBJECT_ENTER: contours=%d, %d->%d/%d\n",
				contours.size(), time(NULL), state->opencv_next_update, (time(NULL) - state->opencv_next_update));
			
			//cv::imwrite("/mnt/usb/motion/image0.bmp", image2);
			//cv::imwrite("/mnt/usb/motion/image1.bmp", image);
		}

		return 2;

	} else if(capturing) {

		//format->es->video.frame_rate.num = 1;
		//mmal_port_format_commit(camera_video_port);

		mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 0);
		fprintf(stderr, "OBJECT_GONE: contours=%d\n", contours.size());

	}

	//printf("%d	%lld/%lld/%lld/%lld\n", detected, time1-time2, time2-time3, time3-time4, time4-time5);

	//return 1;
	return contours.empty() ? 0 : 1;
}

void *opencv_thread(void *pstate) {
	
	printf("OpenCV thread started... \n");

	RASPIVID_STATE* state = (RASPIVID_STATE*)pstate;
	MMAL_PORT_T *camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];


	cv::Size minBox(50, 50);
	cv::Size maxBox(100, 100);
	cv::Mat& m1 = state->opencv_image;

	int y1 = 180;
	int y2 = 600;
	int x1 = 0;
	int x2 = m1.cols;


	cv::Mat m2 = m1(cv::Range(y1, y2), cv::Range(x1, x2));
	int total_object_detected = 0;

	rp::DispManX* dsm = state->disp;
	dsm->init(x1, y1, x2, y2);
	
	rp::CNN *cnn = state->cnn;

	float mat1[] =  {1.0, 1.0, 1.0,
			1.0, 1.0, 1.0,
			1.0, 1.0, 1.0};
	
	for(int i=0; i < sizeof(mat1)/sizeof(float); ++i) {
		mat1[i] = mat1[i] / (sizeof(mat1)/sizeof(float));
	}

	/*
	for(int i=0; i < sizeof(mat1)/sizeof(float); ++i) {
		printf("%.2f, ", mat1[i]);
	}
	printf("\n");
	*/


	cv::Size ksize(31, 31);
	cv::Mat m3, mk(3, 3, CV_32F);
	cv::Mat dst, old, diff;
	cv::Mat kernal(5, 5, CV_8UC1, 1); 
	memcpy(mk.data, mat1, sizeof(mat1));
	cv::HOGDescriptor hog;
	hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

	//for(int i=0; i < kernal.rows; ++i)
	//	for(int j=0; j < kernal.cols; ++j)
	//		printf("%d, ", kernal.at<uint8_t>(i, j));

	int k = 0;
	while(state->running) {

		if(vcos_semaphore_wait(&(state->frame_complete)) == VCOS_SUCCESS && state->running) {

			/*
			char buf[128];	
			snprintf(buf, 128, "/mnt/usb/motion/xxx_%d.bmp", k++);
			cv::imwrite(buf, m1);
			continue; */

			continue;


			/*
			cv::imwrite("/mnt/usb/motion/xxx.bmp", m2);
			long time1 = get_microseconds64();
			cnn->conv1(m2, m3, mk);
			//cv::filter2D(m2, m3, -1, mk);
			long time2 = get_microseconds64();
			cv::imwrite("/mnt/usb/motion/xxx1.bmp", m3);
			printf("%.2f	%dx%d\n", (time2 - time1)/1000.0, m3.cols, m3.rows);
			*/

			int64_t time1 = get_microseconds64();
			state->opencv_next_update = time1 + 1000000;
			//printf("xxxxxxxxxxxxxxx\n"); 
			//sleep(1);
			//state->con.notify_one();
			//continue;

			
			cv::resize(m1, dst, cv::Size(320, 180), 0, 0, cv::INTER_CUBIC);
			//cv::GaussianBlur(m2, dst, ksize, 0.0);
			int64_t time2 = get_microseconds64();
			//cv::imwrite("/mnt/usb/motion/xxx.bmp", dst);
			//continue;

			if(old.empty()) {
				dst.copyTo(old);
				continue;
			}
			
			cv::absdiff(old, dst, diff);
			int64_t time3 = get_microseconds64();
			dst.copyTo(old);

			cv::dilate(diff, diff, kernal);
			int64_t time4 = get_microseconds64();
			cv::threshold(diff, diff, 20, 255, cv::THRESH_BINARY);
			int64_t time5 = get_microseconds64();
			
			std::vector<std::vector<cv::Point>> contours;
			cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			int64_t time6 = get_microseconds64();
			//fprintf(stderr, "contours=%d\n", contours.size());
			//cv::imwrite("/mnt/usb/motion/xxx0.bmp", m1);
			//cv::imwrite("/mnt/usb/motion/xxx1.bmp", diff1);
			//cv::imwrite("/mnt/usb/motion/xxx.bmp", diff);
			//continue;

			std::vector<cv::Rect> bounds;
			std::vector<cv::Point> points;
			//cv::Mat m2 = state->opencv_image(cv::Range(y1, y2), cv::Range(0, x2));
			
			//hog.detect(m2, points);
			//hog.detectMultiScale(m2, bounds);//, 0, cv::Size(8, 8), cv::Size(0, 0), 1.05, 2);
			//state->cascade.detectMultiScale(m2, bounds, 1.1, 3, 0, minBox, maxBox);
			//printf("zzzzzzzzzz\n");

			printf("%lld/%lld/%lld/%lld/%lld\n", time1-time2, time2-time3, time3-time4, time4-time5, time5-time6);


			//fprintf(stderr, "%.2f, contours = %d\n", (time2 - time1)/1000.0, contours.size());
			//

			int detected = 0;
			for(size_t i = 0; i < contours.size(); ++i) {
				std::vector<cv::Point>& contour = contours.at(i);
				if(cv::contourArea(contour) > 400) {
					cv::Rect r = cv::boundingRect(contour);
					cv::rectangle(dst, cv::Point(r.x, r.y), cv::Point(r.x+r.width, r.y+r.height), 0xFFFFFF, 2);
					detected++;
				}
			}

			if(detected) {
				state->opencv_next_update = get_microseconds64() + 10000000;
				mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1);
				fprintf(stderr, "MOTION_DETECTED: %.2f, contours=%d, detected=%d\n", (time2 - time1)/1000.0,
					contours.size(), detected);
				cv::imwrite("/mnt/usb/motion/xxx.bmp", m1);
				cv::imwrite("/mnt/usb/motion/xxx1.bmp", dst);

			} else {
				mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 0);
				state->opencv_next_update = get_microseconds64() + 1000000;
				//fprintf(stdout, "MOTION_NOT_DETECTED\n");
				
			}
			
			//printf("%d	%d\n" , m1.cols, m1.rows);
			//printf("%d	%d\n" , m2.cols, m2.rows);
			//printf("%.2f	total = %d, bounds  = %d\n", (time2 - time1)/1000.0, total_object_detected,  bounds.size());

			if(dsm->hasDisplay()) {

				dsm->clear();
				dsm->fillRect(0, 0, maxBox.width, maxBox.height, 0xF000);
				dsm->fillRect(0, 0, minBox.width, minBox.height, 0x0F00);

				
				if(!points.empty()) {
					soundAlert(5, 20);
					for(size_t i=0; i < points.size(); ++i) {
						cv::Point p = points.at(i);
						dsm->fillRect((p.x-50), (p.y-50), 100, 100, 0x001F);	
					}
				}

				if(!bounds.empty()) {
					soundAlert(5, 20);
					total_object_detected++;
					for(size_t i=0; i < bounds.size(); ++i) {
						cv::Rect r = bounds.at(i);
						dsm->fillRect(r.x, r.y, r.width, r.height, 0x001F);	
					}
				}	
			}

			state->disp->finalize(); 
		}
	} 

	printf("OpenCV thread exiting...\n");
	
	return pstate;
}

FILE* rp::MP4Wrapper::fp_ = NULL;
RASPIVID_STATE* gptr_state = NULL;
void sighandler(int signal) {

	printf("SIGINT received (%d).\n", signal);
	gptr_state->callback_data.abort = 1;
	//gptr_state->con.notify_one();
	//gptr_state->running = 0;
}


/*
int file_writer_callback(uint8_t *buf, int len) {
	return fwrite(buf, sizeof(uint8_t), len, gptr_state->callback_data.file_handle);
}


int64_t file_seek_callback(int64_t offset, int whence) {

	if(fseek(gptr_state->callback_data.file_handle, offset, whence) == 0) {
		return offset;
	}

	return -1;
}*/

void audio_func_call(void* userdata, u_char* buffer, int samples, int bytes) {
	//fprintf(stderr, "%s(userdata=%x, samples=%x, bytes=%d)\n", __func__, userdata, buffer, bytes);
	rp::MP4Wrapper* mp4w = (rp::MP4Wrapper*) userdata;
	mp4w->writeSample(buffer, samples, bytes);
}

/**
 * main
 */
int main(int argc, const char **argv)
{
	printf("Hello OpenCV_%d.%d.%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);
	//signal(SIGINT, signalHandler);  
	signal(SIGTERM, sighandler);
	signal(SIGINT, sighandler);

   // Our main data storage vessel..

   RASPIVID_STATE state;
   int exit_code = EX_OK;
	gptr_state = &state;

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_preview_port = NULL;
   MMAL_PORT_T *camera_video_port = NULL;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *preview_input_port = NULL;
   MMAL_PORT_T *resizer_input_port = NULL;
   MMAL_PORT_T *resizer_output_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;
   MMAL_PORT_T *splitter_input_port = NULL;
   MMAL_PORT_T *splitter_output_port = NULL;
   MMAL_PORT_T *splitter_preview_port = NULL;

  

   check_camera_stack();

   bcm_host_init();

	
/*	if(wiringPiSetupPhys() == -1) {
		vcos_log_error("%s: wiringPiSetupSys failed", __func__);
	} else {
		pinMode(36, OUTPUT);
	}
*/

   // Register our application with the logging system
   vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

   //signal(SIGINT, default_signal_handler);

   // Disable USR1 for the moment - may be reenabled if go in to signal capture mode
   signal(SIGUSR1, SIG_IGN);

   set_app_name(argv[0]);

   // Do we have any parameters
   if (argc == 1)
   {
      display_valid_parameters((char*)basename(get_app_name()), &application_help_message);
      exit(EX_USAGE);
   }

   default_status(&state);

   // Parse the command line and put options in to our status structure
   if (parse_cmdline(argc, argv, &state))
   {
      status = (MMAL_STATUS_T) -1;
      exit(EX_USAGE);
   }

   if (state.timeout == -1)
      state.timeout = 5000;

   // open cv configuration
   state.opencv_width = state.common_settings.width;
   state.opencv_height = state.common_settings.height;

   printf("Camera is: %d x %d\n", state.common_settings.width, state.common_settings.height);
	//printf(">>>> %s\n", MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER);


   //state.opencv_image.create(state.opencv_height, state.opencv_width, CV_8UC1);
   vcos_semaphore_create(&state.frame_complete, "opencv_demo", 0);

   state.cascade.load("/mnt/usb/raspberry/opencv/data/haarcascades/haarcascade_frontalface_alt.xml");
   //state.cascade.load("/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml");
   if(state.cascade.empty()) {
      vcos_log_error("%s: Empty cascade file", __func__);
   }

	//printf(">>>>>>>> r=%d, rf=%d, mo=%d\n", state.raw_output, state.raw_output_fmt, state.detect_motion);
	rp::DispManX dm(state.common_settings.width, state.common_settings.height, 120, 0, state.detect_motion);
	state.disp = &dm;

	//vcos_log_error("state.use_resizer=%d", state.use_resizer);

	//state.use_resizer = 1;
	if(state.use_resizer) {
		state.preview_parameters.wantPreview = 0;
		state.kernal.create(5, 5, CV_8UC1);
		state.kernal.setTo(1);
	}

	rp::MP4Wrapper mp4w(rp::AlsaMic::alsamic_sink_pcm,
		state.common_settings.width, state.common_settings.height,
		state.framerate, state.bitrate, 64000, state.mp4_logging);
	//mp4w.ffmpeg_file_callbacks((void*)&mp4w, &file_writer_callback, &file_seek_callback);
	mp4w.set_call_func((void*)&mp4w, &audio_func_call);
	
	state.mp4w = &mp4w;

	rp::CNN cnn;
	state.cnn = &cnn;
	
   // Setup for sensor specific parameters, only set W/H settings if zero on entry
   get_sensor_defaults(state.common_settings.cameraNum, state.common_settings.camera_name,
                       &state.common_settings.width, &state.common_settings.height);

   if (state.common_settings.verbose)
   {
      print_app_details(stderr);
      dump_status(&state);
   }

   check_camera_model(state.common_settings.cameraNum);

   if (state.common_settings.gps)
      if (raspi_gps_setup(state.common_settings.verbose))
         state.common_settings.gps = 0;

   // OK, we have a nice set of parameters. Now set up our components
   // We have three components. Camera, Preview and encoder.

   if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create camera component", __func__);
      exit_code = EX_SOFTWARE;
   }
   else if ((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create preview component", __func__);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }
   else if ((status = create_encoder_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create encode component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }
   else if (state.raw_output && state.use_resizer != 2 && (status = create_splitter_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create splitter component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      destroy_encoder_component(&state);
      exit_code = EX_SOFTWARE;
   }
   else if (state.use_resizer==2 && (status = create_resizer_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create resizer component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      destroy_encoder_component(&state);
      destroy_splitter_component(&state);
      exit_code = EX_SOFTWARE;
   }	
   else
   {
      if (state.common_settings.verbose)
         fprintf(stderr, "Starting component connection stage\n");

      camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
      camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
      camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
      preview_input_port  = state.preview_parameters.preview_component->input[0];
      encoder_input_port  = state.encoder_component->input[0];
      encoder_output_port = state.encoder_component->output[0];

	if(state.use_resizer==2) {
		resizer_input_port = state.resizer_component->input[0];
		resizer_output_port = state.resizer_component->output[RESIZER_OUTPUT_PORT];

		//printf("%x	-> %x\n", resizer_input_port, resizer_output_port);

	} else if (state.raw_output) {
		splitter_input_port = state.splitter_component->input[0];
		splitter_output_port = state.splitter_component->output[SPLITTER_OUTPUT_PORT];
		splitter_preview_port = state.splitter_component->output[SPLITTER_PREVIEW_PORT];
  	}

	

      if (state.preview_parameters.wantPreview )
      {
         if (state.raw_output)
         {
            if (state.common_settings.verbose)
               fprintf(stderr, "Connecting camera preview port to splitter input port\n");

            // Connect camera to splitter
	    status = connect_ports(camera_preview_port, splitter_input_port, &state.splitter_connection);

            if (status != MMAL_SUCCESS)
            {
               state.splitter_connection = NULL;
               vcos_log_error("%s: Failed to connect camera preview port to splitter input", __func__);
               goto error;
            }

            if (state.common_settings.verbose)
            {
               fprintf(stderr, "Connecting splitter preview port to preview input port\n");
               fprintf(stderr, "Starting video preview\n");
            }

            // Connect splitter to preview
            status = connect_ports(splitter_preview_port, preview_input_port, &state.preview_connection);

	    // Connect splitter to resizer
	    //status = connect_ports(splitter_output_port, resizer_input_port, &state.resizer_connection);
         }
         else
         {
            if (state.common_settings.verbose)
            {
               fprintf(stderr, "Connecting camera preview port to preview input port\n");
               fprintf(stderr, "Starting video preview\n");
            }

            // Connect camera to preview
            status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);
         }

         if (status != MMAL_SUCCESS)
            state.preview_connection = NULL;
      }
      else
      {
         if (state.raw_output)
         {
            if (state.common_settings.verbose)
               fprintf(stderr, "Connecting camera preview port to splitter input port\n");

            // Connect camera to splitter
		if(state.use_resizer==2) 
			status = connect_ports(camera_preview_port, resizer_input_port, &state.resizer_connection);
		else
			status = connect_ports(camera_preview_port, splitter_input_port, &state.splitter_connection);

            if (status != MMAL_SUCCESS)
            {
               state.splitter_connection = NULL;
               vcos_log_error("%s: Failed to connect camera preview port to splitter input", __func__);
               goto error;
            }
         }
         else
         {
            status = MMAL_SUCCESS;
         }
      }

      if (status == MMAL_SUCCESS)
      {
         if (state.common_settings.verbose)
            fprintf(stderr, "Connecting camera video port to encoder input port\n");

         // Now connect the camera to the encoder
         status = connect_ports(camera_video_port, encoder_input_port, &state.encoder_connection);

         if (status != MMAL_SUCCESS)
         {
            state.encoder_connection = NULL;
            vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
            goto error;
         }
      }

      if (status == MMAL_SUCCESS)
      {
         // Set up our userdata - this is passed though to the callback where we need the information.
         state.callback_data.pstate = &state;
         state.callback_data.abort = 0;

	if(state.use_resizer==2) {

		resizer_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

		if (state.common_settings.verbose)
			fprintf(stderr, "Enabling resizer output port\n");

		// Enable the splitter output port and tell it its callback function
		status = mmal_port_enable(resizer_output_port, resizer_buffer_callback);

		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("%s: Failed to setup splitter output port", __func__);
			goto error;
		}

	} else if (state.raw_output) {

            splitter_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

            if (state.common_settings.verbose)
               fprintf(stderr, "Enabling splitter output port\n");

            // Enable the splitter output port and tell it its callback function
            status = mmal_port_enable(splitter_output_port, splitter_buffer_callback);

            if (status != MMAL_SUCCESS)
            {
               vcos_log_error("%s: Failed to setup splitter output port", __func__);
               goto error;
            } 
         }

         state.callback_data.file_handle = NULL;

         if (state.common_settings.filename)
         {
            if (state.common_settings.filename[0] == '-')
            {
               state.callback_data.file_handle = stdout;
            }
            else
            {
		state.file_open_delegated = state.mp4_video;
               state.callback_data.file_handle = open_filename(&state, state.common_settings.filename);
            }

            if (!state.callback_data.file_handle)
            {
               // Notify user, carry on but discarding encoded output buffers
               vcos_log_error("%s: Error opening output file: %s\nNo output file will be generated\n", __func__, state.common_settings.filename);

            } else if(state.mp4_video) {

		if(!state.audio_card) {

		} else if(!state.mp4w->alsa_open(state.audio_card)) {	
			vcos_log_error("%s: AlsaMic: alsa_open failed", __func__);

		} else if(!state.mp4w->alsa_init()) {	
			vcos_log_error("%s: AlsaMic: alsa_init failed", __func__);
		}
	    }
         }

         state.callback_data.imv_file_handle = NULL;

         if (state.imv_filename)
         {
            if (state.imv_filename[0] == '-')
            {
               state.callback_data.imv_file_handle = stdout;
            }
            else
            {
               state.callback_data.imv_file_handle = open_filename(&state, state.imv_filename);
            }

            if (!state.callback_data.imv_file_handle)
            {
               // Notify user, carry on but discarding encoded output buffers
               fprintf(stderr, "Error opening output file: %s\nNo output file will be generated\n",state.imv_filename);
               state.inlineMotionVectors=0;
            }
         }

         state.callback_data.pts_file_handle = NULL;

         if (state.pts_filename)
         {
            if (state.pts_filename[0] == '-')
            {
               state.callback_data.pts_file_handle = stdout;
            }
            else
            {
               state.callback_data.pts_file_handle = open_filename(&state, state.pts_filename);
               if (state.callback_data.pts_file_handle) /* save header for mkvmerge */
                  fprintf(state.callback_data.pts_file_handle, "# timecode format v2\n");
            }

            if (!state.callback_data.pts_file_handle)
            {
               // Notify user, carry on but discarding encoded output buffers
               fprintf(stderr, "Error opening output file: %s\nNo output file will be generated\n",state.pts_filename);
               state.save_pts=0;
            }
         }

         state.callback_data.raw_file_handle = NULL;

         if (state.raw_filename)
         {
            if (state.raw_filename[0] == '-')
            {
               state.callback_data.raw_file_handle = stdout;
            }
            else
            {
               //state.callback_data.raw_file_handle = open_filename(&state, state.raw_filename);
            }

            /*if (!state.callback_data.raw_file_handle)
            {
               // Notify user, carry on but discarding encoded output buffers
               fprintf(stderr, "Error opening output file: %s\nNo output file will be generated\n", state.raw_filename);
               state.raw_output = 0;
            }*/
         }

         if(state.bCircularBuffer)
         {
            if(state.bitrate == 0)
            {
               vcos_log_error("%s: Error circular buffer requires constant bitrate and small intra period\n", __func__);
               goto error;
            }
            else if(state.timeout == 0)
            {
               vcos_log_error("%s: Error, circular buffer size is based on timeout must be greater than zero\n", __func__);
               goto error;
            }
            else if(state.waitMethod != WAIT_METHOD_KEYPRESS && state.waitMethod != WAIT_METHOD_SIGNAL)
            {
               vcos_log_error("%s: Error, Circular buffer mode requires either keypress (-k) or signal (-s) triggering\n", __func__);
               goto error;
            }
            else if(!state.callback_data.file_handle)
            {
               vcos_log_error("%s: Error require output file (or stdout) for Circular buffer mode\n", __func__);
               goto error;
            }
            else
            {
               int count = state.bitrate * (state.timeout / 1000) / 8;

               state.callback_data.cb_buff = (char *) malloc(count);
               if(state.callback_data.cb_buff == NULL)
               {
                  vcos_log_error("%s: Unable to allocate circular buffer for %d seconds at %.1f Mbits\n", __func__, state.timeout / 1000, (double)state.bitrate/1000000.0);
                  goto error;
               }
               else
               {
                  state.callback_data.cb_len = count;
                  state.callback_data.cb_wptr = 0;
                  state.callback_data.cb_wrap = 0;
                  state.callback_data.cb_data = 0;
                  state.callback_data.iframe_buff_wpos = 0;
                  state.callback_data.iframe_buff_rpos = 0;
                  state.callback_data.header_wptr = 0;
               }
            }
         }

         // Set up our userdata - this is passed though to the callback where we need the information.
         encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

         if (state.common_settings.verbose)
            fprintf(stderr, "Enabling encoder output port\n");

         // Enable the encoder output port and tell it its callback function
         status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Failed to setup encoder output");
            goto error;
         }

         if (state.demoMode)
         {
            // Run for the user specific time..
            int num_iterations = state.timeout / state.demoInterval;
            int i;

            if (state.common_settings.verbose)
               fprintf(stderr, "Running in demo mode\n");

            for (i=0; state.timeout == 0 || i<num_iterations; i++)
            {
               raspicamcontrol_cycle_test(state.camera_component);
               vcos_sleep(state.demoInterval);
            }
         }
         else
         {
            // Only encode stuff if we have a filename and it opened
            // Note we use the copy in the callback, as the call back MIGHT change the file handle
            //if (state.callback_data.file_handle || state.callback_data.raw_file_handle)
            if (state.callback_data.file_handle || state.raw_filename)
            {
               state.running = 1;

               // Send all the buffers to the encoder output port
               if (state.callback_data.file_handle)
               {
                  int num = mmal_queue_length(state.encoder_pool->queue);
                  int q;
                  for (q=0; q<num; q++)
                  {
                     MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

                     if (!buffer)
                        vcos_log_error("Unable to get a required buffer %d from pool queue", q);

                     if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
                        vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
                  }
               }

               // Send all the buffers to the splitter output port
               //if (state.callback_data.raw_file_handle)
		if(state.use_resizer == 2) {

	          int num = mmal_queue_length(state.resizer_pool->queue);
                  int q;
                  for (q = 0; q < num; q++)
                  {
                     MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.resizer_pool->queue);

                     if (!buffer)
                        vcos_log_error("Unable to get a required buffer %d from pool queue", q);

                     if (mmal_port_send_buffer(resizer_output_port, buffer)!= MMAL_SUCCESS)
                        vcos_log_error("Unable to send a buffer to resizer output port (%d)", q);
                  }
			
               } else if (state.raw_filename)
               {
                  int num = mmal_queue_length(state.splitter_pool->queue);
                  int q;
                  for (q = 0; q < num; q++)
                  {
                     MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.splitter_pool->queue);

                     if (!buffer)
                        vcos_log_error("Unable to get a required buffer %d from pool queue", q);

                     if (mmal_port_send_buffer(splitter_output_port, buffer)!= MMAL_SUCCESS)
                        vcos_log_error("Unable to send a buffer to splitter output port (%d)", q);
                  }
               }

		//state.disp = new rp::DispManX();
		//state.disp->init(0, 0);	

		pthread_t thread_id = 0;
               int initialCapturing=state.bCapturing;
               while (state.running)
               {
                  // Change state

                  state.bCapturing = !state.bCapturing;

                  if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, state.bCapturing) != MMAL_SUCCESS)
                  {
                     // How to handle?
                  }

                  if(thread_id == 0) {
			//pthread_create(&thread_id, NULL, opencv_thread, &state);
		  }

                  // In circular buffer mode, exit and save the buffer (make sure we do this after having paused the capture
                  if(state.bCircularBuffer && !state.bCapturing)
                  {
                     break;
                  }

                  if (state.common_settings.verbose)
                  {
                     if (state.bCapturing)
                        fprintf(stderr, "Starting video capture\n");
                     else
                        fprintf(stderr, "Pausing video capture\n");
                  }

                  if(state.splitWait)
                  {
                     if(state.bCapturing)
                     {
                        if (mmal_port_parameter_set_boolean(encoder_output_port, MMAL_PARAMETER_VIDEO_REQUEST_I_FRAME, 1) != MMAL_SUCCESS)
                        {
                           vcos_log_error("failed to request I-FRAME");
                        }
                     }
                     else
                     {
                        if(!initialCapturing)
                           state.splitNow=1;
                     }
                     initialCapturing=0;
                  }

                  state.running = wait_for_next_change(&state);
               }

		vcos_semaphore_post(&state.frame_complete);
		pthread_join(thread_id, NULL);

               if (state.common_settings.verbose)
                  fprintf(stderr, "Finished capture\n");
            }
            else
            {
               if (state.timeout)
                  vcos_sleep(state.timeout);
               else
               {
                  // timeout = 0 so run forever
                  while(1)
                     vcos_sleep(ABORT_INTERVAL);
               }
            }
         }
      }
      else
      {
         mmal_status_to_int(status);
         vcos_log_error("%s: Failed to connect camera to preview", __func__);
      }

      if(state.bCircularBuffer)
      {
         int copy_from_end, copy_from_start;

         copy_from_end = state.callback_data.cb_len - state.callback_data.iframe_buff[state.callback_data.iframe_buff_rpos];
         copy_from_start = state.callback_data.cb_len - copy_from_end;
         copy_from_start = state.callback_data.cb_wptr < copy_from_start ? state.callback_data.cb_wptr : copy_from_start;
         if(!state.callback_data.cb_wrap)
         {
            copy_from_start = state.callback_data.cb_wptr;
            copy_from_end = 0;
         }

         fwrite(state.callback_data.header_bytes, 1, state.callback_data.header_wptr, state.callback_data.file_handle);
         // Save circular buffer
         fwrite(state.callback_data.cb_buff + state.callback_data.iframe_buff[state.callback_data.iframe_buff_rpos], 1, copy_from_end, state.callback_data.file_handle);
         fwrite(state.callback_data.cb_buff, 1, copy_from_start, state.callback_data.file_handle);
         if(state.callback_data.flush_buffers) fflush(state.callback_data.file_handle);
      }

error:
	state.con.notify_one();
      mmal_status_to_int(status);

      if (state.common_settings.verbose)
         fprintf(stderr, "Closing down\n");

	//mp4w.finish();

      // Disable all our ports that are not handled by connections
      check_disable_port(camera_still_port);
      check_disable_port(encoder_output_port);
      check_disable_port(resizer_output_port);
      check_disable_port(splitter_output_port);
      //

      if (state.preview_parameters.wantPreview && state.preview_connection)
         mmal_connection_destroy(state.preview_connection);

      if (state.resizer_connection)
         mmal_connection_destroy(state.resizer_connection);

      if (state.encoder_connection)
         mmal_connection_destroy(state.encoder_connection);

      if (state.splitter_connection)
         mmal_connection_destroy(state.splitter_connection);

      // Can now close our file. Note disabling ports may flush buffers which causes
      // problems if we have already closed the file!

	
	if (!state.callback_data.file_handle) {

	} else if(state.mp4_video) {
		state.mp4w->endMovie();
	} else if(state.callback_data.file_handle != stdout) {
		fclose(state.callback_data.file_handle);
	}

      if (state.callback_data.imv_file_handle && state.callback_data.imv_file_handle != stdout)
         fclose(state.callback_data.imv_file_handle);
      if (state.callback_data.pts_file_handle && state.callback_data.pts_file_handle != stdout)
         fclose(state.callback_data.pts_file_handle);
      if (state.callback_data.raw_file_handle && state.callback_data.raw_file_handle != stdout)
         fclose(state.callback_data.raw_file_handle);

      /* Disable components */
      if (state.encoder_component)
         mmal_component_disable(state.encoder_component);

      if (state.preview_parameters.preview_component)
         mmal_component_disable(state.preview_parameters.preview_component);

      if (state.resizer_component)
         mmal_component_disable(state.resizer_component);

      if (state.splitter_component)
         mmal_component_disable(state.splitter_component);

      if (state.camera_component)
         mmal_component_disable(state.camera_component);

      destroy_encoder_component(&state);
      raspipreview_destroy(&state.preview_parameters);
      destroy_resizer_component(&state);	
      destroy_splitter_component(&state);
      destroy_camera_component(&state);

      if (state.common_settings.verbose)
         fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
   }

   if (status != MMAL_SUCCESS)
      raspicamcontrol_check_configuration(128);

   if (state.common_settings.gps)
      raspi_gps_shutdown(state.common_settings.verbose);

   return exit_code;
}
