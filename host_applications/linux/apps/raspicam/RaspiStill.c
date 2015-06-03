/*
Copyright (c) 2013, Broadcom Europe Ltd
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
 * \file RaspiStill.c
 * Command line program to capture a still frame and encode it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * \date 31 Jan 2013
 * \Author: James Hughes
 *
 * Description
 *
 * 3 components are created; camera, preview and JPG encoder.
 * Camera component has three ports, preview, video and stills.
 * This program connects preview and stills to the preview and jpg
 * encoder. Using mmal we don't need to worry about buffers between these
 * components, but we do need to handle buffers from the encoder, which
 * are simply written straight to the file in the requisite buffer callback.
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 */

// We use some GNU extensions (asprintf, basename)
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sysexits.h>

#define VERSION_STRING "v1.3.8"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"


#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiTex.h"
#include "minmea.h"

#include <semaphore.h>
#include <pthread.h>
#include <wiringSerial.h>
// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2


// Stills format information
// 0 implies variable
#define STILLS_FRAME_RATE_NUM 0
#define STILLS_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

#define MAX_USER_EXIF_TAGS      32
#define MAX_EXIF_PAYLOAD_LENGTH 128

/// Frame advance method
#define FRAME_NEXT_SINGLE        0
#define FRAME_NEXT_TIMELAPSE     1
#define FRAME_NEXT_KEYPRESS      2
#define FRAME_NEXT_FOREVER       3
#define FRAME_NEXT_GPIO          4
#define FRAME_NEXT_SIGNAL        5
#define FRAME_NEXT_IMMEDIATELY   6


int mmal_status_to_int(MMAL_STATUS_T status);
static void signal_handler(int signal_number);
enum COORDTYPE
{ 
   north,
   east,
   west,
   south
};

struct coordinate
{
   COORDTYPE ref;
   int deg;
   int min_scaled;
   int min_scale;   
}

struct gps_info
{
   int serial;
   coordinate latitude;
   coordinate longitude;
   struct minmea_float speed;
   struct minmea_float course;
   struct minmea_float altitude;
} nav_data;

/** Structure containing all state information for the current run
 */
typedef struct
{
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   int width;                          /// Requested width of image
   int height;                         /// requested height of image
   int quality;                        /// JPEG quality setting (1-100)
   int wantRAW;                        /// Flag for whether the JPEG metadata also contains the RAW bayer image
   char *filename;                     /// filename of output file
   char *linkname;                     /// filename of output file
   MMAL_PARAM_THUMBNAIL_CONFIG_T thumbnailConfig;
   int verbose;                        /// !0 if want detailed run information
   int demoMode;                       /// Run app in demo mode
   int demoInterval;                   /// Interval between camera settings changes
   MMAL_FOURCC_T encoding;             /// Encoding to use for the output file.
   const char *exifTags[MAX_USER_EXIF_TAGS]; /// Array of pointers to tags supplied from the command line
   int numExifTags;                    /// Number of supplied tags
   int enableExifTags;                 /// Enable/Disable EXIF tags in output
   int timelapse;                      /// Delay between each picture in timelapse mode. If 0, disable timelapse
   int fullResPreview;                 /// If set, the camera preview port runs at capture resolution. Reduces fps.
   int frameNextMethod;                /// Which method to use to advance to next frame
   int useGL;                          /// Render preview using OpenGL
   int glCapture;                      /// Save the GL frame-buffer instead of camera output
   int settings;                       /// Request settings from the camera
   int cameraNum;                      /// Camera number
   int burstCaptureMode;               /// Enable burst mode
   int sensor_mode;                     /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.
   int datetime;                       /// Use DateTime instead of frame#
   int timestamp;                      /// Use timestamp instead of frame#

   RASPIPREVIEW_PARAMETERS preview_parameters;    /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_COMPONENT_T *null_sink_component; /// Pointer to the null sink component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

   RASPITEX_STATE raspitex_state; /// GL renderer state and parameters

} RASPISTILL_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
   RASPISTILL_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

static void display_valid_parameters(char *app_name);
static void store_exif_tag(RASPISTILL_STATE *state, const char *exif_tag);

/// Comamnd ID's and Structure defining our command line options
#define CommandHelp         0
#define CommandWidth        1
#define CommandHeight       2
#define CommandQuality      3
#define CommandRaw          4
#define CommandOutput       5
#define CommandVerbose      6
#define CommandTimeout      7
#define CommandThumbnail    8
#define CommandDemoMode     9
#define CommandEncoding     10
#define CommandExifTag      11
#define CommandTimelapse    12
#define CommandFullResPreview 13
#define CommandLink         14
#define CommandKeypress     15
#define CommandSignal       16
#define CommandGL           17
#define CommandGLCapture    18
#define CommandSettings     19
#define CommandCamSelect    20
#define CommandBurstMode    21
#define CommandSensorMode   22
#define CommandDateTime     23
#define CommandTimeStamp    24

static COMMAND_LIST cmdline_commands[] =
{
   { CommandHelp,    "-help",       "?",  "This help information", 0 },
   { CommandWidth,   "-width",      "w",  "Set image width <size>", 1 },
   { CommandHeight,  "-height",     "h",  "Set image height <size>", 1 },
   { CommandQuality, "-quality",    "q",  "Set jpeg quality <0 to 100>", 1 },
   { CommandRaw,     "-raw",        "r",  "Add raw bayer data to jpeg metadata", 0 },
   { CommandOutput,  "-output",     "o",  "Output filename <filename> (to write to stdout, use '-o -'). If not specified, no file is saved", 1 },
   { CommandLink,    "-latest",     "l",  "Link latest complete image to filename <filename>", 1},
   { CommandVerbose, "-verbose",    "v",  "Output verbose information during run", 0 },
   { CommandTimeout, "-timeout",    "t",  "Time (in ms) before takes picture and shuts down (if not specified, set to 5s)", 1 },
   { CommandThumbnail,"-thumb",     "th", "Set thumbnail parameters (x:y:quality) or none", 1},
   { CommandDemoMode,"-demo",       "d",  "Run a demo mode (cycle through range of camera options, no capture)", 0},
   { CommandEncoding,"-encoding",   "e",  "Encoding to use for output file (jpg, bmp, gif, png)", 1},
   { CommandExifTag, "-exif",       "x",  "EXIF tag to apply to captures (format as 'key=value') or none", 1},
   { CommandTimelapse,"-timelapse", "tl", "Timelapse mode. Takes a picture every <t>ms", 1},
   { CommandFullResPreview,"-fullpreview","fp", "Run the preview using the still capture resolution (may reduce preview fps)", 0},
   { CommandKeypress,"-keypress",   "k",  "Wait between captures for a ENTER, X then ENTER to exit", 0},
   { CommandSignal,  "-signal",     "s",  "Wait between captures for a SIGUSR1 from another process", 0},
   { CommandGL,      "-gl",         "g",  "Draw preview to texture instead of using video render component", 0},
   { CommandGLCapture, "-glcapture","gc", "Capture the GL frame-buffer instead of the camera image", 0},
   { CommandSettings, "-settings",  "set","Retrieve camera settings and write to stdout", 0},
   { CommandCamSelect, "-camselect","cs", "Select camera <number>. Default 0", 1 },
   { CommandBurstMode, "-burst",    "bm", "Enable 'burst capture mode'", 0},
   { CommandSensorMode,"-mode",     "md", "Force sensor mode. 0=auto. See docs for other modes available", 1},
   { CommandDateTime,  "-datetime",  "dt", "Replace frame number in file name with DateTime (YearMonthDayHourMinSec)", 0},
   { CommandTimeStamp, "-timestamp", "ts", "Replace frame number in file name with unix timestamp (seconds since 1900)", 0},
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);

static struct
{
   char *format;
   MMAL_FOURCC_T encoding;
} encoding_xref[] =
{
   {"jpg", MMAL_ENCODING_JPEG},
   {"bmp", MMAL_ENCODING_BMP},
   {"gif", MMAL_ENCODING_GIF},
   {"png", MMAL_ENCODING_PNG}
};

static int encoding_xref_size = sizeof(encoding_xref) / sizeof(encoding_xref[0]);


static struct
{
   char *description;
   int nextFrameMethod;
} next_frame_description[] =
{
      {"Single capture",         FRAME_NEXT_SINGLE},
      {"Capture on timelapse",   FRAME_NEXT_TIMELAPSE},
      {"Capture on keypress",    FRAME_NEXT_KEYPRESS},
      {"Run forever",            FRAME_NEXT_FOREVER},
      {"Capture on GPIO",        FRAME_NEXT_GPIO},
      {"Capture on signal",      FRAME_NEXT_SIGNAL},
};

static int next_frame_description_size = sizeof(next_frame_description) / sizeof(next_frame_description[0]);

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPISTILL_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   state->timeout = 5000; // 5s delay before take image
   state->width = 2592;
   state->height = 1944;
   state->quality = 85;
   state->wantRAW = 0;
   state->filename = NULL;
   state->linkname = NULL;
   state->verbose = 0;
   state->thumbnailConfig.enable = 1;
   state->thumbnailConfig.width = 64;
   state->thumbnailConfig.height = 48;
   state->thumbnailConfig.quality = 35;
   state->demoMode = 0;
   state->demoInterval = 250; // ms
   state->camera_component = NULL;
   state->encoder_component = NULL;
   state->preview_connection = NULL;
   state->encoder_connection = NULL;
   state->encoder_pool = NULL;
   state->encoding = MMAL_ENCODING_JPEG;
   state->numExifTags = 0;
   state->enableExifTags = 1;
   state->timelapse = 0;
   state->fullResPreview = 0;
   state->frameNextMethod = FRAME_NEXT_SINGLE;
   state->useGL = 0;
   state->glCapture = 0;
   state->settings = 0;
   state->cameraNum = 0;
   state->burstCaptureMode=0;
   state->sensor_mode = 0;
   state->datetime = 0;
   state->timestamp = 0;

   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);

   // Set initial GL preview state
   raspitex_set_defaults(&state->raspitex_state);
}

/**
 * Dump image state parameters to stderr. Used for debugging
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPISTILL_STATE *state)
{
   int i;

   if (!state)
   {
      vcos_assert(0);
      return;
   }

   fprintf(stderr, "Width %d, Height %d, quality %d, filename %s\n", state->width,
         state->height, state->quality, state->filename);
   fprintf(stderr, "Time delay %d, Raw %s\n", state->timeout,
         state->wantRAW ? "yes" : "no");
   fprintf(stderr, "Thumbnail enabled %s, width %d, height %d, quality %d\n",
         state->thumbnailConfig.enable ? "Yes":"No", state->thumbnailConfig.width,
         state->thumbnailConfig.height, state->thumbnailConfig.quality);
   fprintf(stderr, "Link to latest frame enabled ");
   if (state->linkname)
   {
      fprintf(stderr, " yes, -> %s\n", state->linkname);
   }
   else
   {
      fprintf(stderr, " no\n");
   }
   fprintf(stderr, "Full resolution preview %s\n", state->fullResPreview ? "Yes": "No");

   fprintf(stderr, "Capture method : ");
   for (i=0;i<next_frame_description_size;i++)
   {
      if (state->frameNextMethod == next_frame_description[i].nextFrameMethod)
         fprintf(stderr, next_frame_description[i].description);
   }
   fprintf(stderr, "\n\n");

   if (state->enableExifTags)
   {
	   if (state->numExifTags)
	   {
		  fprintf(stderr, "User supplied EXIF tags :\n");

		  for (i=0;i<state->numExifTags;i++)
		  {
			 fprintf(stderr, "%s", state->exifTags[i]);
			 if (i != state->numExifTags-1)
				fprintf(stderr, ",");
		  }
		  fprintf(stderr, "\n\n");
	   }
   }
   else
      fprintf(stderr, "EXIF tags disabled\n");

   raspipreview_dump_parameters(&state->preview_parameters);
   raspicamcontrol_dump_parameters(&state->camera_parameters);
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters to
 * @return non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, const char **argv, RASPISTILL_STATE *state)
{
   // Parse the command line arguments.
   // We are looking for --<something> or -<abreviation of something>

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

      // If we found a command but are missing a parameter, continue (and we will drop out of the loop)
      if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc) )
         continue;

      //  We are now dealing with a command line option
      switch (command_id)
      {
      case CommandHelp:
         display_valid_parameters(basename(argv[0]));
         // exit straight away if help requested
         return -1;

      case CommandWidth: // Width > 0
         if (sscanf(argv[i + 1], "%u", &state->width) != 1)
            valid = 0;
         else
            i++;
         break;

      case CommandHeight: // Height > 0
         if (sscanf(argv[i + 1], "%u", &state->height) != 1)
            valid = 0;
         else
            i++;
         break;

      case CommandQuality: // Quality = 1-100
         if (sscanf(argv[i + 1], "%u", &state->quality) == 1)
         {
            if (state->quality > 100)
            {
               fprintf(stderr, "Setting max quality = 100\n");
               state->quality = 100;
            }
            i++;
         }
         else
            valid = 0;

         break;

      case CommandRaw: // Add raw bayer data in metadata
         state->wantRAW = 1;
         break;

      case CommandOutput:  // output filename
      {
         int len = strlen(argv[i + 1]);
         if (len)
         {
            state->filename = malloc(len + 10); // leave enough space for any timelapse generated changes to filename
            vcos_assert(state->filename);
            if (state->filename)
               strncpy(state->filename, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandLink :
      {
         int len = strlen(argv[i+1]);
         if (len)
         {
            state->linkname = malloc(len + 10);
            vcos_assert(state->linkname);
            if (state->linkname)
               strncpy(state->linkname, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;

      }
      case CommandVerbose: // display lots of data during run
         state->verbose = 1;
         break;
      case CommandDateTime: // use datetime
         state->datetime = 1;
         break;
      case CommandTimeStamp: // use timestamp
         state->timestamp = 1;
         break;

      case CommandTimeout: // Time to run viewfinder for before taking picture, in seconds
      {
         if (sscanf(argv[i + 1], "%u", &state->timeout) == 1)
         {
            // Ensure that if previously selected CommandKeypress we don't overwrite it
            if (state->timeout == 0 && state->frameNextMethod == FRAME_NEXT_SINGLE)
               state->frameNextMethod = FRAME_NEXT_FOREVER;

            i++;
         }
         else
            valid = 0;
         break;
      }
      case CommandThumbnail : // thumbnail parameters - needs string "x:y:quality"
         if ( strcmp( argv[ i + 1 ], "none" ) == 0 )
         {
            state->thumbnailConfig.enable = 0;
         }
         else
         {
            sscanf(argv[i + 1], "%d:%d:%d",
                   &state->thumbnailConfig.width,
                   &state->thumbnailConfig.height,
                   &state->thumbnailConfig.quality);
         }
         i++;
         break;

      case CommandDemoMode: // Run in demo mode - no capture
      {
         // Demo mode might have a timing parameter
         // so check if a) we have another parameter, b) its not the start of the next option
         if (i + 1 < argc  && argv[i+1][0] != '-')
         {
            if (sscanf(argv[i + 1], "%u", &state->demoInterval) == 1)
            {
               // TODO : What limits do we need for timeout?
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

      case CommandEncoding :
      {
         int len = strlen(argv[i + 1]);
         valid = 0;

         if (len)
         {
            int j;
            for (j=0;j<encoding_xref_size;j++)
            {
               if (strcmp(encoding_xref[j].format, argv[i+1]) == 0)
               {
                  state->encoding = encoding_xref[j].encoding;
                  valid = 1;
                  i++;
                  break;
               }
            }
         }
         break;
      }

      case CommandExifTag:
         if ( strcmp( argv[ i + 1 ], "none" ) == 0 )
         {
            state->enableExifTags = 0;
         }
         else
         {
            store_exif_tag(state, argv[i+1]);
         }
         i++;
         break;

      case CommandTimelapse:
         if (sscanf(argv[i + 1], "%u", &state->timelapse) != 1)
            valid = 0;
         else
         {
            if (state->timelapse)
               state->frameNextMethod = FRAME_NEXT_TIMELAPSE;
            else
               state->frameNextMethod = FRAME_NEXT_IMMEDIATELY;


            i++;
         }
         break;

      case CommandFullResPreview:
         state->fullResPreview = 1;
         break;

      case CommandKeypress: // Set keypress between capture mode
         state->frameNextMethod = FRAME_NEXT_KEYPRESS;
         break;

      case CommandSignal:   // Set SIGUSR1 between capture mode
         state->frameNextMethod = FRAME_NEXT_SIGNAL;
         // Reenable the signal
         signal(SIGUSR1, signal_handler);
         break;

      case CommandGL:
         state->useGL = 1;
         break;

      case CommandGLCapture:
         state->glCapture = 1;
         break;

      case CommandSettings:
         state->settings = 1;
         break;

         
      case CommandCamSelect:  //Select camera input port
      {
         if (sscanf(argv[i + 1], "%u", &state->cameraNum) == 1)
         {
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandBurstMode: 
         state->burstCaptureMode=1;
         break;

      case CommandSensorMode:
      {
         if (sscanf(argv[i + 1], "%u", &state->sensor_mode) == 1)
         {
            i++;
         }
         else
            valid = 0;
         break;
      }


      default:
      {
         // Try parsing for any image specific parameters
         // result indicates how many parameters were used up, 0,1,2
         // but we adjust by -1 as we have used one already
         const char *second_arg = (i + 1 < argc) ? argv[i + 1] : NULL;
         int parms_used = raspicamcontrol_parse_cmdline(&state->camera_parameters, &argv[i][1], second_arg);

         // Still unused, try preview options
         if (!parms_used)
            parms_used = raspipreview_parse_cmdline(&state->preview_parameters, &argv[i][1], second_arg);

         // Still unused, try GL preview options
         if (!parms_used)
            parms_used = raspitex_parse_cmdline(&state->raspitex_state, &argv[i][1], second_arg);

         // If no parms were used, this must be a bad parameters
         if (!parms_used)
            valid = 0;
         else
            i += parms_used - 1;

         break;
      }
      }
   }

   /* GL preview parameters use preview parameters as defaults unless overriden */
   if (! state->raspitex_state.gl_win_defined)
   {
      state->raspitex_state.x       = state->preview_parameters.previewWindow.x;
      state->raspitex_state.y       = state->preview_parameters.previewWindow.y;
      state->raspitex_state.width   = state->preview_parameters.previewWindow.width;
      state->raspitex_state.height  = state->preview_parameters.previewWindow.height;
   }
   /* Also pass the preview information through so GL renderer can determine
    * the real resolution of the multi-media image */
   state->raspitex_state.preview_x       = state->preview_parameters.previewWindow.x;
   state->raspitex_state.preview_y       = state->preview_parameters.previewWindow.y;
   state->raspitex_state.preview_width   = state->preview_parameters.previewWindow.width;
   state->raspitex_state.preview_height  = state->preview_parameters.previewWindow.height;
   state->raspitex_state.opacity         = state->preview_parameters.opacity;
   state->raspitex_state.verbose         = state->verbose;

   if (!valid)
   {
      fprintf(stderr, "Invalid command line option (%s)\n", argv[i-1]);
      return 1;
   }

   return 0;
}

/**
 * Display usage information for the application to stdout
 *
 * @param app_name String to display as the application name
 */
static void display_valid_parameters(char *app_name)
{
   fprintf(stderr, "Runs camera for specific time, and take JPG capture at end if requested\n\n");
   fprintf(stderr, "usage: %s [options]\n\n", app_name);

   fprintf(stderr, "Image parameter commands\n\n");

   raspicli_display_help(cmdline_commands, cmdline_commands_size);

   // Help for preview options
   raspipreview_display_help();

   // Now display any help information from the camcontrol code
   raspicamcontrol_display_help();

   // Now display GL preview help
   raspitex_display_help();

   fprintf(stderr, "\n");

   return;
}

/**
 *  buffer header callback function for camera control
 *
 *  No actions taken in current version
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
   {
      MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
      switch (param->hdr.id) {
         case MMAL_PARAMETER_CAMERA_SETTINGS:
         {
            MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
            vcos_log_error("Exposure now %u, analog gain %u/%u, digital gain %u/%u",
			settings->exposure,
                        settings->analog_gain.num, settings->analog_gain.den,
                        settings->digital_gain.num, settings->digital_gain.den);
            vcos_log_error("AWB R=%u/%u, B=%u/%u",
                        settings->awb_red_gain.num, settings->awb_red_gain.den,
                        settings->awb_blue_gain.num, settings->awb_blue_gain.den
                        );
         }
         break;
      }
   }
   else if (buffer->cmd == MMAL_EVENT_ERROR)
   {
      vcos_log_error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
   }
   else
      vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);

   mmal_buffer_header_release(buffer);
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
   int complete = 0;

   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

   if (pData)
   {
      int bytes_written = buffer->length;

      if (buffer->length && pData->file_handle)
      {
         mmal_buffer_header_mem_lock(buffer);

         bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);

         mmal_buffer_header_mem_unlock(buffer);
      }

      // We need to check we wrote what we wanted - it's possible we have run out of storage.
      if (bytes_written != buffer->length)
      {
         vcos_log_error("Unable to write buffer to file - aborting");
         complete = 1;
      }

      // Now flag if we have completed
      if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
         complete = 1;
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
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

      if (new_buffer)
      {
         status = mmal_port_send_buffer(port, new_buffer);
      }
      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }

   if (complete)
      vcos_semaphore_post(&(pData->complete_semaphore));
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct. camera_component member set to the created camera_component if successfull.
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPISTILL_STATE *state)
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
      goto error;
   }

   status = raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
   status += raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
   status += raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set stereo mode : error %d", status);
      goto error;
   }

   MMAL_PARAMETER_INT32_T camera_num =
      {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->cameraNum};

   status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", status);
      goto error;
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera doesn't have output ports");
      goto error;
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
      goto error;
   }

   preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

   if (state->settings)
   {
      MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
         {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T)},
          MMAL_PARAMETER_CAMERA_SETTINGS, 1};

      status = mmal_port_parameter_set(camera->control, &change_event_request.hdr);
      if ( status != MMAL_SUCCESS )
      {
         vcos_log_error("No camera settings events");
      }
   }

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, camera_control_callback);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable control port : error %d", status);
      goto error;
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->width,
         .max_stills_h = state->height,
         .stills_yuv422 = 0,
         .one_shot_stills = 1,
         .max_preview_video_w = state->preview_parameters.previewWindow.width,
         .max_preview_video_h = state->preview_parameters.previewWindow.height,
         .num_preview_video_frames = 3,
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 0,
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
      };

      if (state->fullResPreview)
      {
         cam_config.max_preview_video_w = state->width;
         cam_config.max_preview_video_h = state->height;
      }

      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }
 
   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   // Now set up the port formats

   format = preview_port->format;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 166, 1000 }, {999, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   if (state->fullResPreview)
   {
      // In this mode we are forcing the preview to be generated from the full capture resolution.
      // This runs at a max of 15fps with the OV5647 sensor.
      format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
      format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
      format->es->video.crop.x = 0;
      format->es->video.crop.y = 0;
      format->es->video.crop.width = state->width;
      format->es->video.crop.height = state->height;
      format->es->video.frame_rate.num = FULL_RES_PREVIEW_FRAME_RATE_NUM;
      format->es->video.frame_rate.den = FULL_RES_PREVIEW_FRAME_RATE_DEN;
   }
   else
   {
      // Use a full FOV 4:3 mode
      format->es->video.width = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.width, 32);
      format->es->video.height = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.height, 16);
      format->es->video.crop.x = 0;
      format->es->video.crop.y = 0;
      format->es->video.crop.width = state->preview_parameters.previewWindow.width;
      format->es->video.crop.height = state->preview_parameters.previewWindow.height;
      format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
      format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;
   }

   status = mmal_port_format_commit(preview_port);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera viewfinder format couldn't be set");
      goto error;
   }

   // Set the same format on the video  port (which we dont use here)
   mmal_format_full_copy(video_port->format, format);
   status = mmal_port_format_commit(video_port);

   if (status  != MMAL_SUCCESS)
   {
      vcos_log_error("camera video format couldn't be set");
      goto error;
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   format = still_port->format;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(still_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 167, 1000 }, {999, 1000}};
        mmal_port_parameter_set(still_port, &fps_range.hdr);
   }
   // Set our stills format on the stills (for encoder) port
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->width;
   format->es->video.crop.height = state->height;
   format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
   format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;


   status = mmal_port_format_commit(still_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }

   /* Ensure there are enough buffers to avoid dropping frames */
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera component couldn't be enabled");
      goto error;
   }

   if (state->useGL)
   {
      status = raspitex_configure_preview_port(&state->raspitex_state, preview_port);
      if (status != MMAL_SUCCESS)
      {
         fprintf(stderr, "Failed to configure preview port for GL rendering");
         goto error;
      }
   }

   state->camera_component = camera;

   if (state->verbose)
      fprintf(stderr, "Camera component done\n");

   return status;

error:

   if (camera)
      mmal_component_destroy(camera);

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPISTILL_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct. encoder_component member set to the created camera_component if successfull.
 *
 * @return a MMAL_STATUS, MMAL_SUCCESS if all OK, something else otherwise
 */
static MMAL_STATUS_T create_encoder_component(RASPISTILL_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create JPEG encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("JPEG encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Specify out output format
   encoder_output->format->encoding = state->encoding;

   encoder_output->buffer_size = encoder_output->buffer_size_recommended;

   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      goto error;
   }

   // Set the JPEG quality level
   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state->quality);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set JPEG quality");
      goto error;
   }

   // Set up any required thumbnail
   {
      MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb = {{MMAL_PARAMETER_THUMBNAIL_CONFIGURATION, sizeof(MMAL_PARAMETER_THUMBNAIL_CONFIG_T)}, 0, 0, 0, 0};

      if ( state->thumbnailConfig.enable &&
           state->thumbnailConfig.width > 0 && state->thumbnailConfig.height > 0 )
      {
         // Have a valid thumbnail defined
         param_thumb.enable = 1;
         param_thumb.width = state->thumbnailConfig.width;
         param_thumb.height = state->thumbnailConfig.height;
         param_thumb.quality = state->thumbnailConfig.quality;
      }
      status = mmal_port_parameter_set(encoder->control, &param_thumb.hdr);
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status  != MMAL_SUCCESS)
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

   if (state->verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;

   error:

   if (encoder)
      mmal_component_destroy(encoder);

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPISTILL_STATE *state)
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


/**
 * Add an exif tag to the capture
 *
 * @param state Pointer to state control struct
 * @param exif_tag String containing a "key=value" pair.
 * @return  Returns a MMAL_STATUS_T giving result of operation
 */
static MMAL_STATUS_T add_exif_tag(RASPISTILL_STATE *state, const char *exif_tag)
{
   MMAL_STATUS_T status;
   MMAL_PARAMETER_EXIF_T *exif_param = (MMAL_PARAMETER_EXIF_T*)calloc(sizeof(MMAL_PARAMETER_EXIF_T) + MAX_EXIF_PAYLOAD_LENGTH, 1);

   vcos_assert(state);
   vcos_assert(state->encoder_component);

   // Check to see if the tag is present or is indeed a key=value pair.
   if (!exif_tag || strchr(exif_tag, '=') == NULL || strlen(exif_tag) > MAX_EXIF_PAYLOAD_LENGTH-1)
      return MMAL_EINVAL;

   exif_param->hdr.id = MMAL_PARAMETER_EXIF;

   strncpy((char*)exif_param->data, exif_tag, MAX_EXIF_PAYLOAD_LENGTH-1);

   exif_param->hdr.size = sizeof(MMAL_PARAMETER_EXIF_T) + strlen((char*)exif_param->data);

   status = mmal_port_parameter_set(state->encoder_component->output[0], &exif_param->hdr);

   free(exif_param);

   return status;
}

/**
 * Add a basic set of EXIF tags to the capture
 * Make, Time etc
 *
 * @param state Pointer to state control struct
 *
 */
static void add_exif_tags(RASPISTILL_STATE *state)
{
   time_t rawtime;
   struct tm *timeinfo;
   char time_buf[32];
   char exif_buf[128];
   int i;

   add_exif_tag(state, "IFD0.Model=RP_OV5647");
   add_exif_tag(state, "IFD0.Make=RaspberryPi");

   time(&rawtime);
   timeinfo = localtime(&rawtime);

   snprintf(time_buf, sizeof(time_buf),
            "%04d:%02d:%02d %02d:%02d:%02d",
            timeinfo->tm_year+1900,
            timeinfo->tm_mon+1,
            timeinfo->tm_mday,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec);

   snprintf(exif_buf, sizeof(exif_buf), "EXIF.DateTimeDigitized=%s", time_buf);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf, sizeof(exif_buf), "EXIF.DateTimeOriginal=%s", time_buf);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf, sizeof(exif_buf), "IFD0.DateTime=%s", time_buf);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLatitude=%d/1,%d/%d,0/1000", nav_data.latitude.deg,nav_data.latitude.min_scaled,nav_data.latitude.min_scale);
   add_exif_tag(state, exif_buf);

   if (nav_data.latitude.ref == north)
      snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLatitudeRef=N");
   if (nav_data.latitude.ref == south)
      snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLatitudeRef=S");
   add_exif_tag(state, exif_buf);


   snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLongitude=%d/1,%d/%d,0/1000", nav_data.longitude.deg,nav_data.longitude.min_scaled,nav_data.longitude.min_scale);
   add_exif_tag(state, exif_buf);

   if (nav_data.longitude.ref == north)
      snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLongitudeRef=N");
   if (nav_data.longitude.ref == south)
      snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLongitudeRef=S");
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf,sizeof(exif_buf),"GPS.GPSAltitude=%d/%d",nav_data.altitude.value,nav_data.altitude.scale);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf,sizeof(exif_buf),"GPS.GPSSpeed=%d/%d",nav_data.speed.value,nav_data.speed.scale);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf,sizeof(exif_buf),"GPS.GPSTrack=%d/%d",nav_data.course.value,nav_data.course.scale);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf,sizeof(exif_buf),"GPS.GPSImgDirection=%d/%d",nav_data.course.value,nav_data.course.scale);
   add_exif_tag(state, exif_buf);




   // now send any user supplied tags

   for (i=0;i<state->numexiftags && i < max_user_exif_tags;i++)
   {
      if (state->exiftags[i])
      {
         add_exif_tag(state, state->exiftags[i]);
      }
   }
}

/**
 * stores an exif tag in the state, incrementing various pointers as necessary.
 * any tags stored in this way will be added to the image file when add_exif_tags
 * is called
 *
 * will not store if run out of storage space
 *
 * @param state pointer to state control struct
 * @param exif_tag exif tag string
 *
 */
static void store_exif_tag(raspistill_state *state, const char *exif_tag)
{
   if (state->numexiftags < max_user_exif_tags)
   {
      state->exiftags[state->numexiftags] = exif_tag;
      state->numexiftags++;
   }
}

/**
 * connect two specific ports together
 *
 * @param output_port pointer the output port
 * @param input_port pointer the input port
 * @param pointer to a mmal connection pointer, reassigned if function successful
 * @return returns a mmal_status_t giving result of operation
 *
 */
static mmal_status_t connect_ports(mmal_port_t *output_port, mmal_port_t *input_port, mmal_connection_t **connection)
{
   mmal_status_t status;

   status =  mmal_connection_create(connection, output_port, input_port, mmal_connection_flag_tunnelling | mmal_connection_flag_allocation_on_input);

   if (status == mmal_success)
   {
      status =  mmal_connection_enable(*connection);
      if (status != mmal_success)
         mmal_connection_destroy(*connection);
   }

   return status;
}


/**
 * allocates and generates a filename based on the
 * user-supplied pattern and the frame number.
 * on successful return, finalname and tempname point to malloc()ed strings
 * which must be freed externally.  (on failure, returns nulls that
 * don't need free()ing.)
 *
 * @param finalname pointer receives an
 * @param pattern sprintf pattern with %d to be replaced by frame
 * @param frame for timelapse, the frame number
 * @return returns a mmal_status_t giving result of operation
*/

mmal_status_t create_filenames(char** finalname, char** tempname, char * pattern, int frame)
{
   *finalname = null;
   *tempname = null;
   if (0 > asprintf(finalname, pattern, frame) ||
       0 > asprintf(tempname, "%s~", *finalname))
   {
      if (*finalname != null)
      {
         free(*finalname);
      }
      return mmal_enomem;    // it may be some other error, but it is not worth getting it right
   }
   return mmal_success;
}

/**
 * checks if specified port is valid and enabled, then disables it
 *
 * @param port  pointer the port
 *
 */
static void check_disable_port(mmal_port_t *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}

/**
 * handler for sigint signals
 *
 * @param signal_number id of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
   if (signal_number == sigusr1)
   {
      // handle but ignore - prevents us dropping out if started in none-signal mode
      // and someone sends us the usr1 signal anyway
   }
   else
   {
      // going to abort on all other signals
      vcos_log_error("aborting program\n");
      exit(130);
   }
}


/**
 * function to wait in various ways (depending on settings) for the next frame
 *
 * @param state pointer to the state data
 * @param [in][out] frame the last frame number, adjusted to next frame number on output
 * @return !0 if to continue, 0 if reached end of run
 */
static int wait_for_next_frame(raspistill_state *state, int *frame)
{
   static int64_t complete_time = -1;
   int keep_running = 1;

   int64_t current_time =  vcos_getmicrosecs64()/1000;

   if (complete_time == -1)
      complete_time =  current_time + state->timeout;

   // if we have run out of time, flag we need to exit
   // if timeout = 0 then always continue
   if (current_time >= complete_time && state->timeout != 0)
      keep_running = 0;

   switch (state->framenextmethod)
   {
   case frame_next_single :
      // simple timeout for a single capture
      vcos_sleep(state->timeout);
      return 0;

   case frame_next_forever :
   {
      *frame+=1;

      // have a sleep so we don't hog the cpu.
      vcos_sleep(10000);

      // run forever so never indicate end of loop
      return 1;
   }

   case frame_next_timelapse :
   {
      static int64_t next_frame_ms = -1;

      // always need to increment by at least one, may add a skip later
      *frame += 1;

      if (next_frame_ms == -1)
      {
         vcos_sleep(state->timelapse);

         // update our current time after the sleep
         current_time =  vcos_getmicrosecs64()/1000;

         // set our initial 'next frame time'
         next_frame_ms = current_time + state->timelapse;
      }
      else
      {
         int64_t this_delay_ms = next_frame_ms - current_time;

         if (this_delay_ms < 0)
         {
            // we are already past the next exposure time
            if (-this_delay_ms < -state->timelapse/2)
            {
               // less than a half frame late, take a frame and hope to catch up next time
               next_frame_ms += state->timelapse;
               vcos_log_error("frame %d is %d ms late", *frame, (int)(-this_delay_ms));
            }
            else
            {
               int nskip = 1 + (-this_delay_ms)/state->timelapse;
               vcos_log_error("skipping frame %d to restart at frame %d", *frame, *frame+nskip);
               *frame += nskip;
               this_delay_ms += nskip * state->timelapse;
               vcos_sleep(this_delay_ms);
               next_frame_ms += (nskip + 1) * state->timelapse;
            }
         }
         else
         {
            vcos_sleep(this_delay_ms);
            next_frame_ms += state->timelapse;
         }
      }

      return keep_running;
   }

   case frame_next_keypress :
   {
      int ch;

      if (state->verbose)
         fprintf(stderr, "press enter to capture, x then enter to exit\n");

      ch = getchar();
      *frame+=1;
      if (ch == 'x' || ch == 'x')
         return 0;
      else
      {
         return keep_running;
      }
   }

   case frame_next_immediately :
   {
      // not waiting, just go to next frame.
      // actually, we do need a slight delay here otherwise exposure goes
      // badly wrong since we never allow it frames to work it out
      // this could probably be tuned down.
      // first frame has a much longer delay to ensure we get exposure to a steady state
      if (*frame == 0)
         vcos_sleep(1000);
      else
         vcos_sleep(30);

      *frame+=1;

      return keep_running;
   }

   case frame_next_gpio :
   {
       // intended for gpio firing of a capture
      return 0;
   }

   case frame_next_signal :
   {
      // need to wait for a sigusr1 signal
      sigset_t waitset;
      int sig;
      int result = 0;

      sigemptyset( &waitset );
      sigaddset( &waitset, sigusr1 );

      // we are multi threaded because we use mmal, so need to use the pthread
      // variant of procmask to block sigusr1 so we can wait on it.
      pthread_sigmask( sig_block, &waitset, null );

      if (state->verbose)
      {
         fprintf(stderr, "waiting for sigusr1 to initiate capture\n");
      }

      result = sigwait( &waitset, &sig );

      if (state->verbose)
      {
         if( result == 0)
         {
            fprintf(stderr, "received sigusr1\n");
         }
         else
         {
            fprintf(stderr, "bad signal received - error %d\n", errno);
         }
      }

      *frame+=1;

      return keep_running;
   }
   } // end of switch

   // should have returned by now, but default to timeout
   return keep_running;
}

static void rename_file(raspistill_state *state, file *output_file,
      const char *final_filename, const char *use_filename, int frame)
{
   mmal_status_t status;

   fclose(output_file);
   vcos_assert(use_filename != null && final_filename != null);
   if (0 != rename(use_filename, final_filename))
   {
      vcos_log_error("could not rename temp file to: %s; %s",
            final_filename,strerror(errno));
   }
   if (state->linkname)
   {
      char *use_link;
      char *final_link;
      status = create_filenames(&final_link, &use_link, state->linkname, frame);

      // create hard link if possible, symlink otherwise
      if (status != mmal_success
            || (0 != link(final_filename, use_link)
               &&  0 != symlink(final_filename, use_link))
            || 0 != rename(use_link, final_link))
      {
         vcos_log_error("could not link as filename: %s; %s",
               state->linkname,strerror(errno));
      }
      if (use_link) free(use_link);
      if (final_link) free(final_link);
   }
}

void* gps_update(struct gps_info* sharedData)
{
   char buffer[500];
   char line_buffer[100];
   int in_buffer = 0;
   while(1)
   {
      int read_bytes=read(shareddata.serial,(buffer+in_buffer),sizeof(buffer)-in_buffer);
      in_buffer += read_bytes;
      int start_line = 0;
      for (int i = 0; i<in_buffer; i++)
      {
         if (buffer[i] == '\n')
         {
            int line_length = i - start_line + 1;
            strncpy(line_buffer,&buffer[start_line],line_length);
            line_buffer[line_length]='\0';
            start_line = i+1;
            switch(minmea_sentence_id(line_buffer))
            {
               case minmea_sentence_rmc:
               {
                  struct minmea_sentence_rmc frame;
                  if (minmea_parse_rmc(&frame, line))
                  {

                     struct coordinate latitude;
                     if (frame.latitude.value > 0)
                     {
                        latitude.ref = north;
                     }
                     else
                     {
                        frame.latitude.value *= -1;
                        latitude.ref = south;
                     }
                     latitude.deg = frame.latitude.value / (frame.latitude.scale * 100);
                     latitude.min_scaled = frame.latitude.value % (frame.latitude.scale * 100);
                     latitude.min_scale = frame.latitude.scale;


                     struct coordinate longitude;
                     if (frame.longitude.value > 0)
                     {
                        longitude.ref = east;
                     }
                     else
                     {
                        frame.longitude.value *= -1;
                        longitude.ref = west;
                     }
                     longitude.deg = frame.longitude.value / (frame.longitude.scale * 100);
                     longitude.min_scaled = frame.longitude.value % (frame.longitude.scale * 100);
                     longitude.min_scale = frame.longitude.scale;


                     shareddata.latitude = latitude;
                     shareddata.longitude = longitude;
                     shareddata.speed = frame.speed;
                     shareddata.course = frame.course;
                     
                  }
               } break;
               case minmea_sentence_gga:
               {
                  struct minmea_sentence_gga frame;
                  if (minmea_parse_gga(&frame, line))
                  {
                     struct coordinate latitude;
                     if (frame.latitude.value > 0)
                     {
                        latitude.ref = north;
                     }
                     else
                     {
                        frame.latitude.value *= -1;
                        latitude.ref = south;
                     }
                     latitude.deg = frame.latitude.value / (frame.latitude.scale * 100);
                     latitude.min_scaled = frame.latitude.value % (frame.latitude.scale * 100);
                     latitude.min_scale = frame.latitude.scale;

                     struct coordinate longitude;
                     if (frame.longitude.value > 0)
                     {
                        longitude.ref = east;
                     }
                     else
                     {
                        frame.longitude.value *= -1;
                        longitude.ref = west;
                     }
                     longitude.deg = frame.longitude.value / (frame.longitude.scale * 100);
                     longitude.min_scaled = frame.longitude.value % (frame.longitude.scale * 100);
                     longitude.min_scale = frame.longitude.scale;

                     shareddata.latitude = latitude;
                     shareddata.longitude = longitude;

                     shareddata.altitude = frame.altitude;

                  }
               } break;
            }
            in_buffer -= line_length;
         }

      }
      if (start_line > 0)
      {
         memmove(buffer,buffer+start_line,in_buffer);
      }
      usleep(100000);
   }

}

/**
 * main
 */
int main(int argc, const char **argv)
{
   int serial_ret = serialopen("/dev/ttyama0",9600);
   if (serial_ret < 0)
   {
      fprintf(stderr, "failed to init serial\n");
      exit(ex_usage);
   }
   nav_data.serial=serial_ret;
   pthread_t gps_sync;
   pthread_create(&gps_sync, null, gps_update, &nav_data);



   // our main data storage vessel..
   raspistill_state state;
   int exit_code = ex_ok;

   mmal_status_t status = mmal_success;
   mmal_port_t *camera_preview_port = null;
   mmal_port_t *camera_video_port = null;
   mmal_port_t *camera_still_port = null;
   mmal_port_t *preview_input_port = null;
   mmal_port_t *encoder_input_port = null;
   mmal_port_t *encoder_output_port = null;

   bcm_host_init();

   // register our application with the logging system
   vcos_log_register("raspistill", vcos_log_category);

   signal(sigint, signal_handler);

   // disable usr1 for the moment - may be reenabled if go in to signal capture mode
   signal(sigusr1, sig_ign);

   default_status(&state);

   // do we have any parameters
   if (argc == 1)
   {
      fprintf(stderr, "\%s camera app %s\n\n", basename(argv[0]), version_string);

      display_valid_parameters(basename(argv[0]));
      exit(ex_usage);
   }

   // parse the command line and put options in to our status structure
   if (parse_cmdline(argc, argv, &state))
   {
      exit(ex_usage);
   }

   if (state.verbose)
   {
      fprintf(stderr, "\n%s camera app %s\n\n", basename(argv[0]), version_string);

      dump_status(&state);
   }

   if (state.usegl)
      raspitex_init(&state.raspitex_state);

   // ok, we have a nice set of parameters. now set up our components
   // we have three components. camera, preview and encoder.
   // camera and encoder are different in stills/video, but preview
   // is the same so handed off to a separate module

   if ((status = create_camera_component(&state)) != mmal_success)
   {
      vcos_log_error("%s: failed to create camera component", __func__);
      exit_code = ex_software;
   }
   else if ((!state.usegl) && (status = raspipreview_create(&state.preview_parameters)) != mmal_success)
   {
      vcos_log_error("%s: failed to create preview component", __func__);
      destroy_camera_component(&state);
      exit_code = ex_software;
   }
   else if ((status = create_encoder_component(&state)) != mmal_success)
   {
      vcos_log_error("%s: failed to create encode component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      exit_code = ex_software;
   }
   else
   {
      port_userdata callback_data;

      if (state.verbose)
         fprintf(stderr, "starting component connection stage\n");

      camera_preview_port = state.camera_component->output[mmal_camera_preview_port];
      camera_video_port   = state.camera_component->output[mmal_camera_video_port];
      camera_still_port   = state.camera_component->output[mmal_camera_capture_port];
      encoder_input_port  = state.encoder_component->input[0];
      encoder_output_port = state.encoder_component->output[0];

      if (! state.usegl)
      {
         if (state.verbose)
            fprintf(stderr, "connecting camera preview port to video render.\n");

         // note we are lucky that the preview and null sink components use the same input port
         // so we can simple do this without conditionals
         preview_input_port  = state.preview_parameters.preview_component->input[0];

         // connect camera to preview (which might be a null_sink if no preview required)
         status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);
      }

      if (status == mmal_success)
      {
         vcos_status_t vcos_status;

         if (state.verbose)
            fprintf(stderr, "connecting camera stills port to encoder input port\n");

         // now connect the camera to the encoder
         status = connect_ports(camera_still_port, encoder_input_port, &state.encoder_connection);

         if (status != mmal_success)
         {
            vcos_log_error("%s: failed to connect camera video port to encoder input", __func__);
            goto error;
         }

         // set up our userdata - this is passed though to the callback where we need the information.
         // null until we open our filename
         callback_data.file_handle = null;
         callback_data.pstate = &state;
         vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "raspistill-sem", 0);

         vcos_assert(vcos_status == vcos_success);

         /* if gl preview is requested then start the gl threads */
         if (state.usegl && (raspitex_start(&state.raspitex_state) != 0))
            goto error;

         if (status != mmal_success)
         {
            vcos_log_error("failed to setup encoder output");
            goto error;
         }

         if (state.demomode)
         {
            // run for the user specific time..
            int num_iterations = state.timeout / state.demointerval;
            int i;
            for (i=0;i<num_iterations;i++)
            {
               raspicamcontrol_cycle_test(state.camera_component);
               vcos_sleep(state.demointerval);
            }
         }
         else
         {
            int frame, keep_looping = 1;
            file *output_file = null;
            char *use_filename = null;      // temporary filename while image being written
            char *final_filename = null;    // name that file gets once writing complete

            frame = 0;

            while (keep_looping)
            {
               keep_looping = wait_for_next_frame(&state, &frame);

                if (state.datetime)
                {
                   time_t rawtime;
                   struct tm *timeinfo;

                   time(&rawtime);
                   timeinfo = localtime(&rawtime);

                   frame = timeinfo->tm_mon+1;
                   frame *= 100;
                   frame += timeinfo->tm_mday;
                   frame *= 100;
                   frame += timeinfo->tm_hour;
                   frame *= 100;
                   frame += timeinfo->tm_min;
                   frame *= 100;
                   frame += timeinfo->tm_sec;
                }
                if (state.timestamp)
                {
                   frame = (int)time(null);
                }

               // open the file
               if (state.filename)
               {
                  if (state.filename[0] == '-')
                  {
                     output_file = stdout;

                     // ensure we don't upset the output stream with diagnostics/info
                     state.verbose = 0;
                  }
                  else
                  {
                     vcos_assert(use_filename == null && final_filename == null);
                     status = create_filenames(&final_filename, &use_filename, state.filename, frame);
                     if (status  != mmal_success)
                     {
                        vcos_log_error("unable to create filenames");
                        goto error;
                     }

                     if (state.verbose)
                        fprintf(stderr, "opening output file %s\n", final_filename);
                        // technically it is opening the temp~ filename which will be ranamed to the final filename

                     output_file = fopen(use_filename, "wb");

                     if (!output_file)
                     {
                        // notify user, carry on but discarding encoded output buffers
                        vcos_log_error("%s: error opening output file: %s\nno output file will be generated\n", __func__, use_filename);
                     }
                  }

                  callback_data.file_handle = output_file;
               }

               // we only capture if a filename was specified and it opened
               if (state.usegl && state.glcapture && output_file)
               {
                  /* save the next gl framebuffer as the next camera still */
                  int rc = raspitex_capture(&state.raspitex_state, output_file);
                  if (rc != 0)
                     vcos_log_error("failed to capture gl preview");
                  rename_file(&state, output_file, final_filename, use_filename, frame);
               }
               else if (output_file)
               {
                  int num, q;

                  // must do this before the encoder output port is enabled since
                  // once enabled no further exif data is accepted
                  if ( state.enableexiftags )
                  {
                     add_exif_tags(&state);
                  }
                  else
                  {
                     mmal_port_parameter_set_boolean(
                        state.encoder_component->output[0], mmal_parameter_exif_disable, 1);
                  }

                  // same with raw, apparently need to set it for each capture, whilst port
                  // is not enabled
                  if (state.wantraw)
                  {
                     if (mmal_port_parameter_set_boolean(camera_still_port, mmal_parameter_enable_raw_capture, 1) != mmal_success)
                     {
                        vcos_log_error("raw was requested, but failed to enable");
                     }
                  }

                  // there is a possibility that shutter needs to be set each loop.
                  if (mmal_status_to_int(mmal_port_parameter_set_uint32(state.camera_component->control, mmal_parameter_shutter_speed, state.camera_parameters.shutter_speed) != mmal_success))
                     vcos_log_error("unable to set shutter speed");


                  // enable the encoder output port
                  encoder_output_port->userdata = (struct mmal_port_userdata_t *)&callback_data;

                  if (state.verbose)
                     fprintf(stderr, "enabling encoder output port\n");

                  // enable the encoder output port and tell it its callback function
                  status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

                  // send all the buffers to the encoder output port
                  num = mmal_queue_length(state.encoder_pool->queue);

                  for (q=0;q<num;q++)
                  {
                     mmal_buffer_header_t *buffer = mmal_queue_get(state.encoder_pool->queue);

                     if (!buffer)
                        vcos_log_error("unable to get a required buffer %d from pool queue", q);

                     if (mmal_port_send_buffer(encoder_output_port, buffer)!= mmal_success)
                        vcos_log_error("unable to send a buffer to encoder output port (%d)", q);
                  }

                  if (state.burstcapturemode && frame==1)
                  {
                     mmal_port_parameter_set_boolean(state.camera_component->control,  mmal_parameter_camera_burst_capture, 1);
                  }

                  if (state.verbose)
                     fprintf(stderr, "starting capture %d\n", frame);

                  if (mmal_port_parameter_set_boolean(camera_still_port, mmal_parameter_capture, 1) != mmal_success)
                  {
                     vcos_log_error("%s: failed to start capture", __func__);
                  }
                  else
                  {
                     // wait for capture to complete
                     // for some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
                     // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
                     vcos_semaphore_wait(&callback_data.complete_semaphore);
                     if (state.verbose)
                        fprintf(stderr, "finished capture %d\n", frame);
                  }

                  // ensure we don't die if get callback with no open file
                  callback_data.file_handle = null;

                  if (output_file != stdout)
                  {
                     rename_file(&state, output_file, final_filename, use_filename, frame);
                  }
                  else
                  {
                     fflush(output_file);
                  }
                  // disable encoder output port
                  status = mmal_port_disable(encoder_output_port);
               }

               if (use_filename)
               {
                  free(use_filename);
                  use_filename = null;
               }
               if (final_filename)
               {
                  free(final_filename);
                  final_filename = null;
               }
            } // end for (frame)

            vcos_semaphore_delete(&callback_data.complete_semaphore);
         }
      }
      else
      {
         mmal_status_to_int(status);
         vcos_log_error("%s: failed to connect camera to preview", __func__);
      }

error:

      mmal_status_to_int(status);

      if (state.verbose)
         fprintf(stderr, "closing down\n");

      if (state.usegl)
      {
         raspitex_stop(&state.raspitex_state);
         raspitex_destroy(&state.raspitex_state);
      }

      // disable all our ports that are not handled by connections
      check_disable_port(camera_video_port);
      check_disable_port(encoder_output_port);

      if (state.preview_connection)
         mmal_connection_destroy(state.preview_connection);

      if (state.encoder_connection)
         mmal_connection_destroy(state.encoder_connection);


      /* disable components */
      if (state.encoder_component)
         mmal_component_disable(state.encoder_component);

      if (state.preview_parameters.preview_component)
         mmal_component_disable(state.preview_parameters.preview_component);

      if (state.camera_component)
         mmal_component_disable(state.camera_component);

      destroy_encoder_component(&state);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);

      if (state.verbose)
         fprintf(stderr, "close down completed, all components disconnected, disabled and destroyed\n\n");
   }

   if (status != MMAL_SUCCESS)
      raspicamcontrol_check_configuration(128);

   return exit_code;
}


