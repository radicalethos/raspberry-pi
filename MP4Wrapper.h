#ifndef MP4Wrapper_h
#define MP4Wrapper_h

#include <stdio.h>
#include <iostream>
#include <map>


#include "minimp4.h"
#include "AlsaMic.h"

#define FFMPEG_LIB

#ifdef FFMPEG_LIB
#include "FFMPEG.h"
#endif

#ifndef _FILENAME_
#define _FILENAME_	(strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__ )
#endif


//#define TIME_BASE	90000

namespace rp {

typedef struct {
	uint8_t code[4];
	uint8_t byte1;
} NALU_HEADER;

class MP4Wrapper {



public:
	MP4Wrapper(AlsaMic::alsamic_sink_t sink_type, int width, int height, int framerate, int vbitrate, int abitrate, bool logging) : mux_(NULL), frameLen_(0), ats_(0), ts_(0), samples_(0),
		framerate_(framerate), alsa_mic_(sink_type, 1, 44100, abitrate), sink_type_(sink_type)
#ifdef FFMPEG_LIB
		,ffmpeg_(width, height, framerate, vbitrate, abitrate)
		
#endif
		{
		fprintf(stderr, "%s:%s(%d x %d)\n", _FILENAME_, __func__, width, height);

#ifdef FFMPEG_LIB
		ffmpeg_.func_writer_callback = file_writer_callback;	
		ffmpeg_.func_seek_callback = file_seek_callback;
		ffmpeg_.setLogging(logging);			
#endif
		
	}

	virtual ~MP4Wrapper() {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);
		finish();
	}

	void finish() {
		//fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);
		alsa_mic_.alsa_sink_close();
		endMovie();
	}

	bool alsa_open(const std::string& device) {
		return alsa_mic_.alsa_open(device);
	}

	bool alsa_init() {
		return alsa_mic_.alsa_init();
	}

	bool aac_init() {
		return alsa_mic_.aac_init();
	}

	bool alsa_good() {
		return alsa_mic_.good();
	}

	bool compare_time() {
		return alsa_good() && ats_ < ts_;

#ifdef FFMPEG_LIB
		return alsa_good() && ffmpeg_.compare_time();
#else
		return alsa_good() && ats_ < ts_;
#endif
	}

	void alsa_sink_open(const char* outfile) {
		alsa_mic_.alsa_sink_open(outfile);
	}

	/*
	void ffmpeg_file_callbacks(void* data, int(*func_write)(uint8_t*, int), int64_t(*func_seek)(int64_t, int)) {
#ifdef FFMPEG_LIB
		ffmpeg_.func_writer_callback = func_write;

		ffmpeg_.func_seek_callback = func_seek;
#endif		
	}*/

	void set_call_func(void* data, void (*func)(void*, uint8_t*, int, int)) {
		alsa_mic_.set_call_func(data, func);	
	}

	void alsa_sink_close() {
		alsa_mic_.alsa_sink_close();
	}

	int alsa_capture(int echo) {
		int samples = alsa_mic_.alsa_capture(echo);
		ats_ = (uint64_t) alsa_mic_.getSamplesRead() * TIME_SCALE/alsa_mic_.getSampleRate();
		return samples;
	}

	bool addAudio(const std::string& filename) {

#ifdef FFMPEG_LIB
		try {
			ffmpeg_.addAudio();
			return true;
		} catch (std::exception& e) {
			fprintf(stderr, ">>>>>>>> %s(%s)\n", __func__, e.what());
			return false;
		}
#else
		
		if(AlsaMic::alsamic_sink_aac == sink_type_) {
			return openAudioTrack();

		} else {
			alsa_mic_.alsa_sink_open(filename.c_str());
			return true;
		}
#endif
	}

	bool should_allow_addmovie() {
		return ffmpeg_.should_allow_addmovie();
	}

	bool addMovie(const std::string& filename) {
		//fprintf(stderr, "%s()\n", __func__);
		
#ifdef FFMPEG_LIB
		try {
			ffmpeg_.addMovie(filename, framebuf_, frameLen_);
			frameLen_ = 0;
			return true;
		} catch (std::exception& e) {
			fprintf(stderr, ">>>>>>>> %s(%s)\n", __func__, e.what());
			frameLen_ = 0;
			return false;
		}
#else
		mux_ = MP4E_open(0, 0, fp_, write_callback);
		if (MP4E_STATUS_OK != mp4_h26x_write_init(&mp4wr_, mux_, width, height, 0)) {
			false;
		}
		return true;
#endif
	}

	void writeHeader(const std::string& filename) {
#ifdef FFMPEG_LIB
		return ffmpeg_.writeHeader(filename);
#endif
	}

	void endMovie() {
		//fprintf(stderr, "%s()\n", __func__);
#ifdef FFMPEG_LIB
		ffmpeg_.endMovie();
#else
		MP4E_close(mux_);
  		mp4_h26x_write_close(&mp4wr_);
#endif
	}

	void copyBuffer(uint8_t* buf, int count) {

		memcpy(framebuf_ + frameLen_, buf, count);	
		frameLen_ += count;
	}

	bool writeNAL() {

#ifdef FFMPEG_LIB

		bool iframe = false;
		uint8_t code[] = { 0x00, 0x00, 0x00, 0x01 };
		NALU_HEADER* head = (NALU_HEADER*)framebuf_;
		if(!memcmp(head->code, code, 4)) {
			unsigned char  c = head->byte1;
			int forbidden_zero_bit = (c >> 7);
			int nal_ref_idc = (c >> 5) & 3;
			int nal_unit_type = c & 0x1F;
			iframe = (nal_unit_type == 5);
		} 

		ffmpeg_.write_video_frame(framebuf_, frameLen_, iframe);
		ts_ += TIME_SCALE/framerate_; 
		frameLen_ = 0;
		return true;
#else

		int res;
		if(config)
			res = mp4_h26x_write_nal(&mp4wr_, framebuf_, frameLen_, TIME_SCALE/framerate_);
		else
			res = mp4_h26x_write2_nal(&mp4wr_, framebuf_, frameLen_, TIME_SCALE/framerate_);

		frameLen_ = 0;

		if(MP4E_STATUS_OK == res) {
			ts_ += TIME_SCALE/framerate_; 
			return true;
		} 

		return false;

#endif
	}

	bool writeSample(uint8_t* buffer, int samples, int bytes) {
#ifdef FFMPEG_LIB
		return ffmpeg_.write_audio_frame(buffer, samples, bytes);
#endif
		//fprintf (stderr, "%s: bytes = %d\n", __func__, bytes);
		return MP4E_STATUS_OK == MP4E_put_sample(mux_, audio_track_id_, 
			buffer, bytes, samples*TIME_SCALE/alsa_mic_.getSampleRate(), MP4E_SAMPLE_RANDOM_ACCESS);
	}

private:

	static int file_writer_callback(uint8_t *buf, int len) {
		assert(fp_ != NULL);
		return fwrite(buf, sizeof(uint8_t), len, fp_);
	}

	static int64_t file_seek_callback(int64_t offset, int whence) {
		assert(fp_ != NULL);
		if(fseek(fp_, offset, whence) == 0) {
			return offset;
		}
		return -1;
	}

	bool openAudioTrack() {

		MP4E_track_t tr;
		tr.track_media_kind = e_audio;
		tr.language[0] = 'u';
		tr.language[1] = 'n';
		tr.language[2] = 'd';
		tr.language[3] = 0;
		tr.object_type_indication = MP4_OBJECT_TYPE_AUDIO_ISO_IEC_14496_3;
		tr.time_scale = TIME_SCALE;
		tr.default_duration = 0;
		tr.u.a.channelcount = 1;
		audio_track_id_ = MP4E_add_track(mux_, &tr);
		return MP4E_STATUS_OK == MP4E_set_dsi(mux_, audio_track_id_, alsa_mic_.getAACConfBuf(), alsa_mic_.getAACConfSize()); 
	}

	static int write_callback(int64_t offset, const void *buffer, size_t size, void *token) {

		FILE *f = (FILE*)token;
		fseek(f, offset, SEEK_SET);
		return fwrite(buffer, 1, size, f) != size;
	}

#ifdef FFMPEG_LIB
	FFMPEG ffmpeg_;
#endif
	AlsaMic::alsamic_sink_t sink_type_;
    	mp4_h26x_writer_t mp4wr_;
	MP4E_mux_t *mux_;
	static FILE* fp_;
	
	int audio_track_id_;
	int samples_;
	uint64_t ts_;
	uint64_t ats_;
	
	int frameLen_;
	int framerate_;
	uint8_t framebuf_[1024*1024];

	rp::AlsaMic alsa_mic_;


	
	/*
	MP4E_track_t tr;
	tr.track_media_kind = e_audio;
	tr.language[0] = 'u';
	tr.language[1] = 'n';
	tr.language[2] = 'd';
	tr.language[3] = 0;
	tr.object_type_indication = MP4_OBJECT_TYPE_AUDIO_ISO_IEC_14496_3;
	tr.time_scale = TIME_SCALE;
	tr.default_duration = 0;
	tr.u.a.channelcount = 1;
	state.audio_track_id = MP4E_add_track(state.mux, &tr);
	int status = MP4E_set_dsi(state.mux, state.audio_track_id,
		state.alsa->getConfBuf(), state.alsa->getConfSize()); */
};

}

#endif
