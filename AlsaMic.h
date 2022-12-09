#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>

#include <iostream>
#include <stdexcept>
#include <alsa/asoundlib.h>

#ifdef AAC_LIB
#include <fdk-aac/aacenc_lib.h>
#include <fdk-aac/aacdecoder_lib.h>
#endif

//#include <shine/layer3.h>


#define AUDIO_RATE 12000

#ifndef _FILENAME_
#define _FILENAME_	(strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__ )
#endif
//#define WAVE_READER




namespace rp {

class AlsaMic {

// SND_PCM_FORMAT_S16_LE (2)

// SND_PCM_FORMAT_FLOAT (14)


public:

	typedef enum {

		alsamic_sink_nul,
		alsamic_sink_pcm,
		alsamic_sink_wav,
		alsamic_sink_aac,
		alsamic_sink_buf,
		alsamic_sink_aac2,
		alsamic_sink_mp3,

	} alsamic_sink_t;

	AlsaMic(alsamic_sink_t sink_type, int channels, int rsample, int bitrate) : m_bitrate(bitrate),
		m_sink_type(sink_type), m_h_alsa(0), m_format(SND_PCM_FORMAT_S16_LE), 
		m_rate(rsample), m_channels(channels), m_alsa_audiobuf(NULL), 
		m_output_encbuf(NULL), m_alsa_info(NULL), m_file_sink_out_(NULL), func_buf_call_ptr(NULL), 
		m_alsa_bits_per_frame(0), alsa_ok(false), loop_ok(true), m_alsa_non_blocking(0),
		m_out_encbuf_bytes(0), m_alsa_frame_size(0), m_userdata(),
		m_alsa_total_bytes(0), m_alsa_total_spl(0), m_counter(0)
#ifdef AAC_LIB
		,m_h_aac(NULL)
#endif	
		 {

		fprintf(stderr, "%s:AlsaMic()\n", _FILENAME_);

		if(m_alsa_non_blocking) {
			m_alsa_non_blocking = SND_PCM_NONBLOCK;	
		}

		switch(m_sink_type) {
			case AlsaMic::alsamic_sink_aac:
				if(!aac_init()) {
					fprintf(stderr, "%s(>>>>>>>>>>>>>>> aac_init failed)\n", __func__);
				}
				break;
			case AlsaMic::alsamic_sink_mp3:
				if(!shine_init()) {
					fprintf(stderr, "%s(>>>>>>>>>>>>>>> shine_init failed)\n", __func__);
				}
				break;
		}

		
	}

	virtual ~AlsaMic() {
		fprintf(stderr, "%s:~AlsaMic()\n", _FILENAME_);

		if(m_h_alsa) {
			snd_pcm_close(m_h_alsa);
			free(m_output_encbuf);
			free(m_alsa_audiobuf);
		}
	}

	void set_call_func(void* data, void (*func)(void*, uint8_t*, int, int)) {
		func_buf_call_ptr = func;
		m_userdata = data;
	}

	int alsa_capture(int echo) {

		unsigned long t1 = getMicroTime();

		//size_t f = m_alsa_chunk_bytes * 8 / m_alsa_bits_per_frame;

		//fprintf(stderr, "%s	%d ms\n", __func__, f);

		//printf("m_alsa_chunk_bytes=%d, f=%d, m_alsa_bits_per_frame=%d\n", m_alsa_chunk_bytes, f, m_alsa_bits_per_frame);
		
		int err = 0;
		if ((err = snd_pcm_readi(m_h_alsa, m_alsa_audiobuf, m_alsa_frame_size)) != m_alsa_frame_size) {

			if(err == -EPIPE) {
				if(echo) {
					fprintf (stderr, "%s: EPIPE (%s)\n", __func__,  snd_strerror(err));
				}
				snd_pcm_prepare(m_h_alsa);
				return 0;

			} else if(err == -EAGAIN) {
				snd_pcm_wait(m_h_alsa, 100);
				err = m_alsa_frame_size;

			} else {				
				fprintf (stderr, "read from audio interface failed (%s)\n", snd_strerror(err));
				alsa_ok = false;
				return -1;
			}
		}

		if(echo) {
			fprintf(stderr, "%s %d frames read in %.2f ms\n", __func__, m_alsa_frame_size, (getMicroTime() - t1)/1000.0);
		}

		int bytes = err * m_alsa_bits_per_frame / 8;
		m_alsa_total_bytes += bytes;
		m_alsa_total_spl += err;

		return alsa_proc_buffer(m_alsa_audiobuf, err, bytes);
	}

	void alsa_start_capture(int echo) {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__); 

		for (int s = 0; loop_ok; ++s) {
			if(alsa_capture(echo) < 0) {
				break;
			} 
		}

		snd_pcm_drain(m_h_alsa);
	}

	int alsa_proc_buffer(uint8_t* buffer, int samples, int bytes) {
		//fprintf(stderr, "%s(samples=%d, bytes=%d)\n", __func__, samples, bytes);

		int outsize = bytes;
		//int out_len = sizeof(outbuf);
		//printf("m_aac_chunk_bytes=%d\n", m_out_encbuf_bytes);

		switch(m_sink_type) {
			case alsamic_sink_nul:
				break;
			case alsamic_sink_pcm:
				alsa_write_raw(buffer, samples, bytes);
				break;
			case alsamic_sink_wav:
				alsa_write_raw(buffer, samples, bytes);
				break;
			case alsamic_sink_aac:
				assert(m_output_encbuf != NULL);
				if((outsize = get_aac_frame(buffer, m_output_encbuf, bytes, m_out_encbuf_bytes)) > 0)
					alsa_write_raw(m_output_encbuf, samples, outsize);
				break;
			case alsamic_sink_aac2:
				get_ffm_frame(buffer, m_output_encbuf, bytes, m_out_encbuf_bytes);
				break;
			case alsamic_sink_buf:
				break;
			default:
				fprintf(stderr, "%s(unknown sink)\n", __func__);
		}

		return outsize;
	}

	void alsa_sink_open(const char* outfile) {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		if(m_file_sink_out_) {
			fclose(m_file_sink_out_);
			m_file_sink_out_ = NULL;
		}

		if(outfile) {
			m_file_sink_out_ = fopen(outfile, "wb");
		}

		switch(m_sink_type) {
			case alsamic_sink_nul:
				break;
			case alsamic_sink_pcm:
				break;
			case alsamic_sink_wav:
				if(m_file_sink_out_) {
					alsa_write_wave_header(m_file_sink_out_);
				}
				break;
			case alsamic_sink_aac:
				break;
			case alsamic_sink_aac2:
				break;
			case alsamic_sink_buf:
				assert(m_file_sink_out_ == NULL);
				break;
			default:
				fprintf(stderr, "%s(unknown sink)\n", __func__);
		}
	}

	void alsa_sink_close() {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		switch(m_sink_type) {
			case alsamic_sink_nul:
				break;
			case alsamic_sink_pcm:
				if(m_file_sink_out_) {
					fclose(m_file_sink_out_);
					m_file_sink_out_ = NULL;
				}
				break;
			case alsamic_sink_wav:				
				alsa_write_wave_trailor(m_file_sink_out_, m_alsa_total_bytes);
				break;
			case alsamic_sink_aac:
#ifdef AAC_LIB
				aacEncClose(&m_h_aac);
#endif
				break;
			case alsamic_sink_aac2:
				break;
			case alsamic_sink_buf:
				break;
			default:
				fprintf(stderr, "%s(unknown sink)\n", __func__);
		}

		if(m_file_sink_out_) {
			fclose(m_file_sink_out_);
			m_file_sink_out_ = NULL;
		}
	}

	void alsa_write_raw(uint8_t* buffer, int samples, int bytes) {

		if(m_file_sink_out_) {
			fwrite(buffer, 1, bytes, m_file_sink_out_);

		} else if(func_buf_call_ptr) {
			func_buf_call_ptr(m_userdata, buffer, samples, bytes);

		} else {
			fprintf(stderr, "%s func_buf_call_ptr not set\n", __func__);	
		}
	}


	int get_ffm_frame(uint8_t* inbuf, uint8_t* outbuf, int in_len, int out_len) {
		return 0;
	}


	int get_aac_frame(uint8_t* inbuf, uint8_t* outbuf, int in_len, int out_len) {

#ifdef AAC_LIB
		AACENC_BufDesc in_buf = { 0 }, out_buf = { 0 };
		AACENC_InArgs in_args = { 0 };
		AACENC_OutArgs out_args = { 0 };
		int in_identifier = IN_AUDIO_DATA;
		int in_size, in_elem_size;
		int out_identifier = OUT_BITSTREAM_DATA;
		int out_size, out_elem_size;
		//int read, i;
		void *in_ptr, *out_ptr;
		
		AACENC_ERROR err;

		
		in_ptr = inbuf;
		in_size = in_len;
		in_elem_size = 2;

		in_args.numInSamples = in_len <= 0 ? -1 : in_len/2;
		in_buf.numBufs = 1;
		in_buf.bufs = &in_ptr;
		in_buf.bufferIdentifiers = &in_identifier;
		in_buf.bufSizes = &in_size;
		in_buf.bufElSizes = &in_elem_size;

		out_ptr = outbuf;
		out_size = out_len;
		out_elem_size = 1;
		out_buf.numBufs = 1;
		out_buf.bufs = &out_ptr;
		out_buf.bufferIdentifiers = &out_identifier;
		out_buf.bufSizes = &out_size;
		out_buf.bufElSizes = &out_elem_size;

		unsigned long t1 = getMicroTime();
		if ((err = aacEncEncode(m_h_aac, &in_buf, &out_buf, &in_args, &out_args)) != AACENC_OK) {
				
			if (err == AACENC_ENCODE_EOF) {
				printf("aacEndEncode - done\n");
			} else {
				fprintf(stderr, "Encoding failed\n");
			}

			return -1;
		}

		//fprintf(stderr, "%s(%.2f ms)\n", __func__, (getMicroTime() - t1)/1000.0);
		//fprintf(stderr, "%s(%d	%d)\n", __func__, out_args.numOutBytes, in_len);

		return out_args.numOutBytes;

#else
		return 0;
#endif
	}

public:

	bool alsa_open(const std::string& device) {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		int err;
		if ((err = snd_pcm_open(&m_h_alsa, device.c_str(), SND_PCM_STREAM_CAPTURE, m_alsa_non_blocking)) < 0) { // SND_PCM_NONBLOCK
			fprintf (stderr, "%s: cannot open audio device %s (%s)\n", __func__, device.c_str(), snd_strerror(err));
			return false;
		}

		snd_pcm_info_alloca(&m_alsa_info);
		if ((err = snd_pcm_info(m_h_alsa, m_alsa_info)) < 0) { // SND_PCM_NONBLOCK
			fprintf (stderr, "%s: cannot get pcm info %s (%s)\n", __func__, snd_strerror(err));
			return false;
		}
	
		if (m_alsa_non_blocking && (err = snd_pcm_nonblock(m_h_alsa, 1)) < 0) {
			fprintf (stderr, "%s: snd_pcm_nonblock failed %s (%s)\n", __func__, snd_strerror(err));
			return false;
		}

		alsa_ok = true;

		return alsa_ok;
	}

	bool alsa_init() {

		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		int err;
		unsigned buffer_time = 0;
		unsigned period_time = 0;
		snd_pcm_hw_params_t *hw_params;
		snd_pcm_sw_params_t *sw_params;

		snd_pcm_hw_params_alloca(&hw_params);
		snd_pcm_sw_params_alloca(&sw_params);
		snd_pcm_uframes_t buffer_frames;
		snd_pcm_uframes_t buffer_size;

		snd_pcm_uframes_t chunk_size;
		snd_pcm_uframes_t period_frames;

		if(!alsa_ok) {
			fprintf (stderr, "%s: alsa_ok set false in alsa_open (%s)\n", __func__);

		} else if ((err = snd_pcm_hw_params_any(m_h_alsa, hw_params)) < 0) {
			fprintf (stderr, "%s: cannot initialize hardware parameter (%s)\n", __func__, snd_strerror (err));

		} else if ((err = snd_pcm_hw_params_set_access(m_h_alsa, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
			fprintf (stderr, "%s: cannot set access type (%s)\n", __func__, snd_strerror(err));

		} else if ((err = snd_pcm_hw_params_set_format(m_h_alsa, hw_params, m_format)) < 0) {
			fprintf (stderr, "%s: cannot set sample format (%s)\n", __func__, snd_strerror(err));

		} else if ((err = snd_pcm_hw_params_set_channels(m_h_alsa, hw_params, m_channels)) < 0) {
			fprintf (stderr, "%s: cannot set channel count (%s)\n", __func__, snd_strerror(err));

		} else if((err = snd_pcm_hw_params_set_rate_near(m_h_alsa, hw_params, &m_rate, 0)) < 0) {
			fprintf (stderr, "%s: cannot set sample rate (%s)\n", __func__, snd_strerror(err));

		} else if((err = snd_pcm_hw_params_get_buffer_time_max(hw_params, &buffer_time, 0)) < 0) {
			fprintf (stderr, "%s: snd_pcm_hw_params_get_buffer_time_max failed (%s)\n", __func__);
		}

		if(buffer_time > 500000)
			buffer_time = 500000;
		
		if (buffer_time > 0)
			period_time = buffer_time / 4;
		else
			period_frames = buffer_frames / 4;

		if (period_time > 0)
			err = snd_pcm_hw_params_set_period_time_near(m_h_alsa, hw_params,
							     &period_time, 0);
		else
			err = snd_pcm_hw_params_set_period_size_near(m_h_alsa, hw_params,
							     &period_frames, 0);
		assert(err >= 0);
		if (buffer_time > 0) {
			err = snd_pcm_hw_params_set_buffer_time_near(m_h_alsa, hw_params,
							     &buffer_time, 0);
		} else {
			err = snd_pcm_hw_params_set_buffer_size_near(m_h_alsa, hw_params,
							     &buffer_frames);
		} 

		int bits_per_sample = snd_pcm_format_physical_width(m_format);
		m_alsa_bits_per_frame = bits_per_sample * m_channels;

		if ((err = snd_pcm_hw_params(m_h_alsa, hw_params)) < 0) {
			fprintf (stderr, "%s: cannot set hw parameters (%s)\n", __func__, snd_strerror(err));
			return false;
		}

		if(!m_alsa_frame_size) {
			snd_pcm_hw_params_get_period_size(hw_params, &m_alsa_frame_size, 0);
			//snd_pcm_hw_params_get_buffer_size(hw_params, &buffer_size);
		} 

		if ((err = snd_pcm_sw_params_current(m_h_alsa, sw_params)) < 0) {
			fprintf (stderr, "%s: cannot get current sw parameters (%s)\n", __func__, snd_strerror(err));
			return false;
		}

		snd_pcm_sw_params_set_avail_min(m_h_alsa, sw_params, chunk_size);
		//snd_pcm_sw_params_set_start_threshold(m_h_alsa, sw_params, 1); // TODO
		//snd_pcm_sw_params_set_stop_threshold(m_h_alsa, sw_params, buffer_size);
		
		
		if ((err = snd_pcm_sw_params(m_h_alsa, sw_params)) < 0) {
			fprintf (stderr, "%s: cannot set sw parameters (%s)\n", __func__, snd_strerror(err));
			return false;
		} 

		int buf_size = m_alsa_frame_size * m_alsa_bits_per_frame / 8;
		
		m_alsa_audiobuf = (uint8_t*) malloc(buf_size);
		if(!m_alsa_audiobuf) {
			fprintf (stderr, "%s: not enough memory\n", __func__);
			return false;
		}

		//printf("bits_per_sample: %d\n", bits_per_sample);
		//printf("m_alsa_bits_per_frame: %d\n", m_alsa_bits_per_frame);
		//printf("m_alsa_frame_size: %d\n", m_alsa_frame_size);

		/*
		if ((err = snd_pcm_prepare(m_h_alsa)) < 0) {
			fprintf (stderr, "cannot prepare audio interface for use (%s)\n", snd_strerror (err));
		} else {
			alsa_ok = true;
		}*/

		alsa_ok = true;

		return alsa_ok;
	}

	bool shine_init() {
		
		/*
 		shine_t s;
		shine_config_t config;
		config.wave.channels = PCM_MONO;
		config.mpeg.bitr = m_bitrate;

		if (shine_check_config(config.wave.samplerate, config.mpeg.bitr) < 0)
			fprintf (stderr, "%s: shine_check_config failed\n", __func__);
			return false;
	
		s = shine_initialise(&config); */
		return true;
	}



	bool aac_init() {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

#ifdef AAC_LIB
		if(aacEncOpen(&m_h_aac, 0, m_channels) != AACENC_OK) {
			fprintf (stderr, "%s: m_h_aacOpen failed\n", __func__);

		} else if (aacEncoder_SetParam(m_h_aac, AACENC_AOT, 2) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_AOT failed\n", __func__);

		} else if(aacEncoder_SetParam(m_h_aac, AACENC_SAMPLERATE, m_rate) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_SAMPLERATE failed\n", __func__);

		} else if(aacEncoder_SetParam(m_h_aac, AACENC_CHANNELMODE, 1) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_CHANNELMODE failed\n", __func__);

		} else if(aacEncoder_SetParam(m_h_aac, AACENC_CHANNELORDER, 1) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_CHANNELORDER failed\n", __func__);

		} else if(aacEncoder_SetParam(m_h_aac, AACENC_BITRATE, m_bitrate) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_BITRATE failed\n", __func__);

		} else if(aacEncoder_SetParam(m_h_aac, AACENC_TRANSMUX, TT_MP4_ADTS) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_TRANSMUX failed\n", __func__);
	
		} else if(aacEncoder_SetParam(m_h_aac, AACENC_AFTERBURNER, 1) != AACENC_OK) {
			fprintf (stderr, "%s: AACENC_AFTERBURNER failed\n", __func__);

		} else if(aacEncEncode(m_h_aac, NULL, NULL, NULL, NULL) != AACENC_OK) {
			fprintf (stderr, "%s: aacEncEncode failed\n", __func__);

		} else if(aacEncInfo(m_h_aac, &m_info_aac) != AACENC_OK) {
			fprintf (stderr, "%s: aacEncInfo failed\n", __func__);

		} else {

			printf("maxOutBufBytes: %d\n", m_info_aac.maxOutBufBytes);
			printf("maxAncBytes: %d\n", m_info_aac.maxAncBytes);
			printf("inBufFillLevel: %d\n", m_info_aac.inBufFillLevel);
			printf("inputChannels: %d\n", m_info_aac.inputChannels);
			printf("frameLength: %d\n", m_info_aac.frameLength);
			printf("nDelayCore: %d\n\n\n", m_info_aac.nDelayCore);
			printf("nDelay: %d\n", m_info_aac.nDelay);
			
			m_alsa_frame_size = m_info_aac.frameLength;
			m_out_encbuf_bytes = m_info_aac.maxOutBufBytes;
			m_output_encbuf = (uint8_t*) malloc(m_out_encbuf_bytes * sizeof(uint8_t));

			return true;
		}

#endif

		return false;
	}

	
	bool good() {
		return alsa_ok;
	}

	void shutdown() {
		loop_ok = false;
	}

	unsigned long getMicroTime() {
		struct timeval tv;
		gettimeofday(&tv,NULL);
		//return tv.tv_usec;
		return 1000000 * tv.tv_sec + tv.tv_usec;
	}

	unsigned char* getAACConfBuf() {
#ifdef AAC_LIB	
		return m_info_aac.confBuf;
#else
		return NULL;
#endif
	}


	int getAACConfSize() {
#ifdef AAC_LIB	
		return m_info_aac.confSize;
#else
		return NULL;
#endif	
	}

	int getSampleRate() {
		return m_rate;
	}

	int getSamplesRead() {
		return m_alsa_total_spl;
	}

private:

	void alsa_write_wave_trailor(FILE* fp, int count) {

		uint32_t sub2length, length;

		fseek(fp, 4, SEEK_SET);
		length = count + 36;
		fwrite(&length, sizeof(length), 1, fp);

		fseek(fp, 40, SEEK_SET);
		sub2length = count;
		fwrite(&sub2length, sizeof(sub2length), 1, fp);
	}

	void alsa_write_wave_header(FILE* fp) {

		const char* riff_tag = "RIFF";
		fwrite(riff_tag, sizeof(char), 4, fp);

		uint32_t length = 0;
		fwrite(&length, sizeof(length), 1, fp);

		const char* wav_tag = "WAVE";
		fwrite(wav_tag, sizeof(char), 4, fp);

		const char* fmt_tag = "fmt ";
		fwrite(fmt_tag, sizeof(char), 4, fp);

		uint32_t sublength = 16;
		fwrite(&sublength, sizeof(sublength), 1, fp);

		
		uint16_t format = 1;
		uint16_t channels = m_channels;
		uint32_t sample_rate = m_rate;

		uint16_t byte_p_spl = m_channels * snd_pcm_format_physical_width(m_format) / 8;
		uint32_t byte_p_sec = byte_p_spl * m_rate;
		
		uint16_t bits_p_spl = snd_pcm_format_physical_width(m_format);

		fwrite(&format, sizeof(format), 1, fp);
		fwrite(&channels, sizeof(channels), 1, fp);
		fwrite(&sample_rate, sizeof(sample_rate), 1, fp);

		fwrite(&byte_p_sec, sizeof(byte_p_sec), 1, fp);
		fwrite(&byte_p_spl, sizeof(byte_p_spl), 1, fp);
		fwrite(&bits_p_spl, sizeof(bits_p_spl), 1, fp);

		const char* data_tag = "data";
		fwrite(data_tag, sizeof(char), 4, fp);

		uint32_t sub2length = 0;
		fwrite(&sub2length, sizeof(sub2length), 1, fp);
	}


	bool loop_ok;
	bool alsa_ok;	
	int m_bitrate;
	int m_channels;
	int m_counter;
	unsigned int m_rate;
	snd_pcm_format_t m_format;
	alsamic_sink_t m_sink_type;
	FILE* m_file_sink_out_;
	void* m_userdata;

	size_t m_alsa_total_spl;
	size_t m_alsa_total_bytes;

	snd_pcm_info_t* m_alsa_info;
	
	void (*func_buf_call_ptr)(void*, uint8_t*, int, int); 

	snd_pcm_uframes_t m_alsa_frame_size;
	unsigned int m_alsa_period_time;
	int m_alsa_bits_per_frame;	
	int m_out_encbuf_bytes;

	int m_alsa_non_blocking;
	uint8_t* m_alsa_audiobuf;
	uint8_t* m_output_encbuf;

	snd_pcm_t *m_h_alsa;

#ifdef AAC_LIB
	HANDLE_AACENCODER m_h_aac;
	AACENC_InfoStruct m_info_aac;
#endif

	double audio_time_;
	int counter_;
	int ret_;

	
};

};



