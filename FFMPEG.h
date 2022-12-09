#ifndef FFMPEG_x_h
#define FFMPEG_x_h


#include <stdexcept>
#include <sys/stat.h>

extern "C" {
	#include <libavutil/opt.h>
	#include <libavutil/time.h>
	#include <libavutil/mathematics.h>
	#include <libavformat/avformat.h>
	#include <libavcodec/avcodec.h>
	#include <libswscale/swscale.h>
	#include <libswresample/swresample.h>
}


#define STREAM_DURATION   10.0
#define SCALE_FLAGS SWS_BICUBIC

#ifndef _FILENAME_
#define _FILENAME_	(strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__ )
#endif


namespace rp {

// a wrapper around a single output AVStream
typedef struct {
    AVStream *st;
    AVCodecContext *enc;

    /* pts of the next frame that will be generated */
    int64_t next_pts;
    int samples_count;

    AVFrame *frame;
    AVFrame *tmp_frame;

    float t, tincr, tincr2;

    struct SwsContext *sws_ctx;
    struct SwrContext *swr_ctx;
} OutputStream;

typedef struct {
	uint8_t data[64];
	int size;
} ConfigNAL;

class FFMPEG {

public:

	FFMPEG(int width, int height, int framerate, int vbitrate=90000, int abitrate=64000, int samplerate=44100) : 	
	m_framerate(framerate), m_vbitrate(vbitrate), m_abitrate(abitrate), m_samplerate(samplerate),
	avdict_opt_(NULL), oc_(NULL), aviobuf_(NULL),
	m_width(width), m_height(height) {

		fprintf(stderr, "%s:%s(framerate=%d, bitrate=%d)\n", _FILENAME_, __func__, framerate, vbitrate);

		av_register_all();
		
		//aviobuf_ = (unsigned char*) av_malloc(32768);
		//avio_ = avio_alloc_context(aviobuf_, 32768, 1, this, NULL, ffm_write_buffer, ffm_seek_buffer);

		// hw encoder libavcodec/v4l2_m2m.c
		// TODO crash on matroskaenc.c file mkv_write_header() -> mkv_write_tracks(1) -> mkv_write_track(5) -> mkv_write_codecprivate(5) -> mkv_write_native_codecprivate(4) -> avc.c::ff_isom_write_avcc(3)

	}

	void setLogging(bool logging) {
		if(logging) {
			av_log_set_level(AV_LOG_DEBUG);
		} else {
			av_log_set_level(AV_LOG_QUIET);
		}
	}

	virtual ~FFMPEG() {
		av_free(aviobuf_);
	}

	bool should_allow_addmovie() {
		return oc_ == NULL;
	}

	void addMovie(const std::string& filename, uint8_t* spspps, int nal_size) {
		fprintf(stderr, "%s:%s(%s)\n", _FILENAME_, __func__, filename.c_str());

		avformat_alloc_output_context2(&oc_, NULL, NULL, filename.c_str());
		if (!oc_) {
			throw std::runtime_error("avformat_alloc_output_context2 failed");
		}
		AVOutputFormat *fmt = oc_->oformat;
		//oc_->pb = avio_;

		fmt->video_codec = AV_CODEC_ID_H264;
		fmt->audio_codec = AV_CODEC_ID_PCM_S16LE;
		//printf("Video encoder>>>>> %s\n", avcodec_get_name(fmt->video_codec));
		//printf("Audio encoder>>>>> %s\n", avcodec_get_name(fmt->audio_codec));

		memset(&video_st_, 0, sizeof(video_st_));
		memset(&audio_st_, 0, sizeof(audio_st_));

		
		
		AVCodecContext *c = add_stream(&video_st_, oc_, &video_codec_, AVMEDIA_TYPE_VIDEO, fmt->video_codec);

		if(c->codec_id == AV_CODEC_ID_H264) {
			c->extradata_size = nal_size;
			c->extradata = (uint8_t*) av_mallocz(c->extradata_size);
			memcpy(c->extradata, spspps, nal_size);
		}

		if(video_codec_ && fmt->video_codec != AV_CODEC_ID_H264) {
			open_video(oc_, video_codec_, &video_st_, NULL);
		}

		/* copy the stream parameters to the muxer */
		if (avcodec_parameters_from_context(video_st_.st->codecpar, c) < 0) {
			throw std::invalid_argument("Could not copy the stream parameters");
		}
	}


	void addAudio() {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		if(!oc_) {
			throw std::runtime_error("Call addVideo first");
		}

		AVOutputFormat *fmt = oc_->oformat;
		AVCodecContext *c = add_stream(&audio_st_, oc_, &audio_codec_, AVMEDIA_TYPE_AUDIO, fmt->audio_codec);

		if(audio_codec_ && fmt->audio_codec != AV_CODEC_ID_PCM_S16LE) {
			open_audio(oc_, audio_codec_, &audio_st_, NULL);
		}

		/* copy the stream parameters to the muxer */
		if (avcodec_parameters_from_context(audio_st_.st->codecpar, c) < 0) {
			throw std::invalid_argument("Could not copy the stream parameters");
		}
	}

	void writeHeader(const std::string& filename) {
		fprintf(stderr, "%s:%s(%s)\n", _FILENAME_, __func__, filename.c_str());

		size_t pos = 0;
		struct stat sb = { 0 };
		if((pos = filename.find_last_of("/")) != std::string::npos) {
			
			if(stat(filename.substr(0, pos).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
				// directory exist - ok
			} else {
				fprintf(stderr, ">>> %s:%s(Creating dir: %s)\n", _FILENAME_, __func__, filename.substr(0, pos).c_str());
				mkdir(filename.substr(0, pos).c_str(), 0755);				
			}
		}


		/* open the output file, if needed */
		if (!(oc_->oformat->flags & AVFMT_NOFILE)) {
			if (avio_open(&oc_->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0) {
				throw std::runtime_error("Could not open output file");
			}
		}

		//printf(">>>vvvvvv_ %d	%d\n", video_st_.enc->time_base.num, video_st_.enc->time_base.den);
		//printf(">>>vvvvvv_ %d	%d\n\n", video_st_.st->time_base.num, video_st_.st->time_base.den);

		AVDictionary* opts = oc_->metadata;//new AVDictionary(NULL);
		av_dict_set(&opts, "title", "Home cctv0", 0);

		if (avformat_write_header(oc_, &opts) < 0) {
			throw std::runtime_error("avformat_write_header failed");
		}

		m_time_start = av_gettime_relative();

		//printf(">>>vvvvvv %d	%d\n", video_st_.enc->time_base.num, video_st_.enc->time_base.den);
		//printf(">>>vvvvvv %d	%d\n\n", video_st_.st->time_base.num, video_st_.st->time_base.den);
	}

	void endMovie() {

		if(oc_) {

			fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);
			av_write_trailer(oc_);

			avcodec_free_context(&video_st_.enc);
			av_frame_free(&video_st_.frame);
			av_frame_free(&video_st_.tmp_frame);
			sws_freeContext(video_st_.sws_ctx);
			swr_free(&video_st_.swr_ctx);

			avcodec_free_context(&audio_st_.enc);
			av_frame_free(&audio_st_.frame);
			av_frame_free(&audio_st_.tmp_frame);
			sws_freeContext(audio_st_.sws_ctx);
			swr_free(&audio_st_.swr_ctx);
			avformat_free_context(oc_);
			oc_ = NULL;

		} /*else {
			fprintf(stderr, "%s:%s(NO_MOVIE_ADDED - are you missing -m4v ?)\n", _FILENAME_, __func__);
		}*/
	}

	int write_video_frame(uint8_t* inbuf, int count, bool iframe) {
		//fprintf(stderr, "%s(config=%d, iframe=%d)\n", __func__, config, iframe);

		AVPacket pkt = { 0 };
		AVCodecContext *c = video_st_.enc;
		av_init_packet(&pkt);
		if(iframe) {
			pkt.flags = AV_PKT_FLAG_KEY;
		}

		/*
		int ret;	
		int got_packet = 1;	
		AVFrame *frame;
		fprintf(stderr, ">>> avcodec_send_frame\n");
		frame = get_video_frame(&video_st_);
		if(avcodec_send_frame(c, frame) < 0) {
			fprintf(stderr, "%s avcodec_send_frame failed\n", __func__);
			return -1;
		}

		while(true) {

			fprintf(stderr, ">>> avcodec_receive_packet\n");

			ret = avcodec_receive_packet(c, &pkt);
			if(!ret) {
				fprintf(stderr, ">>> write_frame\n");
				ret = write_frame(oc_, &c->time_base, video_st_.st, &pkt);

			} else if(ret == AVERROR(EAGAIN)) {
				fprintf(stderr, "%s AVERROR\n", __func__);
				break;
			} else if(ret < 0) {
				fprintf(stderr, "%s ret < 0\n", __func__); 
				return -1;
			} else {
				fprintf(stderr, "%s ?? 0\n", __func__); 
				break;
			}
		}
		return 0;*/


		//double frameTime;
		//double frameDuration;
		//frameDuration = ((double)video_st_.st->avg_frame_rate.den) / m_framerate;
		

		//frameTime = ++video_st_.next_pts * frameDuration;
		
		//pkt.pts = pkt.dts = frameTime / video_st_.st->avg_frame_rate.num;
		//pkt.duration = frameDuration;

		//AVRational framerate = { m_framerate, 1 }; 
		//int64_t next_dts = av_rescale_q(video_st_.next_pts, AV_TIME_BASE_Q, av_inv_q(framerate));
                //video_st_.next_pts = av_rescale_q(next_dts + 1, av_inv_q(framerate), AV_TIME_BASE_Q);


		//int64_t next_dts = av_rescale_q(video_st_.next_pts, video_st_.st->time_base, c->time_base);
                //video_st_.next_pts = av_rescale_q(next_dts + 1, c->time_base, video_st_.st->time_base);


		video_st_.next_pts++;// = (av_gettime_relative() - m_time_start)/1000; // video_st_.next_pts + 1;
		//pkt.pts = pkt.dts = av_rescale_q(video_st_.next_pts, c->time_base, video_st_.st->time_base);
		pkt.pts = pkt.dts = av_rescale_q(video_st_.next_pts, c->time_base, video_st_.st->time_base);
		//pkt.pts = AV_NOPTS_VALUE;

		//pkt.duration = 612;//video_st_.st->time_base.den / m_framerate;
		//frameTime = ++video_st_.next_pts;
		//pkt.pts = pkt.dts = video_st_.next_pts;// * c->time_base.num / c->time_base.den;
		//pkt.pts = av_rescale_q(pkt.pts, c->time_base, video_st_.st->time_base);
		//pkt.dts = av_rescale_q(pkt.dts, c->time_base, video_st_.st->time_base);
		pkt.stream_index = video_st_.st->index;
		pkt.size = count;
		pkt.data = inbuf;

			
		//av_packet_rescale_ts(&pkt, c->time_base, video_st_.st->time_base);
		//printf(">>>v %d/%d	%d\n", c->time_base.num, c->time_base.den, iframe);
		//printf(">>>v %d/%d\n\n", video_st_.st->time_base.num, video_st_.st->time_base.den);
		//printf("vvvvvvvvvvvvvvvvvv %"PRId64", %"PRId64"\n", pkt.pts, pkt.dts);

		int ret;
		if((ret = av_interleaved_write_frame(oc_, &pkt)) < 0) {
			fprintf(stderr, "Error while writing video frame: %d\n", ret);
			return -1;
		}

		av_packet_unref(&pkt);
		/*
		for(int i=0; i < std::min(100, in_len); ++i) {
			printf("%02x, ", pkt.data[i]);
		}
		printf("\n\n");*/
	
		return count;
	}

	int write_audio_frame(uint8_t* inbuf, int samples, int count) {
		//fprintf(stderr, "%s(samples=%d, bytes=%d)\n", __func__, samples, count);

	
		AVPacket pkt = { 0 }; // data and size must be 0;
		av_init_packet(&pkt);
		AVCodecContext *c = audio_st_.enc;

		audio_st_.next_pts = audio_st_.next_pts + samples;
		pkt.pts = pkt.dts = av_rescale_q(audio_st_.next_pts, c->time_base, audio_st_.st->time_base);
		//pkt.pts = AV_NOPTS_VALUE;

		pkt.stream_index = audio_st_.st->index;
		pkt.data = inbuf;
		pkt.size = count;

		//printf(">>>a %d/%d\n", c->time_base.num, c->time_base.den);
		//printf(">>>a %d/%d\n\n", audio_st_.st->time_base.num, audio_st_.st->time_base.den);
		//printf("aaaaaaaaaaaaaaaaaa %"PRId64", %"PRId64"\n", pkt.pts, pkt.dts);

		int ret;
		if((ret = av_interleaved_write_frame(oc_, &pkt)) < 0) {
			fprintf(stderr, "Error while writing audio frame: %d\n", ret);
			return -1;
		}

		return count;
	}

	bool compare_time() {
		int diff = av_compare_ts(video_st_.next_pts, video_st_.enc->time_base, audio_st_.next_pts, audio_st_.enc->time_base);
		printf("diff = %d	%d\n", diff, video_st_.next_pts);
		return diff > 0;
	}


	int (*func_writer_callback)(uint8_t*, int); 

	int64_t (*func_seek_callback)(int64_t, int); 

private:


	void open_video(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg) {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		int ret;
		AVCodecContext *c = ost->enc;
		AVDictionary *opt = NULL;
		av_dict_copy(&opt, opt_arg, 0);

		/* open the codec */
		ret = avcodec_open2(c, codec, NULL);
		av_dict_free(&opt);
		if (ret < 0) {
			char buf[128];
			snprintf(buf, 128, "Could not open video codec: %d", ret);
			throw std::invalid_argument(buf);
		}

		/*
		for(int i=0; i < c->extradata_size; ++i) {
			printf("%02x, ", c->extradata[i]);
		}
		printf("\n>>>>>>> extradata=%d\n", c->extradata_size);
		*/

		/* allocate and init a re-usable frame */

		/*
		ost->frame = av_frame_alloc();
		if (!ost->frame) {
			throw std::runtime_error("Could not allocate video frame");
		}

		ost->frame->format = c->pix_fmt;
		ost->frame->width  = c->width;
		ost->frame->height = c->height;

		ret = av_frame_get_buffer(ost->frame, 32);
		if (ret < 0) {
			throw std::runtime_error("Could not allocate frame data");
		} */

	}

	void open_audio(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg) {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		AVCodecContext *c;
		int nb_samples;
		int ret;
		AVDictionary *opt = NULL;

		c = ost->enc;

		/* open it */
		av_dict_copy(&opt, opt_arg, 0);
		ret = avcodec_open2(c, codec, &opt);
		av_dict_free(&opt);
		if (ret < 0) {
			char buf[128];
			snprintf(buf, 128, "Could not open video codec: %d", ret);
			throw std::invalid_argument(buf);
		}

		/* init signal generator */
		ost->t     = 0;
		ost->tincr = 2 * M_PI * 110.0 / c->sample_rate;
		/* increment frequency by 110 Hz per second */
		ost->tincr2 = 2 * M_PI * 110.0 / c->sample_rate / c->sample_rate;

		if (c->codec->capabilities & AV_CODEC_CAP_VARIABLE_FRAME_SIZE)
			nb_samples = 10000;
		else
			nb_samples = c->frame_size;

		ost->frame = av_frame_alloc();
		ost->tmp_frame = av_frame_alloc();

		if (!ost->frame || !ost->tmp_frame) {
			throw std::runtime_error("Error allocating an audio frame");
		}

		ost->frame->format = c->sample_fmt;
		ost->frame->channel_layout = c->channel_layout;
		ost->frame->sample_rate = c->sample_rate;
		ost->frame->nb_samples = nb_samples;

		if (nb_samples) {
			ret = av_frame_get_buffer(ost->frame, 0);
			if (ret < 0) {
				throw std::runtime_error("Error allocating an audio buffer");
			}
		}


		ost->tmp_frame->format = c->sample_fmt;
		ost->tmp_frame->channel_layout = c->channel_layout;
		ost->tmp_frame->sample_rate = c->sample_rate;
		ost->tmp_frame->nb_samples = nb_samples;


		/* create resampler context */
		ost->swr_ctx = swr_alloc();
		if (!ost->swr_ctx) {
			throw std::runtime_error("Could not allocate resampler context");
		}

		/* set options */
		av_opt_set_int       (ost->swr_ctx, "in_channel_count",   c->channels,       0);
		av_opt_set_int       (ost->swr_ctx, "in_sample_rate",     c->sample_rate,    0);
		av_opt_set_sample_fmt(ost->swr_ctx, "in_sample_fmt",      AV_SAMPLE_FMT_S16, 0);
		av_opt_set_int       (ost->swr_ctx, "out_channel_count",  c->channels,       0);
		av_opt_set_int       (ost->swr_ctx, "out_sample_rate",    c->sample_rate,    0);
		av_opt_set_sample_fmt(ost->swr_ctx, "out_sample_fmt",     c->sample_fmt,     0);

		/* initialize the resampling context */
		if ((ret = swr_init(ost->swr_ctx)) < 0) {
			throw std::invalid_argument("Failed to initialize the resampling context");
		}
	}

	static int ffm_write_buffer(void* data, uint8_t *buf,  int bufsize) {
		FFMPEG* ff = (FFMPEG*)data;
		return ff->func_writer_callback(buf, bufsize);
	}

	static int64_t ffm_seek_buffer(void* data, int64_t offset,  int whence) {
		FFMPEG* ff = (FFMPEG*)data;
		return ff->func_seek_callback(offset, whence);
	}

	AVCodecContext* add_stream(OutputStream *ost, AVFormatContext *oc,  AVCodec **codec, 
			enum AVMediaType codec_type, enum AVCodecID codec_id) {
		fprintf(stderr, "%s:%s()\n", _FILENAME_, __func__);

		AVCodecContext *c;
		
		int i;
		//const uint8_t pps[] = { 0x28, 0xee, 0x1f, 0x2c };	
		//const uint8_t sps[] = { 0x27, 0x64, 0x00, 0x28, 0xac, 0x2b, 0x40, 0x3c, 0x01, 0x13, 0xf2, 0xc0, 0x3c, 0x48, 0x9a, 0x80 };
		//const uint8_t spspps[] = { 0x00, 0x00, 0x00, 0x01, 0x27, 0x64, 0x00, 0x28, 0xac, 0x2b, 0x40, 0x3c, 0x01, 0x13, 0xf2, 0xc0, 0x3c, 0x48, 0x9a, 0x80, 0x00, 0x00, 0x00, 0x01, 0x28, 0xee, 0x1f, 0x2c, 0x00 };

		
		// 0x00, 0x00, 0x00, 0x01, 
		/* find the encoder */
		*codec = avcodec_find_encoder(codec_id);
		if (!(*codec) && codec_id != AV_CODEC_ID_NONE) {
			char buf[128];
			snprintf(buf, 128, "Could not find encoder for '%s'", avcodec_get_name(codec_id));
			throw std::invalid_argument(buf);
		}

		ost->st = avformat_new_stream(oc, NULL);
		if (!ost->st) {
			throw std::runtime_error("Could not allocate stream");
		}

		ost->next_pts = 0;
		ost->st->id = oc->nb_streams-1;
		c = avcodec_alloc_context3(*codec);

		if (!c) {
			throw std::runtime_error("Could not alloc an encoding context");
		}
		
		c->codec_type = codec_type;

		ost->enc = c;
		switch (codec_type) {
			case AVMEDIA_TYPE_AUDIO:				
				c->bit_rate    = m_abitrate;
				c->sample_rate = m_samplerate;
				c->sample_fmt  = AV_SAMPLE_FMT_S16;
				c->channel_layout = AV_CH_LAYOUT_MONO;
				c->channels = av_get_channel_layout_nb_channels(c->channel_layout);
				ost->st->time_base = (AVRational){ 1, c->sample_rate };
				c->time_base       = ost->st->time_base;
				break;

			case AVMEDIA_TYPE_VIDEO:
				c->codec_id = codec_id;
				c->bit_rate = m_vbitrate;

				ost->st->time_base = (AVRational){ 1, m_framerate };
				c->time_base       = ost->st->time_base;
				c->flags = 0;
				c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
				c->pix_fmt       = AV_PIX_FMT_YUV420P;
				c->width = m_width;
				c->height = m_height;

				switch(c->codec_id) {
					case AV_CODEC_ID_MPEG2VIDEO:
						/* just for testing, we also add B-frames */
						c->max_b_frames = 2;
						break;
					case AV_CODEC_ID_MPEG1VIDEO:
						/* Needed to avoid using macroblocks in which some coeffs overflow.
						* This does not happen with normal video, it just happens here as
						* the motion of the chroma plane does not match the luma plane. */
						c->mb_decision = 2;
						break;
					case AV_CODEC_ID_H264:

						

						/*
						c->extradata_size = 7 + sizeof(pps) + sizeof(sps) + sizeof(spsSize) + sizeof(ppsSize);
						c->extradata = extra = (uint8_t*) av_mallocz(c->extradata_size);
						extra[0] = 1;
						extra[1] = 100; // profile
						extra[2] = 0; // compatibility
						extra[3] = 40; // level	
						extra[4] = 0xFC | 3; // res	
						extra[5] = 0xE0 | 1; // num sps = 1
						extra += 6; 	
						memcpy(extra, &spsSize, sizeof(spsSize)); extra += sizeof(spsSize);
						memcpy(extra, sps, sizeof(sps)); extra += sizeof(sps);	
						*extra++ = 1;
						memcpy(extra, &ppsSize, sizeof(ppsSize)); extra += sizeof(ppsSize);
						memcpy(extra, pps, sizeof(pps)); */
						break;
				}
				break;

			default:
				break;
		}

		//printf(">>>xxxx %d	%d\n", c->time_base.num, c->time_base.den);
		//printf(">>>xxxx %d	%d\n\n", ost->st->time_base.num, ost->st->time_base.den);

		/* Some "container" formats (mov, mkv but not mp4) want stream headers to be separate. */
		if (oc->oformat->flags & AVFMT_GLOBALHEADER) {
			c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
		}

		return c;
	}

	int write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt) {
		/* rescale output packet timestamp values from codec to stream timebase */
		av_packet_rescale_ts(pkt, *time_base, st->time_base);
		pkt->stream_index = st->index;

		/* Write the compressed frame to the media file. */
		return av_interleaved_write_frame(fmt_ctx, pkt);
	}

	

	


	
	
/* Prepare a dummy image. */
static void fill_yuv_image(AVFrame *pict, int frame_index,
                           int width, int height)
{
    int x, y, i;

    i = frame_index;

    /* Y */
    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
            pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;

    /* Cb and Cr */
    for (y = 0; y < height / 2; y++) {
        for (x = 0; x < width / 2; x++) {
            pict->data[1][y * pict->linesize[1] + x] = 128 + y + i * 2;
            pict->data[2][y * pict->linesize[2] + x] = 64 + x + i * 5;
        }
    }
}


/* Prepare a 16 bit dummy audio frame of 'frame_size' samples and
 * 'nb_channels' channels. */
static AVFrame *get_audio_frame(OutputStream *ost) {
    AVFrame *frame = ost->tmp_frame;
	
    int j, i, v;
    int16_t *q = (int16_t*)frame->data[0];

	

    /* check if we want to generate more frames */
    if (av_compare_ts(ost->next_pts, ost->enc->time_base,
                      STREAM_DURATION, (AVRational){ 1, 1 }) >= 0)
        return NULL;

	
	
    for (j = 0; j <frame->nb_samples; j++) {
        v = (int)(sin(ost->t) * 10000);
        for (i = 0; i < ost->enc->channels; i++) {
		//printf("i = %d, ", i);
           // *q++ = v;
	}

        ost->t     += ost->tincr;
        ost->tincr += ost->tincr2;
    } 

    frame->pts = ost->next_pts;
    ost->next_pts  += frame->nb_samples;

    return frame;
}

static AVFrame *get_video_frame(OutputStream *ost) {
    AVCodecContext *c = ost->enc;

    /* check if we want to generate more frames */
    if (av_compare_ts(ost->next_pts, c->time_base,
                      STREAM_DURATION, (AVRational){ 1, 1 }) >= 0)
        return NULL;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0)
        exit(1);

    if (c->pix_fmt != AV_PIX_FMT_YUV420P) {
        /* as we only generate a YUV420P picture, we must convert it
         * to the codec pixel format if needed */
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          AV_PIX_FMT_YUV420P,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                fprintf(stderr,
                        "Could not initialize the conversion context\n");
                exit(1);
            }
        }
        fill_yuv_image(ost->tmp_frame, ost->next_pts, c->width, c->height);
        sws_scale(ost->sws_ctx, (const uint8_t * const *) ost->tmp_frame->data,
                  ost->tmp_frame->linesize, 0, c->height, ost->frame->data,
                  ost->frame->linesize);
    } else {
        fill_yuv_image(ost->frame, ost->next_pts, c->width, c->height);
    }

    ost->frame->pts = ost->next_pts++;

    return ost->frame;
}


	int m_width;
	int m_height;
	int m_vbitrate;
	int m_abitrate;
	int m_framerate;
	int m_samplerate;
	int64_t m_time_start;
	
	AVFormatContext *oc_;
	OutputStream video_st_;
	OutputStream audio_st_;
	AVCodec *video_codec_;
	AVCodec *audio_codec_;
	AVDictionary *avdict_opt_;

	unsigned char *aviobuf_;
	AVIOContext *avio_;
	ConfigNAL configNAL;

};

};

/*
if (av_compare_ts(video_st_.next_pts, video_st_.enc->time_base, audio_st_.next_pts, audio_st_.enc->time_base) <= 0) {
            		return write_video_frame(inbuf, in_len, iframe, 0);
		} else {
			return 0;//write_audio_frame(inbuf, in_len);
		}

*/

#endif
