#include <stdio.h>
#include <stdlib.h>

#define FFMPEG_LIB
#include "AlsaMic.h"

rp::AlsaMic* g_mic = NULL;

void sighandler(int signal) {
	printf("Interrupted\n");
	g_mic->shutdown();
}

void func_call(void* userdata, u_char* buffer, int samples, int bytes) {
	fprintf(stderr, "%s(userdata=%x, samples=%x, bytes=%d)\n", __func__, userdata, buffer, samples, bytes);
}

int main (int argc, char *argv[]) {

	signal(SIGINT, sighandler);

	int xx = 1;

	rp::AlsaMic mic(rp::AlsaMic::alsamic_sink_aac2, 1, 44100);
	g_mic = &mic;
	if(mic.alsa_open(argv[1]) && mic.alsa_init()) {
		
		//mic.allocbuf();
		//mic.alsa_pcm2_aac(argv[2]);
		//mic.alsa_pcm2_wav(argv[2]);
		//mic.alsa_pcm2_raw(argv[2]);
		mic.set_call_func(NULL, &func_call);
		mic.alsa_sink_open(argv[2]);
		mic.alsa_start_capture(1);
		mic.alsa_sink_close();
	}
	
	return 0;
  
}
