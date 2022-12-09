#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include "bcm_host.h"

#include "DispManX.h"


#ifndef ALIGN_UP
#define ALIGN_UP(x,y)  ((x + (y)-1) & ~((y)-1))
#endif


namespace rp {

DispManX::DispManX(int img_w, int img_h, int alpha, int display, int draw) : 
	m_top(0), m_left(0), m_width(0), m_height(0), m_pitch(0), m_aligned_height(0),
	m_alpha({ (DISPMANX_FLAGS_ALPHA_T)(DISPMANX_FLAGS_ALPHA_FROM_SOURCE | DISPMANX_FLAGS_ALPHA_FIXED_ALL_PIXELS), 
                             alpha, 0 }), m_type(VC_IMAGE_RGB565) {

	fprintf(stderr, "DispManX::%s()\n", __func__); 
	m_vars.display = vc_dispmanx_display_open(display);

	assert(img_w != 0);
	assert(img_h != 0);

	if(hasDisplay()) {

		int ret = vc_dispmanx_display_get_info(m_vars.display, &m_vars.info);
		assert(ret == 0);
		assert(m_vars.info.width != 0);
		assert(m_vars.info.height != 0);

		m_sx = m_vars.info.width / img_w;
		m_sy = m_vars.info.height / img_h;

		fprintf(stderr, "DispManX::%s(w=%d, h=%d)\n", __func__,  m_vars.info.width, m_vars.info.height); 
		fprintf(stderr, "DispManX::%s(sx=%.2f, sy=%.2f)\n", __func__,  m_sx, m_sy); 

	} else {
		fprintf(stderr, "DispManX::%s(no display)\n", __func__); 
	}
	
}


void DispManX::init(int x1, int y1, int x2, int y2) {

	if(!hasDisplay())
		return;

	m_top = m_sy*y1;
	m_left = m_sx*x1;
	m_width = m_sx*(x2 - x1);
	m_height = m_sy*(y2 - y1);
	
	m_pitch = ALIGN_UP(m_width*2, 32);
	m_aligned_height = ALIGN_UP(m_height, 16);

	m_vars.image = calloc(1, m_pitch * m_height);
	assert(m_vars.image);

	vc_dispmanx_rect_set(&m_dst_rect, 0, 0, m_width, m_height);
	
	m_vars.resource = vc_dispmanx_resource_create(m_type, m_width, m_height, &m_vars.vc_image_ptr);
	assert(m_vars.resource);
	
	int ret = vc_dispmanx_resource_write_data(m_vars.resource, m_type, m_pitch, m_vars.image, &m_dst_rect);


	assert(ret == 0 );
	m_vars.update = vc_dispmanx_update_start(0);
	assert(m_vars.update);


	vc_dispmanx_rect_set(&m_dst_rect, m_left, m_top, m_width, m_height);

	m_vars.element = vc_dispmanx_element_add(m_vars.update,
					m_vars.display, 2000,        // layer
 					&m_dst_rect,
					m_vars.resource,
					&m_src_rect,
					DISPMANX_PROTECTION_NONE,
					&m_alpha,
					NULL,             // clamp
					(DISPMANX_TRANSFORM_T)VC_IMAGE_ROT0);

	ret = vc_dispmanx_update_submit_sync(m_vars.update);
	assert( ret == 0 ); 
}


void DispManX::fillRect(int x, int y, int w, int h, int color) {

	if(hasDisplay()) {
		fillRect1(x*m_sx, y*m_sy, w*m_sx, h*m_sy, color);
	} 
}

void DispManX::clear() {

	if(hasDisplay()) {
		memset(m_vars.image, 0, m_pitch * m_height);
	}
}  


void DispManX::start() {
		
	if(hasDisplay()) {

	}
}

void DispManX::finalize() { 

	if(hasDisplay()) {

		int ret;

		vc_dispmanx_rect_set(&m_dst_rect, 0, 0, m_width, m_height);
		ret = vc_dispmanx_resource_write_data(m_vars.resource,
		                                    m_type,
		                                    m_pitch,
		                                    m_vars.image,
		                                    &m_dst_rect);

		assert(ret == 0 );
		m_vars.update = vc_dispmanx_update_start(0);
		assert(m_vars.update);
	
		vc_dispmanx_rect_set(&m_dst_rect, m_left, m_top, m_width, m_height); 
		ret = vc_dispmanx_element_remove(m_vars.update, m_vars.element);
		assert( ret == 0 );

		m_vars.element = vc_dispmanx_element_add(m_vars.update,
		                                        m_vars.display,
		                                        2000,               // layer
		                                        &m_dst_rect,
		                                        m_vars.resource,
		                                        &m_src_rect,
		                                        DISPMANX_PROTECTION_NONE,
		                                        &m_alpha,
		                                        NULL,             // clamp
		                                        (DISPMANX_TRANSFORM_T)VC_IMAGE_ROT0);

		ret = vc_dispmanx_update_submit_sync(m_vars.update);
		assert( ret == 0 );

	}
}

DispManX::~DispManX() {
	fprintf(stderr, "DispManX::%s()\n", __func__); 

	if(!hasDisplay()) return;

	m_vars.update = vc_dispmanx_update_start(0);
	assert( m_vars.update );
	int ret = vc_dispmanx_element_remove(m_vars.update, m_vars.element);
	assert( ret == 0 );
	ret = vc_dispmanx_update_submit_sync( m_vars.update );
	assert( ret == 0 );
	ret = vc_dispmanx_resource_delete( m_vars.resource );
	assert( ret == 0 );
	ret = vc_dispmanx_display_close( m_vars.display );
	assert( ret == 0 );
}

void DispManX::call_bcm_init() {
	bcm_host_init();
}

int DispManX::getDispWidth() {
	if(hasDisplay()) {
		return m_vars.info.width;
	} else {
		return 1;
	}
}

int DispManX::getDispHeight() {
	if(hasDisplay()) {
		return m_vars.info.height;
	} else {
		return 1;
	}
}

bool DispManX::hasDisplay() {
	return m_vars.display;
}

void DispManX::test() {
	std::cout << "hello dispmanx" << std::endl;
}

void DispManX::fillRect1(int x, int y, int w, int h, int val) {
	
	assert(x <= m_width && y <= m_height && x + w <= m_width && y + h <= m_height);

	int row;
	int col;
	uint16_t *line = (uint16_t *) m_vars.image + y * (m_pitch>>1) + x;

	for ( row = 0; row < h; row++ ) {

		for ( col = 0; col < w; col++ ) {
		    line[col] = val;
		}
		line += (m_pitch>>1);
	}
}

}


/*
int main() {

	//dm.call_bcm_init();

	bcm_host_init();

	rp::DispManX dm;
	
	dm.init(0, 0);
	getchar();
	dm.fillRect(20, 20, 100, 100, 0x001F);
	dm.finalize();
	getchar();
	dm.clear();
	dm.fillRect(50, 50, 100, 100, 0x001F);
	
	dm.fillRect(100, 50, 100, 100, 0xD01F);


	dm.fillRect(50, 1000, 100, 100, 0x0D1F);
	dm.finalize();

	getchar();

} */


