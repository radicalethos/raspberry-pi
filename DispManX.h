#ifndef DISPMANX_H
#define DISPMANX_H

namespace rp {

typedef struct {
    DISPMANX_DISPLAY_HANDLE_T   display;
    DISPMANX_MODEINFO_T         info;
    void                       *image;
    DISPMANX_UPDATE_HANDLE_T    update;
    DISPMANX_RESOURCE_HANDLE_T  resource;
    DISPMANX_ELEMENT_HANDLE_T   element;
    uint32_t                    vc_image_ptr;

} RECT_VARS_T;


class DispManX {

public:
	DispManX(int img_w, int img_h, int alpha, int display, int draw);
	virtual ~DispManX();
	void call_bcm_init();
	void init(int x1, int y1, int x2, int y2);
	void test();

	
	int getY1() { return m_top; }
	int getX1() { return m_left; }

	
	int getY2() { return m_top + m_height; }
	int getX2() { return m_left + m_width; }

	int getWidth() { return m_width; }
	int getHeight() { return m_height; }

	int getDispWidth();
	int getDispHeight();
	

	void fillRect(int x, int y, int w, int h, int color);

	bool hasDisplay();

	void clear();
	void start();
	void finalize();

private:

	void fillRect1(int x, int y, int w, int h, int val);

	float m_sx;
	float m_sy;


	int m_top;
	int m_left;
	int m_width;
	int m_height;
	int m_pitch;
	int m_aligned_height;
	
	RECT_VARS_T m_vars;
	VC_RECT_T m_src_rect;
	VC_RECT_T m_dst_rect;
	VC_IMAGE_TYPE_T m_type;
	VC_DISPMANX_ALPHA_T m_alpha;


};

};

#endif
