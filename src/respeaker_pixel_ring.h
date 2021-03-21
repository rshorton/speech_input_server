
#include <libusb-1.0/libusb.h>

class RespeakerPixelRing
{
public:
	typedef enum {
		pixel_ring_mode_trace = 0,
		pixel_ring_mode_mono = 1,
		pixel_ring_mode_listen = 2,
		pixel_ring_mode_speak = 3,
		pixel_ring_mode_think = 4,
		pixel_ring_mode_spin = 5,
		pixel_ring_mode_customize = 6
	} pixel_ring_mode;

	RespeakerPixelRing();
	~RespeakerPixelRing();

	int Open();
	void Close();

	int SetRingMode(pixel_ring_mode mode, uint8_t *data = NULL, int data_len = 0);
	int ReadVAD();
	int ReadDOA();

private:
	int OpenDevice();
	void CloseDevice();

private:
	struct libusb_device_handle *_dh;

};
