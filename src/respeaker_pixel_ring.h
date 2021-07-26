/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

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
