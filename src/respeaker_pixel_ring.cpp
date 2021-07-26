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

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "respeaker_pixel_ring.h"

#define VID_RESPEAKER	0x2886
#define PID_RESPEAKER	0x0018

#define USB_TIMEOUT 	1000

#define READ_DOAANGLE		21
#define READ_VOICEACTIVITY	19

RespeakerPixelRing::RespeakerPixelRing():
	_dh(NULL)
{
}

RespeakerPixelRing::~RespeakerPixelRing()
{
	if (_dh) {
		CloseDevice();
	}
}

int RespeakerPixelRing::OpenDevice()
{
	if (_dh) {
		printf("ERROR: Respeaker already open\n");
		return -1;
	}

	int r = libusb_init(NULL);
	if (r < 0) {
		printf("ERROR: Cannot init libusb\n");
		return r;
	}

	_dh = libusb_open_device_with_vid_pid(NULL, VID_RESPEAKER, PID_RESPEAKER);
    if (!_dh) {
        printf("ERROR: Cannot open Respeaker\n");
        return -1;
    }
    return 0;
}

void RespeakerPixelRing::CloseDevice()
{
	if (!_dh) {
		return;
	}
	libusb_close(_dh);
	_dh = NULL;

	libusb_exit(NULL);
}

int RespeakerPixelRing::Open()
{
	return OpenDevice();
}

void RespeakerPixelRing::Close()
{
	return CloseDevice();
}

int RespeakerPixelRing::SetRingMode(pixel_ring_mode mode, uint8_t *data, int data_len)
{
	if (!_dh) {
		return -100;
	}
	uint8_t dummy[] = {0};

	int ret = libusb_control_transfer(
				_dh,
				LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
				0,
				mode,
				0x1c,
				data? data: dummy,
				data? data_len: 1,
				USB_TIMEOUT);
	if (ret < 0) {
		printf("ERROR: Failed to write to Respeaker: 0x%08x (%d)\n", ret, ret);
		return ret;
	}
	printf("Sent ring mode command to Respeaker\n");
	return 0;
}

int RespeakerPixelRing::ReadVAD()
{
	if (!_dh) {
		return -100;
	}
	int value = 0;

	//    'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),

	int ret = libusb_control_transfer(
			_dh,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			0,
			0xc0 | 32,
			READ_VOICEACTIVITY,
			(unsigned char*)&value, sizeof(value),
			USB_TIMEOUT);
	if (ret < 0) {
		printf("ERROR: Failed to read VAD status\n");
		return ret;
	}
	return value;
}

int RespeakerPixelRing::ReadDOA()
{
	if (!_dh) {
		return -100;
	}
	int value = 0;

	//    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')

	int ret = libusb_control_transfer(
			_dh,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			0,
			0xc0 | 0,
			READ_DOAANGLE,
			(unsigned char*)&value, sizeof(value),
			USB_TIMEOUT);
	if (ret < 0) {
		printf("ERROR: Failed to read DOA status\n");
		return ret;
	}
	return value;
}

