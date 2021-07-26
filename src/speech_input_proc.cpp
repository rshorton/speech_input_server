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

#include <string>
#include <iostream>
#include <speechapi_cxx.h>

#include <alsa/asoundlib.h>

#include "rclcpp/rclcpp.hpp"

#include "respeaker_pixel_ring.h"
#include "speech_input_proc.h"

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;

using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;

#define SAMPLE_RATE         16000       // input sampling rate (filters assume this rate)
#define SAMPLE_BITS         16          // 16 bits per sample is the max size for the PDM MIC input
#define NUM_CHANNELS		6			// Respeaker outputs 6 channels
#define FRAMES_PER_READ		160			// Number of audio frames per read

const std::string RESPEAKER_MIC_ARRAY_CARD_NAME = "ReSpeaker 4 Mic Array";

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

const std::string SpeechInputProc::WakeWordDetector_HeyRobot = "HeyRobot";
const std::string SpeechInputProc::WakeWordDetector_HeyAnna = "HeyAnna";
const std::string SpeechInputProc::WakeWordDetector_HeyElsaBot = "HeyElsaBot";

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class AudioCapture
{
public:
	AudioCapture():
		_handle(NULL),
		_mute_input(false)
	{

	}

	~AudioCapture()
	{
		Close();
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int Open(const char* pDevice, int sample_rate)
	{
		if (_handle) {
			return -1;
		}

		int err = -1;
		snd_pcm_sw_params_t *swparams;
		snd_pcm_sw_params_alloca(&swparams);

		do {
			if ((err = snd_pcm_open(&_handle, pDevice, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: cannot open snd_pcm audio device (%s)", snd_strerror(err));
				break;
			}

			if ((err = snd_pcm_set_params(_handle,
							  SND_PCM_FORMAT_S16_LE,
							  SND_PCM_ACCESS_RW_INTERLEAVED,
							  NUM_CHANNELS,
							  sample_rate,
							  0,
							  500000)) < 0) {   /* 0.5sec */
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: cannot set snd_pcm params (%s)", snd_strerror(err));
				break;
			}

			err = snd_pcm_sw_params_current(_handle, swparams);
			if (err < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Unable to determine current swparams (%s)", snd_strerror(err));
				break;
			}
			err = snd_pcm_sw_params_set_start_threshold(_handle, swparams, 1);
			if (err < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Unable to set start threshold mode (%s)", snd_strerror(err));
				break;
			}

			err = snd_pcm_sw_params(_handle, swparams);
			if (err < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Unable to set sw params (%s)", snd_strerror(err));
				break;
			}

			if ((err = snd_pcm_prepare(_handle)) < 0) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Cannot prepare audio interface for use (%s)", snd_strerror(err));
				break;
			}
			return 0;
		} while(0);

		if (_handle) {
			snd_pcm_close(_handle);
			_handle = NULL;
		}
		return err;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int Close()
	{
		if (_handle) {
			snd_pcm_close(_handle);
			_handle = NULL;
		}
		return 0;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	int ReadBlock(uint16_t* read_buffer, int num_frames)
	{
		int cnt = snd_pcm_readi(_handle, read_buffer, num_frames);
		if (cnt < 0) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: read from audio interface failed, (%s)", snd_strerror(cnt));
		} else {
			num_frames = cnt;

			if (_mute_input) {
				memset(read_buffer, 0, num_frames*sizeof(uint16_t));
			} else {
				// Pack the first audio channel to the start of buffer
				for (int j = 0; j < num_frames; j++) {
					read_buffer[j] = read_buffer[j*NUM_CHANNELS];
				}
#if 0
				if (fpFile) {
					fwrite(read_buffer, num_frames*2, 1, fpFile);
					fprintf(stdout, ".");
				}
#endif
			}
		}
		return cnt;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	void SetMute(bool mute)
	{
		_mute_input = mute;
	}

private:
	snd_pcm_t* _handle;
	bool _mute_input;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

enum SpeechDetStatus {
	SpeechDetStatus_Ok = 0,
	SpeechDetStatus_InProg,
	SpeechDetStatus_Error,
	SpeechDetStatus_Done
};

class WakeWordDetector
{
public:
	WakeWordDetector(std::string model_path):
		_model_path(model_path),
		_open(false),
		_enable(false),
		_pushAudioInput(nullptr),
		_recognizer(nullptr),
		_keywordModel(nullptr)
	{
	}
	~WakeWordDetector()
	{
	}

	SpeechDetStatus Open()
	{
		// Keyword recognizer setup
		_pushAudioInput = AudioInputStream::CreatePushStream();
		_recognizer = KeywordRecognizer::FromConfig(AudioConfig::FromStreamInput(_pushAudioInput));
		_keywordModel = KeywordRecognitionModel::FromFile(_model_path);
		_open = true;
		return SpeechDetStatus_Ok;
	}
	SpeechDetStatus Close()
	{
		return SpeechDetStatus_Ok;
	}

	SpeechDetStatus Enable(bool enable)
	{
		if (_open) {
			if (enable && !_enable) {
				_result_future = _recognizer->RecognizeOnceAsync(_keywordModel);
			} else if (!enable && _enable) {
				auto stopFuture = _recognizer->StopRecognitionAsync();
				stopFuture.wait();
			}
			_enable = enable;
			return SpeechDetStatus_Ok;
		}
		return SpeechDetStatus_Error;
	}

	SpeechDetStatus ProcessData(uint8_t *pData, int len)
	{
		if (_open && _enable) {
			_pushAudioInput->Write(pData, len);
			if (_result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
				// Auto-start a new detection
				_result_future = _recognizer->RecognizeOnceAsync(_keywordModel);
				return SpeechDetStatus_Done;
			}
			return SpeechDetStatus_InProg;
		}
		return SpeechDetStatus_Error;
	}

	bool GetResult()
	{
		return false;
	}

private:
	std::string _model_path;
	bool _open;
	bool _enable;
	std::future<std::shared_ptr<KeywordRecognitionResult>> _result_future;
	std::shared_ptr<PushAudioInputStream> _pushAudioInput;
	std::shared_ptr<KeywordRecognizer> _recognizer;
	std::shared_ptr<KeywordRecognitionModel> _keywordModel;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class RecogitionDetector
{
public:
	RecogitionDetector():
		_open(false),
		_running(false),
		_pushAudioInput(nullptr),
		_recognizer(nullptr),
		_result(nullptr)
	{

	}
	~RecogitionDetector()
	{

	}

	SpeechDetStatus Open()
	{
		// Speech recognizer setup
		const char* env_key = std::getenv("MS_COGNTIVE_SUB_KEY");
		const char* env_region = std::getenv("MS_COGNTIVE_SUB_REGION");
		if (!env_key || !env_region) {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: MS Cogntive env vars MS_COGNTIVE_SUB_KEY/MS_COGNTIVE_SUB_REGION not set");
			return SpeechDetStatus_Error;
		}
		auto config = SpeechConfig::FromSubscription(env_key, env_region);
		_pushAudioInput = AudioInputStream::CreatePushStream();
		_recognizer = SpeechRecognizer::FromConfig(config, AudioConfig::FromStreamInput(_pushAudioInput));
		_open = true;
		return SpeechDetStatus_Ok;
	}

	SpeechDetStatus Close()
	{
		return SpeechDetStatus_Ok;
	}

	SpeechDetStatus Start()
	{
		if (!_open) {
			Open();
		}
		if (_open) {
			if (!_running) {
				_result_future = _recognizer->RecognizeOnceAsync();
				_running = true;
			}
			return SpeechDetStatus_Ok;
		}
		return SpeechDetStatus_Error;
	}
	SpeechDetStatus Stop()
	{
		// Not supported by RecognizeOnceAsync mode
		return SpeechDetStatus_Error;
	}

	SpeechDetStatus ProcessData(uint8_t *pData, int len)
	{
		if (_open && _running) {
			_pushAudioInput->Write(pData, len);
			if (_result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
				_running = false;
				_result = _result_future.get();
				return SpeechDetStatus_Done;
			}
			return SpeechDetStatus_InProg;
		}
		// Nothing to do, so OK
		return SpeechDetStatus_Ok;
	}

	bool GetResult(std::string &speech_text)
	{
		if (_open && !_running && _result != nullptr) {
			if (_result->Reason == ResultReason::RecognizedSpeech) {
				speech_text = _result->Text;
				return true;
			}
		}
		return false;
	}

private:
	bool _open;
	bool _running;
	std::future<std::shared_ptr<SpeechRecognitionResult>> _result_future;
	std::shared_ptr<PushAudioInputStream> _pushAudioInput;
	std::shared_ptr<SpeechRecognizer> _recognizer;
	std::shared_ptr<SpeechRecognitionResult> _result;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechInputProc::SpeechInputProc():
		_open(false),
		_run(false)
{
	// fix
	_installed_wake_word_detectors[WakeWordDetector_HeyRobot] = "/home/ubuntu/ms_voice/2788310f-4ac6-4b58-9210-fe3e44f2a6f8.table";
	_installed_wake_word_detectors[WakeWordDetector_HeyAnna] = "/home/ubuntu/ms_voice/2beb4831-53c7-4757-b0ea-1ee44b039266.table";
	_installed_wake_word_detectors[WakeWordDetector_HeyElsaBot] = "/home/ubuntu/ms_voice/dd5eafaf-9222-4ec6-86b8-1c6b1c18cc57.table";
}

SpeechInputProc::~SpeechInputProc()
{
	Close();
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechInputProc::Open()
{
	if (_open) {
		return SpeechProcStatus_Error;
	}

	int cardNum = FindCard(RESPEAKER_MIC_ARRAY_CARD_NAME);
	if (cardNum == -1) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Could not find Respeaker mic device\n");
        return SpeechProcStatus_Error;
	}

	_audio_capture = std::make_unique<AudioCapture>();

	stringstream device;
	device << "hw:" << cardNum;
	int ret = _audio_capture->Open(device.str().c_str(), SAMPLE_RATE);
	if (ret) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Could not open audio capture device (%s)\n", snd_strerror(ret));
        return SpeechProcStatus_Error;
	}

	_mic_led_ring = std::make_unique<RespeakerPixelRing>();
	if (_mic_led_ring != nullptr) {
		do {
			ret = _mic_led_ring->Open();
			if (ret) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR; Could not open Respeaker LED device (%d)\n", ret);
				break;
			}
			_mic_led_ring->SetRingMode(RespeakerPixelRing::pixel_ring_mode_listen);

			// Start thread to do the processing
			_run = true;
			_proc_thread = std::thread{std::bind(&SpeechInputProc::Process, this)};

			return SpeechProcStatus_Ok;
		} while(0);
	}
	_audio_capture->Close();
	_audio_capture.reset();
	return SpeechProcStatus_Error;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechInputProc::Close()
{
	if (!_open) {
		return SpeechProcStatus_Error;
	}

	_run = false;
	_proc_thread.join();

	if (_mic_led_ring != nullptr) {
		_mic_led_ring->Close();
		_mic_led_ring.reset();
	}

	if (_audio_capture != nullptr) {
		_audio_capture->Close();
		_audio_capture.reset();
	}
	_open = false;
	return SpeechProcStatus_Ok;
}

////////////////////////////////////////////////////////////////////////
// Find audio card by name
// Based on Source from https://gist.github.com/dontknowmyname/4536535
////////////////////////////////////////////////////////////////////////

int SpeechInputProc::FindCard(const std::string cardName)
{
	int err;
	int cardNum = -1;

	for (;;) {
		snd_ctl_t *cardHandle;

		// Get next sound card's card number.
		// When "cardNum" == -1, then ALSA
		// fetches the first card
		if ((err = snd_card_next(&cardNum)) < 0) {
			break;
		}

		// No more cards? ALSA sets "cardNum" to -1 if so
		if (cardNum < 0) break;

		// Open this card's (cardNum's) control interface.
		// We specify only the card number -- not any device nor sub-device too
		{
			stringstream ss;
			ss << "hw:" << cardNum;
			if ((err = snd_ctl_open(&cardHandle, ss.str().c_str(), 0)) < 0)	//Now cardHandle becomes your sound card.
			{
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't open card %i: %s\n", cardNum, snd_strerror(err));
				continue;
			}
		}

		{
			snd_ctl_card_info_t *cardInfo;	//Used to hold card information
			//We need to get a snd_ctl_card_info_t. Just alloc it on the stack
			snd_ctl_card_info_alloca(&cardInfo);
			//Tell ALSA to fill in our snd_ctl_card_info_t with info about this card
			if ((err = snd_ctl_card_info(cardHandle, cardInfo)) < 0) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't get info for card %i: %s\n", cardNum, snd_strerror(err));
			} else {
				const char* name = snd_ctl_card_info_get_name(cardInfo);
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Card %i = %s\n", cardNum, name);
				if (!cardName.compare(0, strlen(name), snd_ctl_card_info_get_name(cardInfo), cardName.length())) {
					break;
				}
			}
		}
		// Close the card's control interface after we're done with it
		snd_ctl_close(cardHandle);
	}

	//ALSA allocates some mem to load its config file when we call some of the
	//above functions. Now that we're done getting the info, let's tell ALSA
	//to unload the info and free up that mem
	snd_config_update_free_global();
	return cardNum;
}

////////////////////////////////////////////////////////////////////////
// Create wake word detector if it doesn't exist.  If it already
// exists, then set the enable state.
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechInputProc::WakeWordEnable(std::string wake_word, bool bEnable)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	for (const auto & [name, model]: _installed_wake_word_detectors) {
		if (!wake_word.compare(name)) {
			// The wake word is supported.  Has it been created already?
			auto wd = _wake_detectors.find(wake_word);
			if (wd != _wake_detectors.end()) {
				// Yep, update its enable state
				wd->second->Enable(bEnable);
				return SpeechProcStatus_Ok;
			}
			// Create instance
			std::shared_ptr<WakeWordDetector> wwd = std::make_shared<WakeWordDetector>(model);
			if (wwd->Open() == SpeechDetStatus_Ok) {
				wwd->Enable(true);
			} else {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to create wake word detector (%s)", wake_word.c_str());
				return SpeechProcStatus_Error;
			}
			_wake_detectors[wake_word] = wwd;
			return SpeechProcStatus_Ok;
		}
	}
	return SpeechProcStatus_Error;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::SetWakeWordCB(std::function<void(std::string)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_ww_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechInputProc::RecognizeStart()
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting speech recognition");

	const std::lock_guard<std::mutex> lock(_mutex);
	if (_listening_cb != nullptr) {
		_listening_cb(true);
	}
	if (_recog_detector == nullptr) {
		_recog_detector = std::make_unique<RecogitionDetector>();
	}
	if (_recog_detector->Start() == SpeechDetStatus_Ok) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started speech recognition");

		if (_mic_led_ring != nullptr) {
			_mic_led_ring->SetRingMode(RespeakerPixelRing::pixel_ring_mode_spin);
		}
		return SpeechProcStatus_Ok;
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to start speech recognition");
	if (_listening_cb != nullptr) {
		_listening_cb(false);
	}
	return SpeechProcStatus_Error;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

SpeechProcStatus SpeechInputProc::RecognizeStop()
{
	// Not supported
	const std::lock_guard<std::mutex> lock(_mutex);
	return SpeechProcStatus_Error;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::SetRecogizeCB(std::function<void(SpeechProcStatus, std::string)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_recog_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::SetListeningCB(std::function<void(bool)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_listening_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::SetVADCB(std::function<void(bool)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_vad_cb = callback;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::SetAOACB(std::function<void(int32_t)> callback)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	_aoa_cb = callback;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::MuteInput(bool mute)
{
	const std::lock_guard<std::mutex> lock(_mutex);
	if (_audio_capture != nullptr) {
		_audio_capture->SetMute(mute);
	}
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void SpeechInputProc::Process()
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SpeechProcStatus::Process running\n");

	int buffer_frames = FRAMES_PER_READ;
	int buffer_size = buffer_frames * SAMPLE_BITS / 8 * NUM_CHANNELS;
	uint16_t* read_buffer = (uint16_t*)malloc(buffer_size);

	int vad = 0;
	int aoa = 0;
	bool firstVAD = true;
	bool firstAOA = true;

	auto start = system_clock::now();
	bool update_period = false;

	while (_run) {
		auto now = system_clock::now();
		const auto diff = std::chrono::duration_cast<milliseconds>(now - start).count();
		if (diff > 500) {
			update_period = true;
			start = now;
		}

		// Read audio block
		int frames_read = _audio_capture->ReadBlock(read_buffer, buffer_frames);
		if (frames_read > 0) {
			const std::lock_guard<std::mutex> lock(_mutex);

			// Pass frame to each detector
			// Wake word detectors
			for (const auto & wwd: _wake_detectors) {
				auto ret = wwd.second->ProcessData((uint8_t*)read_buffer, frames_read*2);
				if (ret == SpeechDetStatus_Done) {
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detected wake word (%s)", wwd.first.c_str());
					if (_ww_cb != nullptr) {
						_ww_cb(wwd.first);
					}
				} else if (ret == SpeechDetStatus_Error) {
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WW detection error for (%s)", wwd.first.c_str());
				}
			}
			// Speech recognition
			if (_recog_detector != nullptr) {
				auto ret = _recog_detector->ProcessData((uint8_t*)read_buffer, frames_read*2);
				if (ret == SpeechDetStatus_Done) {
					if (_listening_cb != nullptr) {
						_listening_cb(false);
					}
					if (_mic_led_ring != nullptr) {
						_mic_led_ring->SetRingMode(RespeakerPixelRing::pixel_ring_mode_listen);
					}
					std::string text;
					if (_recog_detector->GetResult(text)) {
						RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speech detected: %s", text.c_str());
						if (_recog_cb != nullptr) {
							_recog_cb(SpeechProcStatus_Ok, text);
						}
					} else {
						// No speech detected
						RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No Speech detected");
						if (_recog_cb != nullptr) {
							_recog_cb(SpeechProcStatus_Error, text);
						}
					}
				}
			}
		} else {
			// Throttle this error print when it occurs
			if (update_period) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to read audio samples (%s)", snd_strerror(frames_read));
			}
		}

		// Periodically update VAD and AOA status
		if (update_period) {
			if (_mic_led_ring != nullptr) {
				int val = _mic_led_ring->ReadVAD();
				if (_vad_cb != nullptr &&
					(val != vad || firstVAD)) {
					firstVAD = false;
					_vad_cb(val != 0);
				}
				vad = val;

				val = _mic_led_ring->ReadDOA();
				if (_aoa_cb != nullptr &&
					(val != aoa || firstAOA)) {
					firstAOA = false;
					_aoa_cb(val);
				}
				aoa = val;
			}
		}
		update_period = false;
	}

	free(read_buffer);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

