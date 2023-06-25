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

#include <functional>
#include <thread>
#include <mutex>
#include <string>
#include <map>

class WakeWordDetector;
class RecogitionDetector;
class RespeakerPixelRing;
class AudioCapture;

enum SpeechProcStatus {
	SpeechProcStatus_Ok = 0,
	SpeechProcStatus_Error = 1,
};

class SpeechInputProc
{
public:
	SpeechInputProc();
	~SpeechInputProc();

	SpeechProcStatus Open();
	SpeechProcStatus Close();

	SpeechProcStatus WakeWordInstall(const std::string &wake_word_file);
	SpeechProcStatus WakeWordEnable(std::string wake_word, bool bEnable);
	void SetWakeWordCB(std::function<void(std::string, int32_t angle)> callback);

	SpeechProcStatus RecognizeStart();
	SpeechProcStatus RecognizeStop();
	void SetRecogizeCB(std::function<void(SpeechProcStatus, std::string)> callback);

	void SetListeningCB(std::function<void(bool)> callback);
	void SetVADCB(std::function<void(bool)> callback);
	void SetAOACB(std::function<void(int32_t)> callback);

	void MuteInput(bool mute);

protected:
	void Process();
	int FindCard(const std::string cardName);

private:
	bool _open;
	bool _run;
	std::function<void(std::string, int32_t angle)> _ww_cb;
	std::function<void(SpeechProcStatus, std::string)> _recog_cb;
	std::function<void(bool)> _listening_cb;
	std::function<void(bool)> _vad_cb;
	std::function<void(int32_t)> _aoa_cb;

	std::map<std::string, std::shared_ptr<WakeWordDetector>> _wake_detectors;
	std::unique_ptr<RecogitionDetector> _recog_detector;

	std::unique_ptr<RespeakerPixelRing> _mic_led_ring;
	std::unique_ptr<AudioCapture> _audio_capture;

	std::thread _proc_thread;
	std::mutex _mutex;

	std::map<std::string, std::string> _installed_wake_word_detectors;

};
