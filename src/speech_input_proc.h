
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

	static const std::string WakeWordDetector_HeyRobot;
	static const std::string WakeWordDetector_HeyAnna;

public:
	SpeechInputProc();
	~SpeechInputProc();

	SpeechProcStatus Open();
	SpeechProcStatus Close();

	SpeechProcStatus WakeWordEnable(std::string wake_word, bool bEnable);
	void SetWakeWordCB(std::function<void(std::string)> callback);

	SpeechProcStatus RecognizeStart();
	SpeechProcStatus RecognizeStop();
	void SetRecogizeCB(std::function<void(SpeechProcStatus, std::string)> callback);

	void SetListeningCB(std::function<void(bool)> callback);
	void SetVADCB(std::function<void(bool)> callback);
	void SetAOACB(std::function<void(int32_t)> callback);

	void MuteInput(bool mute);

protected:
	void Process();

private:
	bool _open;
	bool _run;
	std::function<void(std::string)> _ww_cb;
	std::function<void(SpeechProcStatus, std::string)> _recog_cb;
	std::function<void(bool)> _listening_cb;
	std::function<void(bool)> _vad_cb;
	std::function<void(int32_t)> _aoa_cb;

	std::map<std::string, WakeWordDetector*> _wake_detectors;
	RecogitionDetector *_recog_detector;

	RespeakerPixelRing *_mic_led_ring;
	AudioCapture *_audio_capture;

	std::thread _proc_thread;
	std::mutex _mutex;

	std::map<std::string, std::string> _installed_wake_word_detectors;

};
