#include "daisy_seed.h"
#include "daisysp.h"
#include "cmath"

using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;

// ===== Constants =====
constexpr float  kSampleRate     = 48000.0f;
constexpr size_t kBufferSize  = static_cast<size_t>(2.0f * kSampleRate); // 2s ring
 
// Crossfade
constexpr size_t kFadeTime       = 4800*2;  // ~100 ms @ 48k; 
constexpr float  kFadeStep       = 1.0f / float(kFadeTime);

// Guardrails
constexpr size_t kMinDelaySamps  = kFadeTime + 480;             // fade must fit
constexpr size_t kMaxDelaySamps  = kBufferSize - 480;

// ADC smoothing
constexpr float  kMinDelaySec    = float(kMinDelaySamps) / kSampleRate;
constexpr float  kMaxDelaySec    = float(kMaxDelaySamps) / kSampleRate;
constexpr float  kDelayLpfCoeff  = 0.0035f; // knob smoothing

// ===== User params (initial) =====
float delayTimeSec = 0.25f; 
float feedback     = 0.0f;
float wetMix       = 0.5f;

// ===== Buffer/heads =====
float  delayBuffer[kBufferSize];
size_t writePos = 0;

// ===== Delay time state =====
size_t L_current = 0;
size_t L_next    = 0;
float  delayTimeSec_filt = delayTimeSec; //filter-smoothed delay time

// ===== Helpers =====
inline size_t wrapDec(size_t x) { return (x == 0) ? (kBufferSize - 1) : (x - 1); }
inline size_t wrapAdd(size_t a, size_t b) { return (a + b) % kBufferSize; }
inline size_t clampSize(size_t v, size_t lo, size_t hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

// ===== Crossfade =====
CrossFade xfade;
float     fade        = 0.0f;     // 0..1
size_t    fadeCount   = 0;
bool      fading      = false;
bool      fadeSwap    = false; 

// ===== Segment Engine =====
struct Segment {
    size_t head;       // current read head for this segment (moves backward)
    size_t start;      // where this segment started (for debug)
    size_t len;        // segment length (L)
    size_t played;     // samples already played in this segment (0..len)
	bool active;	   // tracks if segment is currently reading from buffer		
	static bool aLag;  // track of which segment is leading/lagging the other
};

Segment segA{}, segB{};
bool Segment::aLag = true; //keep track of which segment is ahead

//Start segment at current write position
inline void StartSegment(Segment& s, size_t currentWrite, size_t len)
{
    s.len    = len;
    s.played = 0;
    s.start  = currentWrite; // Read head begins at the current write position (newest sample), then walks backward
    s.head   = currentWrite;
	s.active = true;
	Segment::aLag = !Segment::aLag; //Segment A activated on startup, setting aLag false
}

//Deactivate segment, reset all parameters
inline void DeactivateSegment(Segment& s)
{	
	s.head = 0;
	s.start = 0;
	s.len = 0;
	s.played = 0;
	s.active = false;
}

//Advance a segment one sample and return its sample
inline float StepSegment(Segment& s, float* buf)
{
    float y = buf[s.head];
    s.head  = wrapDec(s.head);
    s.played++;
    return y;
}

bool startup = true;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{	
		delayTimeSec = (kMinDelaySec + (kMaxDelaySec - kMinDelaySec)) * 0.5f; //NEED TO ADD ADC FUNCTIONALITY AND SMOOTHING
		float delayTimeSamp = delayTimeSec * kSampleRate;
		
		if(startup) // begins cyclic segment handoff process indefinitely
		{
			DeactivateSegment(segA);
			DeactivateSegment(segB);
			StartSegment(segA, writePos, delayTimeSamp);
			startup = false;
		}
		
		float yA = 0;
		float yB = 0;

		if(segA.active && !segB.active) //read from buffer
		{
			yA = StepSegment(segA, delayBuffer);
			yB = 0;
		}
		if(segB.active && !segA.active)
		{
			yA = 0;
			yB = StepSegment(segB, delayBuffer);
		}
		if(segA.active && segB.active)
		{
			yA = StepSegment(segA, delayBuffer);
			yB = StepSegment(segB, delayBuffer);
		}
		if(!segA.active && !segB.active)
		{
			yA = 0;
			yB = 0;
		}

		float output = 0;

		if(fading) //fade logic
		{
			xfade.SetPos(fade);
			
			if(fadeCount < kFadeTime)
			{
				if(!fadeSwap)
					fade += kFadeStep; 
				else
					fade -= kFadeStep; 
				fadeCount++;
			}
			else
			{
				fading = false;
				fadeCount = 0;
				fadeSwap = !fadeSwap;
			}
		}

		output = xfade.Process(yA, yB); //output = (1-fade) * yA + fade * yB

		out[0][i] = in[0][i] + (0.5 * output); //output from Seed

		delayBuffer[writePos] = in[0][i]; //write to delay buffer with feedback

		if(Segment::aLag) //start new segment and begin fading if old segment is within kFadeTime of ending
		{
			if(!fading && segB.played + kFadeTime >= segB.len)
			{
				fading = true;
				fade = 1.0f;
				StartSegment(segA, writePos, delayTimeSamp);
			}
		}
		else
		{
			if(!fading && segA.played + kFadeTime >= segA.len)
			{
				fading = true;
				fade = 0.0f; 
				StartSegment(segB, writePos, delayTimeSamp);
			}
		}

		if(segA.played >= segA.len) //deactivate segment if complete
		{
			DeactivateSegment(segA);
		}
		if(segB.played >= segB.len)
		{
			DeactivateSegment(segB);
		}

		writePos = (writePos + 1) % kBufferSize; //increment and wrap write position
	}
}

int main(void)
{
	hw.Configure();
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	xfade.Init();

	AdcChannelConfig adc[3];

	adc[0].InitSingle(A0);
	adc[1].InitSingle(A1);
	adc[2].InitSingle(A2);

	hw.adc.Init(adc, 3);

	hw.adc.Start();

	hw.StartAudio(AudioCallback);
	
	while(1) {
		float adc0 = hw.adc.Get(0);
		float adc1 = hw.adc.Get(1);
		float adc2 = hw.adc.Get(2);

		delayTimeSec = adc0/65536.0f;
		wetMix = adc1/65536.0f;
		feedback = adc2/65536.0f;
	}
}

/*
		//Debugging
        hw.PrintLine("=====================");
		hw.PrintLine("aLag: %s", Segment::aLag ? "true" : "false");
        hw.PrintLine(" ");
		hw.PrintLine("Write Position: %lu", static_cast<size_t>(writePos));
        hw.PrintLine("Seg A Head: %lu", static_cast<size_t>(segA.head));
        hw.PrintLine("Seg B Head: %lu", static_cast<size_t>(segB.head));
        hw.PrintLine("---------------------");
		hw.PrintLine("Fading: %s", fading ? "true" : "false");
        hw.PrintLine("Fade Swap: %s", fadeSwap ? "true" : "false");
        hw.PrintLine("Fade Count: %lu", static_cast<size_t>(fadeCount));
		hw.PrintLine("Fade: " FLT_FMT3, FLT_VAR3(fade));
        hw.PrintLine("---------------------");
        hw.PrintLine("Seg A Active?: %s", segA.active? "true" : "false");
        hw.PrintLine("Seg A Length: %lu", static_cast<size_t>(segA.len));
        hw.PrintLine("Seg A Start: %lu", static_cast<size_t>(segA.start));
        hw.PrintLine("Seg A Played: %lu", static_cast<size_t>(segA.played));
        hw.PrintLine("---------------------");
        hw.PrintLine("Seg B Active?: %s", segB.active ? "true" : "false");
        hw.PrintLine("Seg B Length: %lu", static_cast<size_t>(segB.len));
        hw.PrintLine("Seg B Start: %lu", static_cast<size_t>(segB.start));
        hw.PrintLine("Seg B Played: %lu", static_cast<size_t>(segB.played));
        hw.PrintLine(" ");
        hw.DelayMs(0.5);
	*/