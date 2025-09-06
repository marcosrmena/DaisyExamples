#include "daisysp.h"
#include "daisy_petal.h"
#include "Effects/pitchshifter.h"

#define MAX_DELAY static_cast<size_t>(48000 * 2.5f) // 2.5s at 48kHz

using namespace daisy;
using namespace daisysp;

DaisyPetal petal;

DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delMems[1];
PitchShifter pitchShifter;

float samplerate;

struct delay
{
    DelayLine<float, MAX_DELAY>* del;
    float currentDelay = 0.f;
    float delayTarget = 0.f;
    float feedback = 0.f;

    // Cubic interpolation helper
    float CubicInterp(float y0, float y1, float y2, float y3, float mu)
    {
        float a0, a1, a2, a3, mu2;

        mu2 = mu * mu;
        a0 = y3 - y2 - y0 + y1;
        a1 = y0 - y1 - a0;
        a2 = y2 - y0;
        a3 = y1;

        return (a0 * mu * mu2 + a1 * mu2 + a2 * mu + a3);
    }

    float Process(float in)
    {
        // Smooth delay changes for fluid transitions
        fonepole(currentDelay, delayTarget, 0.001f);

        // Integer part and fractional part of delay
        float delayInt = floorf(currentDelay);
        float frac = currentDelay - delayInt;

        // Read 4 samples for cubic interpolation
        del->SetDelay(delayInt - 1);
        float y0 = del->Read();

        del->SetDelay(delayInt);
        float y1 = del->Read();

        del->SetDelay(delayInt + 1);
        float y2 = del->Read();

        del->SetDelay(delayInt + 2);
        float y3 = del->Read();

        // Restore fractional delay for write
        del->SetDelay(currentDelay);

        // Interpolate the sample
        float read = CubicInterp(y0, y1, y2, y3, frac);

        // Write input + feedback (no filter)
        del->Write(feedback * read + in);

        return read;
    }
};

delay delays[1];

// Parameters for knobs
Parameter delayTimeParam;
Parameter feedbackParam;
Parameter dryWetParam;
Parameter pitchMixParam;

int drywetPercent = 50;
bool passThruOn = false;

void ProcessControls();

static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    ProcessControls();

    for(size_t i = 0; i < size; i++)
    {
        float drywet = passThruOn ? 0.f : dryWetParam.Process();

        float delayed = delays[0].Process(in[0][i]);
        float pitched = pitchShifter.Process(delayed);

        float pitchMix = pitchMixParam.Process();
        float delayOut = (1.0f - pitchMix) * delayed + pitchMix * pitched;

        float mix = drywet * delayOut * 0.3f + (1.0f - drywet) * in[0][i];

        out[0][i] = out[1][i] = mix;
    }
}

void InitDelays(float samplerate)
{
    delMems[0].Init();
    delays[0].del = &delMems[0];

    delayTimeParam.Init(petal.knob[0], samplerate * 0.03f, MAX_DELAY, Parameter::LOGARITHMIC);
    feedbackParam.Init(petal.knob[1], 0.f, 1.f, Parameter::LINEAR);
    dryWetParam.Init(petal.knob[2], 0.f, 1.f, Parameter::LINEAR);
    pitchMixParam.Init(petal.knob[3], 0.f, 1.f, Parameter::LINEAR);

    pitchShifter.Init(samplerate);
    pitchShifter.SetTransposition(12.0f); // octave up

    delays[0].currentDelay = delays[0].delayTarget = 48000 * 0.5f; // default 0.5s delay
}

int main(void)
{
    petal.Init();
    petal.SetAudioBlockSize(4);
    samplerate = petal.AudioSampleRate();

    InitDelays(samplerate);

    petal.StartAdc();
    petal.StartAudio(AudioCallback);

    while(1)
    {
        int32_t whole = drywetPercent / 12.5f;
        float frac = (float)drywetPercent / 12.5f - whole;

        petal.ClearLeds();

        for(int i = 0; i < whole; i++)
        {
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(i), 0.f, 0.f, 1.f);
        }

        if(whole < 7 && whole > 0)
            petal.SetRingLed(static_cast<DaisyPetal::RingLed>(whole - 1), 0.f, 0.f, frac);

        petal.SetFootswitchLed(DaisyPetal::FOOTSWITCH_LED_1, passThruOn);
        petal.UpdateLeds();

        System::Delay(6);
    }
}

void ProcessControls()
{
    petal.ProcessAnalogControls();
    petal.ProcessDigitalControls();

    delays[0].feedback = feedbackParam.Process();

    delays[0].delayTarget = delayTimeParam.Process();

    drywetPercent += 5 * petal.encoder.Increment();
    if(drywetPercent > 100) drywetPercent = 100;
    if(drywetPercent < 0)   drywetPercent = 0;

    if(petal.switches[0].RisingEdge())
    {
        passThruOn = !passThruOn;
    }
}
