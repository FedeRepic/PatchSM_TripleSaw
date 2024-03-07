#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "envelope.h"
#include "dev/oled_ssd130x.h"


/** TODO: ADD CALIBRATION */

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM patch;

Encoder encoder;
int output_value = 0; //Encoder

Oscillator   osc_a, osc_b, osc_c;

Svf filter;
Parameter  cutoff_ctrl, res_ctrl, drive_ctrl;

float voct_disp;
float coarse_disp;

EnvelopeOscillator::Envelope   env;
constexpr uint8_t NUM_GATE_IN = 2;
bool  gate[NUM_GATE_IN];

using MyDisplay = OledDisplay<SSD130x4WireSpi128x64Driver>;
MyDisplay display;

void DisplayConfig()
{

   MyDisplay::Config display_config;

    SpiHandle::Config& spi_conf = display_config.driver_config.transport_config.spi_config;

    spi_conf.mode = SpiHandle::Config::Mode::MASTER;             // we're in charge
    spi_conf.periph = SpiHandle::Config::Peripheral::SPI_2;      // Use the SPI_2 Peripheral
    spi_conf.direction = SpiHandle::Config::Direction::ONE_LINE; // TWO_LINES_TX_ONLY;

    spi_conf.datasize = 8;
    spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
    spi_conf.clock_phase = SpiHandle::Config::ClockPhase::ONE_EDGE;
    // spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
    spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_128;

    // Pins to use. These must be available on the selected peripheral
    spi_conf.pin_config.sclk = DaisyPatchSM::D10; // Use pin D10 as SCLK
    spi_conf.pin_config.miso = Pin();             // We won't need this
    spi_conf.pin_config.mosi = DaisyPatchSM::D9;  // Use D9 as MOSI
    spi_conf.pin_config.nss = Pin();              // DaisyPatchSM::D1;   // use D1 as NSS

    // data will flow from master to slave over just the MOSI line

    // The master will output on the NSS line
    spi_conf.nss = SpiHandle::Config::NSS::SOFT;

    display_config.driver_config.transport_config.pin_config.dc
        = DaisyPatchSM::D2;
    display_config.driver_config.transport_config.pin_config.reset
        = DaisyPatchSM::D3;
    display.Init(display_config);

}

void Inits()
{

    patch.Init();

    DisplayConfig();

 /** Initialize our Encoder: CLK, DT, SW */ 
    encoder.Init(DaisyPatchSM::A8, DaisyPatchSM::A9, DaisyPatchSM::D8);

    filter.Init(patch.AudioSampleRate());
    //setup controls
    cutoff_ctrl.Init(patch.controls[0], 20, 20000, Parameter::LOGARITHMIC);
    res_ctrl.Init(patch.controls[1], .3, 1, Parameter::LINEAR);
    drive_ctrl.Init(patch.controls[2], .3, 1, Parameter::LINEAR);

    env.Init(patch.AudioSampleRate());

    osc_a.Init(patch.AudioSampleRate());
    osc_b.Init(patch.AudioSampleRate());
    osc_c.Init(patch.AudioSampleRate());

    osc_a.SetWaveform(Oscillator::WAVE_SIN);
    osc_b.SetWaveform(Oscillator::WAVE_SIN);
    osc_c.SetWaveform(Oscillator::WAVE_SIN);

}

void UpdateControls()
{
   patch.ProcessAllControls(); 
   encoder.Debounce();

   //get new control values
   //float cutoff = cutoff_ctrl.Process();
   float cutoff = 900;
   //float res    = res_ctrl.Process();
   //float drive  = drive_ctrl.Process();

    //Set filter to the values we got
    filter.SetFreq(cutoff);
    //svf.SetRes(res);
    //svf.SetDrive(drive);   

   /** Get Coarse, Fine, and V/OCT inputs from hardware 
     *  MIDI Note number are easy to use for defining ranges */
    float knob_coarse = patch.GetAdcValue(CV_1);
    float coarse_tune = fmap(knob_coarse, 12, 84);
    coarse_disp = coarse_tune;

    float knob_fine = patch.GetAdcValue(CV_2);
    float fine_tune = fmap(knob_fine, 0, 10);

    float cv_voct = patch.GetAdcValue(CV_4);
    float voct    = fmap(cv_voct, 0, 60);
    voct_disp = voct;

    // Convert from MIDI note number to frequency 
    float midi_nn = fclamp(coarse_tune + fine_tune + voct, 0.f, 127.f);
    float freq_a  = mtof(midi_nn);

    // Calculate a detune amount 
    float detune_amt = patch.GetAdcValue(CV_3);
    float freq_b     = freq_a + (0.05 * freq_a * detune_amt);
    float freq_c     = freq_a - (0.05 * freq_a * detune_amt);

    // Set all three oscillators' frequencies 
    osc_a.SetFreq(freq_a);
    osc_b.SetFreq(freq_b);
    osc_c.SetFreq(freq_c);

    env.SetRise(0.f);
    env.SetFall(0.f);

    //Envelope Gate Read 
    gate[0] = patch.gate_in_1.State();

    if(gate[0])
    {
        env.Trigger();
    }

    //Encoder
    int increment = encoder.Increment();

    if(increment < 0) output_value += 1; 
    else if(increment > 0)  output_value -= 1;          
    if(encoder.RisingEdge()) output_value = 0;

}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{

    UpdateControls();

    /** Process each sample of the oscillator and send to the hardware outputs */
    for(size_t i = 0; i < size; i++)
    {
        float sig = osc_a.Process(); //+ osc_b.Process(); + osc_c.Process();

        //send the next sample to the filter
        filter.Process(sig);

        OUT_L[i]  = filter.Low();
        OUT_R[i]  = env.Process() * sig;
    }
}

void UpdateDisplay()
{
    char tmp[64];

    display.Fill(false);
    display.SetCursor(15, 8);
    sprintf(tmp, "%d", System::GetUs());
    display.WriteString(tmp, Font_6x8, true);
    display.SetCursor(15, 16);
    display.WriteString("CIRCUITER", Font_11x18, true);  

    std::string str = "FREQ:" + std::to_string(static_cast<uint32_t>(voct_disp));
    char*       cstr = &str[0];
    display.SetCursor(15, 46);
    display.WriteString(cstr, Font_6x8, true);     

    str = "COAR:" + std::to_string(static_cast<uint32_t>(coarse_disp));
    display.SetCursor(15, 54);
    display.WriteString(cstr, Font_6x8, true);

    str = "ENCO:" + std::to_string(static_cast<uint32_t>(output_value));
    display.SetCursor(64, 54);
    display.WriteString(cstr, Font_6x8, true);    

    display.Update();

}

int main(void)
{

    Inits(); 

    patch.StartAdc();
    patch.StartAudio(AudioCallback);

    while(1) {

       UpdateDisplay();

    }
}
