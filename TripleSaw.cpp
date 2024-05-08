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
int     output_value = 0; //Encoder

Oscillator osc_a, osc_b, osc_c;

Svf       filter;
Parameter cutoff_ctrl, res_ctrl, drive_ctrl;

float voct_disp;
float coarse_disp;

EnvelopeOscillator::Envelope env;
constexpr uint8_t            NUM_GATE_IN = 2;
bool                         gate[NUM_GATE_IN];

using MyDisplay = OledDisplay<SSD130x4WireSpi128x64Driver>;
MyDisplay display;

//Variables Menu
int  values[16];
bool trigs[16];
int  stepNumber;
bool trigOut;

int  menuPos;
bool inSubMenu;

void DisplayConfig()
{
    MyDisplay::Config display_config;

    SpiHandle::Config& spi_conf
        = display_config.driver_config.transport_config.spi_config;

    spi_conf.mode = SpiHandle::Config::Mode::MASTER; // we're in charge
    spi_conf.periph
        = SpiHandle::Config::Peripheral::SPI_2; // Use the SPI_2 Peripheral
    spi_conf.direction
        = SpiHandle::Config::Direction::ONE_LINE; // TWO_LINES_TX_ONLY;

    spi_conf.datasize       = 8;
    spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
    spi_conf.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
    // spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
    spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_128;

    // Pins to use. These must be available on the selected peripheral
    spi_conf.pin_config.sclk = DaisyPatchSM::D10; // Use pin D10 as SCLK
    spi_conf.pin_config.miso = Pin();             // We won't need this
    spi_conf.pin_config.mosi = DaisyPatchSM::D9;  // Use D9 as MOSI
    spi_conf.pin_config.nss  = Pin(); // DaisyPatchSM::D1;   // use D1 as NSS

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

    osc_a.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
    osc_b.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
    osc_c.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);

    //patch.StartLog(false);
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
    coarse_disp       = coarse_tune;

    float knob_fine = patch.GetAdcValue(CV_2);
    float fine_tune = fmap(knob_fine, 0, 10);

    float cv_voct = patch.GetAdcValue(CV_4);
    float voct    = fmap(cv_voct, 0, 60);
    voct_disp     = voct;

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

    //encoder
    //can we simplify the menu logic?
    if(!inSubMenu)
    {
        menuPos += encoder.Increment();

        if(menuPos < 16 && menuPos > -1)
        {
            inSubMenu = encoder.RisingEdge() ? true : false;
        }
        else if(menuPos > 15)
        {
            menuPos = 0;
        }
        else if(menuPos < 0)
        {
            menuPos = 15;
        }
    }

    else
    {
        values[menuPos] += encoder.Increment();
        values[menuPos] = values[menuPos] < 0.f ? 0.f : values[menuPos];
        values[menuPos] = values[menuPos] > 60.f ? 60.f : values[menuPos];
        inSubMenu       = encoder.RisingEdge() ? false : true;
    }
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    UpdateControls();

    /** Process each sample of the oscillator and send to the hardware outputs */
    for(size_t i = 0; i < size; i++)
    {
        float sig = osc_a.Process() + osc_b.Process();
        +osc_c.Process();

        //send the next sample to the filter
        filter.Process(sig);

        OUT_L[i] = filter.Low();
        OUT_R[i] = env.Process() * sig;
    }
}

void UpdateDisplay()
{
    display.Fill(false);

    std::string str;
    char*       cstr  = &str[0];
    int         pos_y = 5;
    int         pos_x = 0;

    //values and trigs
    for(int i = 0; i < 16; i++)
    {
        sprintf(cstr, "%d", values[i]);
        display.SetCursor(((pos_x * 25) + 5), pos_y);

        if(i == menuPos)
        {
            display.WriteString(cstr, Font_7x10, true);
        }
        else
        {
            display.WriteString(cstr, Font_6x8, true);
        }

        if(pos_x > 2)
        {
            pos_y += 15;
        }


        if(pos_x < 3)
        {
            pos_x += 1;
        }
        else
        {
            pos_x = 0;
        }


        /*str = trigs[i % 5] ? "X" : "O";
        display.SetCursor(((i * 25) + 10), 24);
        display.WriteString(cstr, Font_6x8, true); */

        /*str = trigs[i % 5] ? "X" : "O";
        display.SetCursor(((i * 25) + 10), 56);
        display.WriteString(cstr, Font_6x8, true); */
    }

    //cursor
    //str = inSubMenu ? "--" : "-";
    /*
    if(menuPos > 3 && menuPos < 8)
    {
        pos_y = 16;
    }
    else if(menuPos > 7 && menuPos < 12)
    {
        pos_y = 32;
    }
    else if(menuPos > 11 && menuPos < 16)
    {
        pos_y = 48;
    }
    else if(menuPos > -1 && menuPos < 4)
    {
        pos_y = 0;
    }

    if(menuPos == 1 || menuPos == 5 || menuPos == 9 || menuPos == 13)
    {
        pos_x = 1;
    }
    else if(menuPos == 2 || menuPos == 6 || menuPos == 10 || menuPos == 14)
    {
        pos_x = 2;
    }
    else if(menuPos == 3 || menuPos == 7 || menuPos == 11 || menuPos == 15)
    {
        pos_x = 3;
    }
    else if(menuPos == 0 || menuPos == 4 || menuPos == 8 || menuPos == 12)
    {
        pos_x = 0;
    } */

    str = inSubMenu ? "SET" : "SEL";
    display.SetCursor(100, 38);
    display.WriteString(cstr, Font_7x10, true);

    sprintf(cstr, "%d", (menuPos + 1));
    display.SetCursor(110, 48);
    display.WriteString(cstr, Font_7x10, true);

    display.Update();
}

int main(void)
{
    Inits();

    patch.StartAdc();
    patch.StartAudio(AudioCallback);

    //patch.PrintLine("Todos los sistemass Inicializados");
    //patch.PrintLine("Let's Go !!!");

    while(1)
    {
        UpdateDisplay();
    }
}
