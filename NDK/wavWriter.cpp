// WavWriter.cpp
// Erwan Leroy - 2025

/*
Example of using this as part of making an Audio Viewer:

set cut_paste_input [stack 0]
version 14.1 v5
push $cut_paste_input
Group {
name AudioViewer
selected true
xpos 1023
ypos 77
addUserKnob {20 User}
addUserKnob {41 sample_rate l "Sample Rate (Hz)" T Write1.sample_rate}
addUserKnob {41 bit_depth l "Bit Depth" T Write1.bit_depth}
addUserKnob {41 min_db l "Min dB" T Write1.min_db}
addUserKnob {41 max_db l "Max dB" T Write1.max_db}
addUserKnob {41 output_channels l "Output Channels" T Write1.output_channels}
addUserKnob {41 input_type l "Input Type" T Write1.input_type}
addUserKnob {22 play l Play T "from PySide2.QtMultimedia import QMediaPlayer, QMediaContent\nplayer = QMediaPlayer()\nnode = nuke.toNode('Write1')\nnuke.execute(node, 1, 1)\nmedia = QMediaContent(node\['file'].value())\nplayer.setMedia(media)\nplayer.play()" +STARTLINE}
addUserKnob {22 stop l Stop -STARTLINE T "if 'player' in globals():\n    player.stop()"}
}
Input {
inputs 0
name Input1
xpos 502
ypos -16
}
Write {
file _temp_sound.wav
file_type wav
output_channels Stereo
input_type "Raw FFT"
checkHashOnRead false
version 31
ocioColorspace scene_linear
display default
view sRGB
name Write1
xpos 502
ypos 24
}
Output {
name Output1
xpos 502
ypos 124
}
end_group


*/


#include "DDImage/FileWriter.h"
#include "DDImage/Row.h"
#include "DDImage/Knobs.h"
#include "DDImage/DDMath.h"
#include "DDImage/Enumeration_KnobI.h"
#include "DDImage/MetaData.h"
#include "DDImage/Iop.h"
#include "DDImage/ChannelSet.h"

#include <stdio.h>
#include <vector>
#include <cmath> 
#include <algorithm> 
#include <fstream>   
#include <string.h>  
#include <stdexcept> 

// KissFFT headers
#include "kiss_fftr.h"
#include "kiss_fft.h" 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define WAV header structures (same as in WavReader)
#pragma pack(push, 1)
struct WavFileHeader {
    char riff_header[4];     // "RIFF"
    uint32_t wav_size;       // Total file size minus 8 bytes
    char wave_header[4];     // "WAVE"
    char fmt_header[4];      // "fmt " (note space at the end)
    uint32_t fmt_chunk_size; // Should be 16 for PCM
    uint16_t audio_format;   // 1 for PCM, 3 for IEEE float
    uint16_t num_channels;   // Number of audio channels
    uint32_t sample_rate;    // Samples per second
    uint32_t byte_rate;      // sample_rate * num_channels * bits_per_sample / 8
    uint16_t block_align;    // num_channels * bits_per_sample / 8
    uint16_t bits_per_sample;// Bits per sample
};

struct WavDataChunkHeader {
    char data_header[4];     // "data"
    uint32_t data_size;      // Total size of audio data in bytes
};
#pragma pack(pop)

using namespace DD::Image;

// --- Knobs Configuration ---
static const char* const BIT_DEPTH_NAMES[] = {
    "16-bit PCM",
    "24-bit PCM",
    "32-bit Float PCM",
    "8-bit PCM",
    nullptr
};

enum BitDepthOptions {
    BIT_DEPTH_16_PCM = 0,
    BIT_DEPTH_24_PCM,
    BIT_DEPTH_32_FLOAT,
    BIT_DEPTH_8_PCM
};

static const char* const CHANNEL_CONFIG_NAMES[] = {
    "Mono",
    "Stereo",
    nullptr
};

static const char* const INPUT_TYPE[] = {
    "Magnitude and Phase", 
    "Raw FFT", 
    "Magnitude only", 
    nullptr
};

enum ChannelConfigOptions {
    CH_CONFIG_MONO = 0,
    CH_CONFIG_STEREO
};

class WavWriter : public FileWriter {
private:
    // Knob values
    int _sample_rate_knob_value;
    int _bit_depth_knob_value;
    float _min_db_knob_value;
    float _max_db_knob_value;
    int _channel_config_knob_value; // Mono or Stereo output
	int _input_type_knob_value; // Input type: Magnitude and Phase, Raw FFT, or Magnitude only

    // Internal state for writing
    WavFileHeader _wav_header;
    WavDataChunkHeader _data_chunk_header;
    uint32_t _total_audio_bytes_written;


public:
    WavWriter(Write* iop) : FileWriter(iop),
        _sample_rate_knob_value(44100),
        _bit_depth_knob_value(BIT_DEPTH_16_PCM),
        _min_db_knob_value(-60.0f),
        _max_db_knob_value(0.0f),
        _channel_config_knob_value(CH_CONFIG_MONO),
        _total_audio_bytes_written(0),
		_input_type_knob_value(0)
    {
        memset(&_wav_header, 0, sizeof(WavFileHeader));
        memset(&_data_chunk_header, 0, sizeof(WavDataChunkHeader));
    }

    ~WavWriter() override {}

    void knobs(Knob_Callback f) override {
        Int_knob(f, &_sample_rate_knob_value, "sample_rate", "Sample Rate (Hz)");
        SetRange(f, 8000, 192000);
        Tooltip(f, "Sample rate of the output WAV file in Hz (e.g., 44100, 48000).");

        Enumeration_knob(f, &_bit_depth_knob_value, BIT_DEPTH_NAMES, "bit_depth", "Bit Depth");
        Tooltip(f, "Bit depth and format of the output WAV file.");

        Float_knob(f, &_min_db_knob_value, "min_db", "Min dB");
        Tooltip(f, "The minimum dB value corresponding to black (0.0) in the input spectrogram's magnitude channels.");

        Float_knob(f, &_max_db_knob_value, "max_db", "Max dB");
        Tooltip(f, "The maximum dB value corresponding to white (1.0) in the input spectrogram's magnitude channels.");

        Enumeration_knob(f, &_channel_config_knob_value, CHANNEL_CONFIG_NAMES, "output_channels", "Output Channels");
        Tooltip(f, "Select Mono or Stereo output. For Stereo, R=L_dB, G=L_phase, B=R_dB, A=R_phase is expected from input.");

		Enumeration_knob(f, &_input_type_knob_value, INPUT_TYPE, "input_type", "Input Type");
		Tooltip(f, "Select the type of input data:\n"
			"1. Magnitude and Phase: Input channels are expected to be in the order L_dB, L_phase, R_dB, R_phase.\n"
			"2. Raw FFT: Input channels are expected to be in the order L_real, L_imaginary, R_real, R_imaginary.\n"
			"3. Magnitude only: Input channels are expected to be in the order L_dB, black, R_dB, black.\n"
            "   This will calculate a phase on the fly to permit uninterupted frequency waves(Ideal for generating audio from scratch).");
		SetFlags(f, Knob::STARTLINE);
    }

    // prepare_wav_header returns false on critical error
    bool prepare_wav_header() {
        if (_sample_rate_knob_value <= 0) {
            iop->error("WavWriter: Sample Rate must be positive.");
            return false;
        }
        if (_min_db_knob_value >= _max_db_knob_value) {
            iop->error("WavWriter: Min dB must be less than Max dB.");
            return false;
        }

        memcpy(_wav_header.riff_header, "RIFF", 4);
        memcpy(_wav_header.wave_header, "WAVE", 4);
        memcpy(_wav_header.fmt_header, "fmt ", 4);
        _wav_header.fmt_chunk_size = 16; // Standard for PCM

        _wav_header.num_channels = (_channel_config_knob_value == CH_CONFIG_STEREO) ? 2 : 1;
        _wav_header.sample_rate = static_cast<uint32_t>(_sample_rate_knob_value);

        switch (_bit_depth_knob_value) {
        case BIT_DEPTH_16_PCM:
            _wav_header.audio_format = 1; // PCM
            _wav_header.bits_per_sample = 16;
            break;
        case BIT_DEPTH_24_PCM:
            _wav_header.audio_format = 1; // PCM
            _wav_header.bits_per_sample = 24;
            break;
        case BIT_DEPTH_32_FLOAT:
            _wav_header.audio_format = 3; // IEEE Float
            _wav_header.bits_per_sample = 32;
            break;
        case BIT_DEPTH_8_PCM:
            _wav_header.audio_format = 1; // PCM
            _wav_header.bits_per_sample = 8;
            break;
        default:
            _wav_header.audio_format = 1;
            _wav_header.bits_per_sample = 16;
            break;
        }

        _wav_header.block_align = (_wav_header.num_channels * _wav_header.bits_per_sample) / 8;
        if (_wav_header.block_align == 0) {
            iop->error("WavWriter: Calculated block_align is zero. Check bit depth and channel settings.");
            return false;
        }
        _wav_header.byte_rate = _wav_header.sample_rate * _wav_header.block_align;

        memcpy(_data_chunk_header.data_header, "data", 4);
        _data_chunk_header.data_size = 0;
        _wav_header.wav_size = 36;
        return true;
    }


    void execute() override {
        if (!open()) {
            iop->error("WavWriter: Could not open file for writing.");
            return;
        }

        _total_audio_bytes_written = 0;

        int image_w = width();
        int image_h = height();

        if (image_h <= 0) {
            iop->warning("WavWriter: Input image height is %d, nothing to process. Writing empty WAV.", image_h);
            if (prepare_wav_header()) {
                _data_chunk_header.data_size = 0;
                _wav_header.wav_size = 36 + 0; // 36 = sizeof(WavFileHeader) - 8 + sizeof(WavDataChunkHeader) - 8 for "data" and its size
                this->write(0, &_wav_header, sizeof(WavFileHeader));
                this->write(sizeof(WavFileHeader), &_data_chunk_header, sizeof(WavDataChunkHeader));
            }
            close();
            return;
        }

        if (image_w < 2) {
            iop->error("WavWriter: Input image width must be at least 2 (for DC and Nyquist components).");
            close();
            return;
        }
        int N_fft = (image_w - 1) * 2;

        if (!prepare_wav_header()) {
            close();
            return;
        }

        this->write(0, &_wav_header, sizeof(WavFileHeader));
        this->write(sizeof(WavFileHeader), &_data_chunk_header, sizeof(WavDataChunkHeader));

        long audio_data_start_offset = sizeof(WavFileHeader) + sizeof(WavDataChunkHeader);

        ChannelSet read_channels;
        read_channels += Chan_Red;
        read_channels += Chan_Green;
        bool write_stereo = (_channel_config_knob_value == CH_CONFIG_STEREO);
        if (write_stereo) {
            read_channels += Chan_Blue;
            read_channels += Chan_Alpha;
        }
        else {
            read_channels += Chan_Alpha;
        }

        input0().request(0, 0, width(), height(), read_channels, 1);

        Row row_in(0, image_w);
        std::vector<kiss_fft_cpx> ifft_input_L;
        std::vector<float> time_domain_L;
        std::vector<kiss_fft_cpx> ifft_input_R;
        std::vector<float> time_domain_R;
        std::vector<unsigned char> pcm_buffer;

        try {
            ifft_input_L.resize(N_fft / 2 + 1);
            time_domain_L.resize(N_fft);
            if (write_stereo) {
                ifft_input_R.resize(N_fft / 2 + 1);
                time_domain_R.resize(N_fft);
            }
        }
        catch (const std::bad_alloc& e) {
            iop->error("WavWriter: Failed to allocate memory for initial FFT buffers: %s", e.what());
            close();
            return;
        }

        kiss_fftr_cfg ifft_cfg_L = kiss_fftr_alloc(N_fft, 1, nullptr, nullptr);
        kiss_fftr_cfg ifft_cfg_R = nullptr;

        if (!ifft_cfg_L) {
            iop->error("WavWriter: Failed to allocate kiss_fftr_cfg for Left/Mono IFFT.");
            close();
            return;
        }
        if (write_stereo) {
            ifft_cfg_R = kiss_fftr_alloc(N_fft, 1, nullptr, nullptr);
            if (!ifft_cfg_R) {
                iop->error("WavWriter: Failed to allocate kiss_fftr_cfg for Right IFFT.");
                if (ifft_cfg_L) kiss_fft_free(ifft_cfg_L);
                close();
                return;
            }
        }

        if (iop->aborted()) { 
            iop->error("WavWriter: Nuke aborted before Y loop");
        }

        long current_audio_write_offset = audio_data_start_offset;
        bool loop_aborted_by_error = false;

        for (int y = 0; y < image_h; ++y) {
            if (iop->aborted()) {
                iop->error("WavWriter: Processing aborted by user request or upstream error before row %d.", y);
                loop_aborted_by_error = true;
                break;
            }
            iop->status(float(y + 1) / float(image_h));

            get(y, 0, width(), read_channels, row_in);

            if (iop->aborted()) {
                iop->error("WavWriter: Nuke aborted operation after get", y);
                loop_aborted_by_error = true;
                break;
            }

            try {
                for (int x_col = 0; x_col < image_w; ++x_col) {
                    if (_input_type_knob_value == 0)
                    {
                        float db_val_L_norm = row_in[Chan_Red][x_col];
                        float phase_val_L_norm = row_in[Chan_Green][x_col];

                        if (!std::isfinite(db_val_L_norm) || !std::isfinite(phase_val_L_norm)) {
                            iop->error("WavWriter: NaN/inf detected in input pixel L/Mono at row %d, col %d. R=%f, G=%f", y, x_col, db_val_L_norm, phase_val_L_norm);
                            loop_aborted_by_error = true; break;
                        }


						float db_L = db_val_L_norm * (_max_db_knob_value - _min_db_knob_value) + _min_db_knob_value;  // Convert normalized dB to actual dB value
                        float amplitude_L_estimate = powf(10.0f, db_L / 20.0f) - 1e-12f; // Convert back from dB to amplitude. Apparently I'm not supposed to subtract epsilon here, but I feel this is a closer inverse to what we do in the read.
						if (amplitude_L_estimate < 0.0f) {
							amplitude_L_estimate = 0.0f; // Clamp to zero if negative, can happen with very low dB values, and my stubbornness to subtract epsilon.
						}
						float phase_L_rad = (phase_val_L_norm * 2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI); // Convert normalized phase to radians, range [-pi, pi] 

                        if (!std::isfinite(amplitude_L_estimate) || !std::isfinite(phase_L_rad)) {
                            iop->error("WavWriter: NaN/inf in L/Mono amplitude/phase calc at row %d, col %d. Amp=%f, PhaseRad=%f", y, x_col, amplitude_L_estimate, phase_L_rad);
                            loop_aborted_by_error = true; break;
                        }

                        // De-normalize the magnitude
                        float kiss_mag_L;
                        if (x_col == 0 || (N_fft > 0 && x_col == N_fft / 2)) {
                            kiss_mag_L = amplitude_L_estimate * static_cast<float>(N_fft);
                        }
                        else {
                            kiss_mag_L = amplitude_L_estimate * (static_cast<float>(N_fft) / 2.0f);
                        }

                        ifft_input_L[x_col].r = kiss_mag_L * cosf(phase_L_rad);
                        ifft_input_L[x_col].i = kiss_mag_L * sinf(phase_L_rad);

                        if (!std::isfinite(ifft_input_L[x_col].r) || !std::isfinite(ifft_input_L[x_col].i)) {
                            iop->error("WavWriter: NaN/inf in L/Mono IFFT input complex pair at row %d, col %d. Real=%f, Imag=%f", y, x_col, ifft_input_L[x_col].r, ifft_input_L[x_col].i);
                            loop_aborted_by_error = true; break;
                        }


                        if (write_stereo) {
                            float db_val_R_norm = row_in[Chan_Blue][x_col];
                            float phase_val_R_norm = row_in[Chan_Alpha][x_col];

                            if (!std::isfinite(db_val_R_norm) || !std::isfinite(phase_val_R_norm)) {
                                iop->error("WavWriter: NaN/inf detected in input pixel R at row %d, col %d. B=%f, A=%f", y, x_col, db_val_R_norm, phase_val_R_norm);
                                loop_aborted_by_error = true; break;
                            }


                            float db_R = db_val_R_norm * (_max_db_knob_value - _min_db_knob_value) + _min_db_knob_value;
                            float amplitude_R_estimate = powf(10.0f, db_R / 20.0f);
                            float phase_R_rad = (phase_val_R_norm * 2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI);

                            if (!std::isfinite(amplitude_R_estimate) || !std::isfinite(phase_R_rad)) {
                                iop->error("WavWriter: NaN/inf in R amplitude/phase calc at row %d, col %d. Amp=%f, PhaseRad=%f", y, x_col, amplitude_R_estimate, phase_R_rad);
                                loop_aborted_by_error = true; break;
                            }

                            float kiss_mag_R;
                            if (x_col == 0 || (N_fft > 0 && x_col == N_fft / 2)) {
                                kiss_mag_R = amplitude_R_estimate * static_cast<float>(N_fft);
                            }
                            else {
                                kiss_mag_R = amplitude_R_estimate * (static_cast<float>(N_fft) / 2.0f);
                            }

                            ifft_input_R[x_col].r = kiss_mag_R * cosf(phase_R_rad);
                            ifft_input_R[x_col].i = kiss_mag_R * sinf(phase_R_rad);

                            if (!std::isfinite(ifft_input_R[x_col].r) || !std::isfinite(ifft_input_R[x_col].i)) {
                                iop->error("WavWriter: NaN/inf in R IFFT input complex pair at row %d, col %d. Real=%f, Imag=%f", y, x_col, ifft_input_R[x_col].r, ifft_input_R[x_col].i);
                                loop_aborted_by_error = true; break;
                            }
                        }
                    }
                    else if (_input_type_knob_value == 1) // Raw FFT
                    {
                        float real_val_L = row_in[Chan_Red][x_col];
                        float imag_val_L = row_in[Chan_Green][x_col];
                        if (!std::isfinite(real_val_L) || !std::isfinite(imag_val_L)) {
                            iop->error("WavWriter: NaN/inf detected in input pixel L/Mono at row %d, col %d. R=%f, G=%f", y, x_col, real_val_L, imag_val_L);
                            loop_aborted_by_error = true; break;
                        }
                        ifft_input_L[x_col].r = real_val_L;
                        ifft_input_L[x_col].i = imag_val_L;

                        if (write_stereo) {
                            float real_val_R = row_in[Chan_Blue][x_col];
                            float imag_val_R = row_in[Chan_Alpha][x_col];
                            if (!std::isfinite(real_val_R) || !std::isfinite(imag_val_R)) {
                                iop->error("WavWriter: NaN/inf detected in input pixel R at row %d, col %d. B=%f, A=%f", y, x_col, real_val_R, imag_val_R);
                                loop_aborted_by_error = true; break;
                            }
                            ifft_input_R[x_col].r = real_val_R;
                            ifft_input_R[x_col].i = imag_val_R;
                        }
					}
                    else if (_input_type_knob_value == 2) // Magnitude only
                    {
						// Calculate phase on the fly to ensure a continuous wave at each frequency. Assume all waves started with no phase shift at the beginning (y=0)
						float magnitude_L = row_in[Chan_Red][x_col];
						if (!std::isfinite(magnitude_L)) {
							iop->error("WavWriter: NaN/inf detected in input pixel L/Mono at row %d, col %d. R=%f", y, x_col, magnitude_L);
							loop_aborted_by_error = true; break;
						}
						// Calculate the phase for this frequency bin based on the row index (time) and column index (frequency bin)
						// Get the frequency of the wave for this bin, which we can calculate via the sample rate, sample length, and frequency bin index. The result will be in hertz.
						float frequency = x_col * (static_cast<float>(_sample_rate_knob_value) / static_cast<float>(N_fft)); 
						// Now calculate the phase based on the row index (time) and frequency
                        // For each time sample, the wave progresses by a certain amount, which is related to the frequency and duration of the time sample
						float progress_per_sample = frequency * (1.0f / static_cast<float>(_sample_rate_knob_value)) * 2.0f * static_cast<float>(M_PI); // Progress in radians per sample
                        float phase_rad = (y * progress_per_sample);


						// Put this back into a complex number
						float kiss_mag_L = magnitude_L * static_cast<float>(N_fft); // Scale magnitude to match FFT bin size
						ifft_input_L[x_col].r = kiss_mag_L * cosf(phase_rad);
						ifft_input_L[x_col].i = kiss_mag_L * sinf(phase_rad);

                        if (write_stereo)
                        {
							float magnitude_R = row_in[Chan_Blue][x_col];
							if (!std::isfinite(magnitude_R)) {
								iop->error("WavWriter: NaN/inf detected in input pixel R at row %d, col %d. B=%f", y, x_col, magnitude_R);
								loop_aborted_by_error = true; break;
							}
                            // Same phase as the left channel
							float kiss_mag_R = magnitude_R * static_cast<float>(N_fft); // Scale magnitude to match FFT bin size
							ifft_input_R[x_col].r = kiss_mag_R * cosf(phase_rad);
							ifft_input_R[x_col].i = kiss_mag_R * sinf(phase_rad);
                        }
                    }
                    
                } // End column loop
                if (loop_aborted_by_error) break;

                kiss_fftri(ifft_cfg_L, ifft_input_L.data(), time_domain_L.data());
                if (write_stereo) {
                    kiss_fftri(ifft_cfg_R, ifft_input_R.data(), time_domain_R.data());
                }

                int bytes_per_sample_single_channel = _wav_header.bits_per_sample / 8;
                int num_output_channels = _wav_header.num_channels;
                try {
                    pcm_buffer.resize(N_fft * bytes_per_sample_single_channel * num_output_channels);
                }
                catch (const std::bad_alloc& e) {
                    iop->error("WavWriter: Failed to allocate memory for PCM buffer (row %d): %s", y, e.what());
                    loop_aborted_by_error = true;
                    break;
                }

                if (iop->aborted()) { 
                    iop->error("WavWriter: Nuke aborted operation before FFT", y);
                    loop_aborted_by_error = true;
                    break;
                }

                for (int i = 0; i < N_fft; ++i) {
                    time_domain_L[i] /= static_cast<float>(N_fft);
                    if (!std::isfinite(time_domain_L[i])) {
                        iop->error("WavWriter: NaN/inf in L/Mono time domain sample after IFFT at row %d, sample %d. Value: %f", y, i, time_domain_L[i]);
                        loop_aborted_by_error = true; break;
                    }
                    float sample_L_float = std::max(-1.0f, std::min(1.0f, time_domain_L[i]));
                    unsigned char* sample_L_ptr = pcm_buffer.data() + (i * num_output_channels * bytes_per_sample_single_channel);

                    switch (_bit_depth_knob_value) {
                    case BIT_DEPTH_16_PCM: { int16_t val = static_cast<int16_t>(sample_L_float * 32767.0f); memcpy(sample_L_ptr, &val, sizeof(int16_t)); break; }
                    case BIT_DEPTH_24_PCM: { int32_t val_32 = static_cast<int32_t>(sample_L_float * 8388607.0f); sample_L_ptr[0] = (val_32 & 0xFF); sample_L_ptr[1] = (val_32 >> 8) & 0xFF; sample_L_ptr[2] = (val_32 >> 16) & 0xFF; break; }
                    case BIT_DEPTH_32_FLOAT: { float val = sample_L_float; memcpy(sample_L_ptr, &val, sizeof(float)); break; }
                    case BIT_DEPTH_8_PCM: { *sample_L_ptr = static_cast<uint8_t>((sample_L_float * 127.5f) + 127.5f); break; }
                    }

                    if (write_stereo) {
                        time_domain_R[i] /= static_cast<float>(N_fft);
                        if (!std::isfinite(time_domain_R[i])) {
                            iop->error("WavWriter: NaN/inf in R time domain sample after IFFT at row %d, sample %d. Value: %f", y, i, time_domain_R[i]);
                            loop_aborted_by_error = true; break;
                        }
                        float sample_R_float = std::max(-1.0f, std::min(1.0f, time_domain_R[i]));
                        unsigned char* sample_R_ptr = sample_L_ptr + bytes_per_sample_single_channel;

                        switch (_bit_depth_knob_value) {
                        case BIT_DEPTH_16_PCM: { int16_t val = static_cast<int16_t>(sample_R_float * 32767.0f); memcpy(sample_R_ptr, &val, sizeof(int16_t)); break; }
                        case BIT_DEPTH_24_PCM: { int32_t val_32 = static_cast<int32_t>(sample_R_float * 8388607.0f); sample_R_ptr[0] = (val_32 & 0xFF); sample_R_ptr[1] = (val_32 >> 8) & 0xFF; sample_R_ptr[2] = (val_32 >> 16) & 0xFF; break; }
                        case BIT_DEPTH_32_FLOAT: { float val = sample_R_float; memcpy(sample_R_ptr, &val, sizeof(float)); break; }
                        case BIT_DEPTH_8_PCM: { *sample_R_ptr = static_cast<uint8_t>((sample_R_float * 127.5f) + 127.5f); break; }
                        }
                    }
                } // End sample loop
                if (iop->aborted()) { // Check immediately after write, as this can be a source of Nuke-side aborts
                    iop->error("WavWriter: Nuke aborted operation before audio data write for row %d.", y);
                    loop_aborted_by_error = true;
                    break;
                }
                if (loop_aborted_by_error) break;

                this->write(current_audio_write_offset, pcm_buffer.data(), pcm_buffer.size());
                if (iop->aborted()) { // Check immediately after write, as this can be a source of Nuke-side aborts
                    iop->error("WavWriter: Nuke aborted operation during/after audio data write for row %d.", y);
                    loop_aborted_by_error = true;
                    break;
                }
                _total_audio_bytes_written += pcm_buffer.size();
                current_audio_write_offset += pcm_buffer.size();

            }

            catch (const std::bad_alloc& e) {
                iop->error("WavWriter: Memory allocation error during row %d processing: %s", y, e.what());
                loop_aborted_by_error = true;
            }
            catch (const std::exception& e) {
                iop->error("WavWriter: Standard Exception during row %d processing: %s", y, e.what());
                loop_aborted_by_error = true;
            }
            catch (...) {
                iop->error("WavWriter: Unknown exception during row %d processing.", y);
                loop_aborted_by_error = true;
            }

            if (loop_aborted_by_error) break; // Break main row loop if any exception was caught

            if (iop->aborted()) { // Final check for this iteration
                if (!loop_aborted_by_error) {
                    iop->error("WavWriter: Processing aborted by user request or upstream error after completing row %d.", y);
                }
                loop_aborted_by_error = true;
                break;
            }
        } // End main row loop

        if (ifft_cfg_L) kiss_fft_free(ifft_cfg_L);
        if (ifft_cfg_R) kiss_fft_free(ifft_cfg_R);

        if (!loop_aborted_by_error && !iop->aborted()) {
            _data_chunk_header.data_size = _total_audio_bytes_written;
            _wav_header.wav_size = 36 + _data_chunk_header.data_size;

            this->write(0, &_wav_header, sizeof(WavFileHeader));
            this->write(sizeof(WavFileHeader), &_data_chunk_header, sizeof(WavDataChunkHeader));
        }
        else {
            iop->warning("WavWriter: Processing was aborted or encountered an error. Output file may be incomplete or deleted by Nuke.");
        }

        close();
    }

    static const Writer::Description d;
};

static Writer* build(Write* iop) {
    return new WavWriter(iop);
}

const Writer::Description WavWriter::d("wav\0", "WAV from Spectrogram (Phase)", build);

