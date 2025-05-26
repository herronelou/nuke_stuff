// WavReader.cpp
// Erwan Leroy - 2025

#include "DDImage/FileReader.h"
#include "DDImage/Row.h"
#include "DDImage/DDMath.h"
#include "DDImage/MetaData.h"
#include "DDImage/Knobs.h"
#include "DDImage/Reader.h" 
#include "DDImage/DDString.h"
#include "DDImage/Iop.h"
#include "DDImage/ARRAY.h"
#include "DDImage/Thread.h"
#include "DDImage/Knob.h"
#include "DDImage/Memory.h"

#include <stdio.h>
#include <vector>
#include <cmath>    
#include <algorithm> 
#include <stdexcept> 
#include <string.h> 
#include <limits>

// Platform-specific file seeking
#ifdef _WIN32
#define FSEEK64 _fseeki64
#define FTELL64 _ftelli64
#else
#define FSEEK64 fseeko
#define FTELL64 ftello
#endif

// KissFFT headers
#include "kiss_fftr.h"
#include "kiss_fft.h" // For kiss_fft_cpx

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constants for image dimensions - these act as upper limits for rows per Nuke frame.
// Width is now determined by FFT output size.
#define MAX_IMAGE_HEIGHT_ROWS 8192    // Max rows per Nuke image (image height)
#define MAX_SAMPLES_PER_ROW_FOR_CALC 4096 // Upper limit for _samples_per_row before FFT output calc

using namespace DD::Image;

#pragma pack(push, 1)
struct WavHeader {
    char riff_header[4];
    uint32_t wav_size;
    char wave_header[4];
    char fmt_header[4];
    uint32_t fmt_chunk_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
};
struct WavDataChunkHeader {
    char data_header[4];
    uint32_t data_size;
};
#pragma pack(pop)


class WavReaderFormat : public ReaderFormat {
public:
	float _max_db_knob_value; // Maximum dB value for audio
	float _min_db_knob_value; // Minimum dB value for audio
    double _row_duration_ms_knob_value;
    bool _separate_magnitude_and_phase;
    

    WavReaderFormat() {
        _row_duration_ms_knob_value = (1024.0 / 44100.0) * 1000.0;
		_separate_magnitude_and_phase = true;
		// Default values for knobs
		_max_db_knob_value = 0.0f; // Default max dB
		_min_db_knob_value = -60.0f; // Default min dB
    }

    void knobs(Knob_Callback cb) override {
		Float_knob(cb, &_max_db_knob_value, "max_db", "Max dB");
		SetRange(cb, -50.0f, 10.0f);
        Tooltip(cb, "The maximum dB value corresponding to white (1.0) in the input spectrogram's magnitude channels. ");
		Float_knob(cb, &_min_db_knob_value, "min_db", "Min dB");
		SetRange(cb, -100.0f, 0.0f);
		Tooltip(cb, "The minimum dB value corresponding to black (0.0) in the input spectrogram's magnitude channels. ");
        Double_knob(cb, &_row_duration_ms_knob_value, "row_duration_ms", "Row Duration (ms)");
        SetRange(cb, 10.0, 500.0);
        Tooltip(cb, "Sets the audio duration for FFT analysis per image row, in milliseconds. "
            "This, with the sample rate, determines N_fft for FFT. Image width becomes N_fft/2 + 1. "
            "Changes apply when the Read node is created or its file path re-evaluated.");
		Bool_knob(cb, &_separate_magnitude_and_phase, "separate_magnitude_and_phase", "Separate Magnitude and Phase");
		SetFlags(cb, Knob::STARTLINE);
		Tooltip(cb, "If checked, the audio ampliture and phase will be broken out in different channels (Amplitude (L) in Red, Phase (L) in Green, Amplitude (R) in Blue, Phase (R) in Alpha.\n"
                    "If unchecked, the vector (complex number out of the FFT) will directly be stored in RG (L) and BA (R)");
    }

    void append(Hash& hash) override {
		hash.append(_max_db_knob_value);
		hash.append(_min_db_knob_value);
        hash.append(_row_duration_ms_knob_value);
		hash.append(_separate_magnitude_and_phase);
    }

	float getMaxDb() const {
		return _max_db_knob_value;
	}

	float getMinDb() const {
		return _min_db_knob_value;
	}

    double getRowDurationMs() const {
        return _row_duration_ms_knob_value;
    }

	bool getSeparateMagnitudeAndPhase() const {
		return _separate_magnitude_and_phase;
	}
};


class WavReader : public Reader {
private:
    FILE* _file_ptr;
    WavHeader _wav_header_main;
    WavDataChunkHeader _data_chunk_info;
    int64_t _data_chunk_file_pos;

	// Knob values
	float _max_db_knob_value; // Maximum dB value for audio
	float _min_db_knob_value; // Minimum dB value for audio
    double _row_duration_ms;
    unsigned int _samples_per_row;   // N_fft: Number of audio samples for FFT input per row (must be even)

    unsigned int _actual_image_width_x;  // Width of the Nuke image: _samples_per_row / 2 + 1
    unsigned int _image_height_y;

    long long _total_sample_frames;
    long long _total_rows_in_wav;
    unsigned int _num_nuke_frames;

	bool _calculate_phase;

    MetaData::Bundle _meta_data;
    Lock lock;
    bool _initialization_failed;

    const MetaData::Bundle& fetchMetaData(const char* key) override
    {
        return _meta_data;
    }

public:
    WavReader(Read* iop_arg, int fd) :
        Reader(iop_arg), // Use iop_arg for Reader constructor
        _file_ptr(nullptr),
        _data_chunk_file_pos(0),
		_max_db_knob_value(0.0f),
		_min_db_knob_value(-60.0f),
        _row_duration_ms(23.22),
        _samples_per_row(0),
        _actual_image_width_x(0),
        _image_height_y(0),
        _total_sample_frames(0),
        _total_rows_in_wav(0),
        _num_nuke_frames(1),
        _initialization_failed(true),
		_calculate_phase(true)
    {
        // Ensure iop (member via Reader base class) is not null
        if (this->iop == nullptr) { // Use this->iop to refer to the member from Reader
            // Cannot call iop->error() if iop is null.
            // Throw or handle error in a way that doesn't crash.
            // For now, we rely on the fact that Reader base constructor should handle iop.
            // If iop_arg was null, Reader constructor might have issues.
            // This state should ideally be prevented by Nuke.
            return;
        }
        _file_ptr = fdopen(fd, "rb");
        if (!_file_ptr) {
            this->iop->error("WavReader: Could not fdopen the provided file descriptor %d.", fd);
            return;
        }

        if (FSEEK64(_file_ptr, 0, SEEK_SET) != 0) {
            this->iop->error("WavReader: Could not FSEEK64 to start of file stream after fdopen.");
            return;
        }

        memset(&_wav_header_main, 0, sizeof(WavHeader));
        if (fread(&_wav_header_main, sizeof(WavHeader), 1, _file_ptr) != 1) {
            this->iop->error("WavReader: Failed to read initial WAV header.");
            return;
        }

        if (strncmp(_wav_header_main.riff_header, "RIFF", 4) != 0 ||
            strncmp(_wav_header_main.wave_header, "WAVE", 4) != 0 ||
            strncmp(_wav_header_main.fmt_header, "fmt ", 4) != 0) {
            this->iop->error("WavReader: Not a valid WAV file (RIFF/WAVE/fmt markers not found).");
            return;
        }

        if (!(_wav_header_main.audio_format == 1 /*PCM*/ || _wav_header_main.audio_format == 3 /*IEEE Float*/)) {
            this->iop->error("WavReader: Unsupported audio format (0x%X). Only PCM (1) or IEEE Float (3) are supported.", _wav_header_main.audio_format);
            return;
        }
        if (_wav_header_main.audio_format == 1 && !(_wav_header_main.bits_per_sample == 8 || _wav_header_main.bits_per_sample == 16 || _wav_header_main.bits_per_sample == 24 || _wav_header_main.bits_per_sample == 32)) {
            this->iop->error("WavReader: For PCM format, only 8, 16, 24, or 32-bit are supported. Got %d bits.", _wav_header_main.bits_per_sample);
            return;
        }
        if (_wav_header_main.audio_format == 3 && _wav_header_main.bits_per_sample != 32) {
            this->iop->error("WavReader: For IEEE Float format, only 32-bit WAV files are supported.");
            return;
        }
        if (_wav_header_main.sample_rate == 0) {
            this->iop->error("WavReader: Sample rate is zero.");
            return;
        }
        if (_wav_header_main.num_channels == 0 || _wav_header_main.num_channels > 8) { // Increased channel limit slightly, though FFT uses first
            this->iop->warning("WavReader: Number of channels is %d. Spectrogram will use the first channel.", _wav_header_main.num_channels);
        if (_wav_header_main.num_channels == 0) {
                this->iop->error("WavReader: Number of channels is zero.");
            return;
        }
        }
        if (_wav_header_main.block_align == 0) {
            this->iop->error("WavReader: WAV header block_align is zero.");
            return;
        }

        if (_wav_header_main.fmt_chunk_size > 16) {
            long offset_to_skip = _wav_header_main.fmt_chunk_size - 16;
            if (FSEEK64(_file_ptr, offset_to_skip, SEEK_CUR) != 0) {
                this->iop->error("WavReader: Failed to seek past %ld bytes of extra format data.", offset_to_skip);
                return;
            }
        }

        memset(&_data_chunk_info, 0, sizeof(WavDataChunkHeader));
        char chunk_id_arr[4];
        uint32_t current_chunk_size_val;
        bool data_chunk_found = false;
        int64_t current_pos_val;
        int64_t limit_search_pos_val = static_cast<int64_t>(_wav_header_main.wav_size) + 8 - sizeof(WavDataChunkHeader); // Theoretical end of RIFF chunk

        current_pos_val = FTELL64(_file_ptr);
        if (current_pos_val == -1L) {
            this->iop->error("WavReader: FTELL64 failed before searching for 'data' chunk.");
            return;
        }
        // If wav_size is small or potentially incorrect, provide a reasonable search limit.
        if (limit_search_pos_val <= current_pos_val || _wav_header_main.wav_size < sizeof(WavHeader) + sizeof(WavDataChunkHeader)) {
            limit_search_pos_val = current_pos_val + 1024 * 1024 * 5; // Search up to 5MB past current for 'data' if header seems off
        }


        int safety_counter = 0;
        while ((current_pos_val = FTELL64(_file_ptr)) != -1 && current_pos_val < limit_search_pos_val && safety_counter < 1000) {
            safety_counter++;
            if (fread(chunk_id_arr, sizeof(char), 4, _file_ptr) != 4) { break; }
            if (fread(&current_chunk_size_val, sizeof(uint32_t), 1, _file_ptr) != 1) { break; }

            if (strncmp(chunk_id_arr, "data", 4) == 0) {
                memcpy(_data_chunk_info.data_header, chunk_id_arr, 4);
                _data_chunk_info.data_size = current_chunk_size_val;
                _data_chunk_file_pos = FTELL64(_file_ptr);
                if (_data_chunk_file_pos == -1L) { this->iop->error("WavReader: Error getting 'data' chunk start pos."); return; }
                data_chunk_found = true;
                break;
            }
            else {
                // Check for huge chunk sizes that might indicate a corrupt file or going past file end
                if (current_chunk_size_val > (limit_search_pos_val - (current_pos_val + 8)) && current_chunk_size_val > 0) { // +8 for chunk_id and size itself
                    this->iop->warning("WavReader: Chunk '%.4s' size %u seems too large for remaining file. Stopping search.", chunk_id_arr, current_chunk_size_val);
                    break;
                }
                if (FSEEK64(_file_ptr, current_chunk_size_val, SEEK_CUR) != 0) {
                    // If seek fails, we might be at EOF or have an issue.
                    // this->iop->warning("WavReader: Failed to seek past chunk '%.4s'. Might be near EOF.", chunk_id_arr); 
                    break;
                }
            }
        }

        if (!data_chunk_found) { this->iop->error("WavReader: 'data' chunk not found. File may be corrupt or not a standard WAV."); return; }
        if (_data_chunk_info.data_size == 0) { this->iop->warning("WavReader: 'data' chunk has zero size. No audio data to process."); }

        WavReaderFormat* readerFormat = dynamic_cast<WavReaderFormat*>(this->iop->handler());
        if (!readerFormat) {
            this->iop->error("WavReader: Failed to get WavReaderFormat from IOP.");
            return;
        }
		_max_db_knob_value = readerFormat->getMaxDb();
		_min_db_knob_value = readerFormat->getMinDb();
        _row_duration_ms = readerFormat->getRowDurationMs();
        _samples_per_row = static_cast<unsigned int>((_row_duration_ms / 1000.0) * _wav_header_main.sample_rate);
		_calculate_phase = readerFormat->getSeparateMagnitudeAndPhase();

        // Ensure _samples_per_row (N_fft) is even and at least 2 for kiss_fftr
        if (_samples_per_row == 0) {
            _samples_per_row = 2;
        }
        else if (_samples_per_row % 2 != 0) {
            _samples_per_row++; // Make it even
        }
        // Safety cap for N_fft, though MAX_SAMPLES_PER_ROW_FOR_CALC is more of a guideline for duration knob
        if (_samples_per_row > MAX_SAMPLES_PER_ROW_FOR_CALC) {
            this->iop->warning("WavReader: Calculated N_fft (%u) is very large. Clamping to %d.", _samples_per_row, MAX_SAMPLES_PER_ROW_FOR_CALC);
            _samples_per_row = MAX_SAMPLES_PER_ROW_FOR_CALC;
            if (_samples_per_row % 2 != 0) _samples_per_row--; // Ensure it's still even after clamping
        }


        // Image width is N_fft / 2 + 1
        _actual_image_width_x = _samples_per_row / 2 + 1;
        if (_actual_image_width_x == 0) _actual_image_width_x = 1; // Should not happen if _samples_per_row >= 2

        if (_wav_header_main.block_align > 0) {
            _total_sample_frames = _data_chunk_info.data_size / _wav_header_main.block_align;
        }
        else {
            _total_sample_frames = 0;
        }

        if (_samples_per_row == 0) { // Should have been caught and set to 2
            this->iop->error("WavReader: N_fft (_samples_per_row) is zero after adjustments."); return;
        }
        _total_rows_in_wav = (_total_sample_frames + _samples_per_row - 1) / _samples_per_row;
        if (_total_rows_in_wav == 0 && _total_sample_frames > 0) _total_rows_in_wav = 1;


        if (_total_rows_in_wav == 0) {
            _image_height_y = 1;
            _num_nuke_frames = 1;
        }
        else if (static_cast<unsigned int>(_total_rows_in_wav) <= MAX_IMAGE_HEIGHT_ROWS) {
            _image_height_y = static_cast<unsigned int>(_total_rows_in_wav);
            _num_nuke_frames = 1;
        }
        else {
            _image_height_y = MAX_IMAGE_HEIGHT_ROWS;
            if (_image_height_y == 0) { this->iop->error("WavReader: MAX_IMAGE_HEIGHT_ROWS is zero."); return; } // Should not happen
            _num_nuke_frames = static_cast<unsigned int>((_total_rows_in_wav + _image_height_y - 1) / _image_height_y);
        }
        if (_image_height_y == 0) _image_height_y = 1;

        DD::Image::ChannelSet output_channels;
        output_channels += DD::Image::Chan_Red;
        output_channels += DD::Image::Chan_Green;
        output_channels += DD::Image::Chan_Blue;
        output_channels += DD::Image::Chan_Alpha;

        set_info(_actual_image_width_x, _image_height_y, 4, 1.0);
        info_.channels(output_channels);
        info_.ydirection(1);

        this->iop->expectedFrameRange(1, _num_nuke_frames > 0 ? _num_nuke_frames : 1);

        _meta_data.setData(DD::Image::MetaData::FILENAME, filename());
        _meta_data.setData("wav/sample_rate", _wav_header_main.sample_rate);
        _meta_data.setData("wav/channels", _wav_header_main.num_channels);
        _meta_data.setData("wav/bits_per_sample", _wav_header_main.bits_per_sample);
        _meta_data.setData("wav/duration_seconds", static_cast<double>(_total_sample_frames) / (_wav_header_main.sample_rate > 0 ? _wav_header_main.sample_rate : 1.0));
        _meta_data.setData("image/row_duration_ms", _row_duration_ms);
        _meta_data.setData("image/fft_samples_per_row (N_fft)", _samples_per_row);
        _meta_data.setData("image/actual_width (N_fft/2+1)", _actual_image_width_x);
        _meta_data.setData("image/total_rows_in_wav", static_cast<double>(_total_rows_in_wav));
        _meta_data.setData("image/nuke_frames_total", _num_nuke_frames);

        _initialization_failed = false;
    }

    ~WavReader() override {
        if (_file_ptr) {
            fclose(_file_ptr);
            _file_ptr = nullptr;
        }
    }

    void open() override {
        // NOP
    }

    void engine(int y_row_in_nuke_img, int x_start_col, int r_end_col, DD::Image::ChannelMask channels, DD::Image::Row& out_row) override {
        if (_initialization_failed || !_file_ptr || _actual_image_width_x == 0) {
            foreach(z, channels) out_row.erase(z);
            if (channels & DD::Image::Chan_Alpha) {
                float* alpha_buf = out_row.writable(DD::Image::Chan_Alpha);
                if (alpha_buf) for (int i = x_start_col; i < r_end_col; ++i) alpha_buf[i] = 1.0f;
            }
            return;
        }

        int current_nuke_frame_num = 1;
        int first_nuke_frame = 1;
        if (iop) {
            current_nuke_frame_num = iop->outputContext().frame();
            first_nuke_frame = iop->first_frame();
        }
        if (current_nuke_frame_num < first_nuke_frame) current_nuke_frame_num = first_nuke_frame;

        long long overall_row_index_in_wav =
            (long long)(current_nuke_frame_num - first_nuke_frame) * _image_height_y + y_row_in_nuke_img;

        if (overall_row_index_in_wav < 0 || overall_row_index_in_wav >= _total_rows_in_wav) {
            foreach(z, channels) out_row.erase(z);
            if (channels & DD::Image::Chan_Alpha) {
                float* alpha_buf = out_row.writable(DD::Image::Chan_Alpha);
                if (alpha_buf) for (int i = x_start_col; i < r_end_col; ++i) alpha_buf[i] = 1.0f;
            }
            return;
        }

        long long start_sample_frame_for_row = overall_row_index_in_wav * _samples_per_row;
        int64_t file_offset_for_row_start = _data_chunk_file_pos + start_sample_frame_for_row * _wav_header_main.block_align;

        size_t bytes_per_sample_frame_local = _wav_header_main.block_align;
        size_t row_buffer_size_bytes = _samples_per_row * bytes_per_sample_frame_local;
        std::vector<unsigned char> row_audio_buffer;
        try {
            if (row_buffer_size_bytes > 0) {
                row_audio_buffer.resize(row_buffer_size_bytes);
            }
        }
        catch (const std::bad_alloc& e) {
            if (iop) iop->error("WavReader::engine: Failed to allocate row_audio_buffer: %s", e.what());
            foreach(z, channels) out_row.erase(z); if (channels & DD::Image::Chan_Alpha) { float* alpha_buf = out_row.writable(DD::Image::Chan_Alpha); if (alpha_buf) for (int i = x_start_col; i < r_end_col; ++i) alpha_buf[i] = 1.0f; }
            return;
        }

        size_t samples_to_read_for_this_row = _samples_per_row;
        if (start_sample_frame_for_row + _samples_per_row > _total_sample_frames) {
            if (_total_sample_frames > start_sample_frame_for_row) {
                samples_to_read_for_this_row = static_cast<size_t>(_total_sample_frames - start_sample_frame_for_row);
            }
            else {
                samples_to_read_for_this_row = 0;
            }
        }

        Guard guard(lock);

        size_t bytes_actually_read = 0;
        {
            Guard guard(lock); // Protect file access

            
            if (samples_to_read_for_this_row > 0 && !row_audio_buffer.empty()) {
                if (FSEEK64(_file_ptr, file_offset_for_row_start, SEEK_SET) != 0) {
                    if (iop) iop->error("WavReader: FSEEK64 failed in engine for row %lld.", overall_row_index_in_wav);
                    foreach(z, channels) out_row.erase(z); if (channels & DD::Image::Chan_Alpha) { float* alpha_buf = out_row.writable(DD::Image::Chan_Alpha); if (alpha_buf) for (int i = x_start_col; i < r_end_col; ++i) alpha_buf[i] = 1.0f; }
                    return;
                }
                bytes_actually_read = fread(row_audio_buffer.data(), 1, samples_to_read_for_this_row * bytes_per_sample_frame_local, _file_ptr);
                if (ferror(_file_ptr) && iop) {
                    iop->error("WavReader: File read error in engine for row %lld.", overall_row_index_in_wav);
                    clearerr(_file_ptr);
                    foreach(z, channels) out_row.erase(z); if (channels & DD::Image::Chan_Alpha) { float* alpha_buf = out_row.writable(DD::Image::Chan_Alpha); if (alpha_buf) for (int i = x_start_col; i < r_end_col; ++i) alpha_buf[i] = 1.0f; }
                    return;
                }
            }
        }

        size_t sample_frames_actually_read_for_row = (bytes_per_sample_frame_local > 0) ? (bytes_actually_read / bytes_per_sample_frame_local) : 0;

        int N_fft = _samples_per_row;
        std::vector<float> float_audio_L(N_fft, 0.0f);
        std::vector<float> float_audio_R(N_fft, 0.0f);
        bool is_stereo = (_wav_header_main.num_channels >= 2);

        uint16_t bits_per_sample_local = _wav_header_main.bits_per_sample;
        uint16_t bytes_per_single_sample = bits_per_sample_local / 8;

        for (int i = 0; i < N_fft; ++i) {
            if (static_cast<size_t>(i) < sample_frames_actually_read_for_row) {
                unsigned char* current_sample_frame_ptr = row_audio_buffer.data() + (i * bytes_per_sample_frame_local);

                // Process Left Channel (Channel 0)
                unsigned char* left_sample_data_ptr = current_sample_frame_ptr;
                float sample_val_L = 0.0f;
                if (_wav_header_main.audio_format == 3) {
                    if (bits_per_sample_local == 32) {
                        memcpy(&sample_val_L, left_sample_data_ptr, sizeof(float));
                    }
                }
                else {
                    int32_t raw_s_val = 0;
                    if (bits_per_sample_local == 8) { raw_s_val = *left_sample_data_ptr; sample_val_L = (static_cast<float>(raw_s_val) - 127.5f) / 127.5f; }
                    else if (bits_per_sample_local == 16) { int16_t temp_s16; memcpy(&temp_s16, left_sample_data_ptr, sizeof(int16_t)); sample_val_L = static_cast<float>(temp_s16) / 32768.0f; }
                    else if (bits_per_sample_local == 24) { raw_s_val = (left_sample_data_ptr[2] << 24) | (left_sample_data_ptr[1] << 16) | (left_sample_data_ptr[0] << 8); raw_s_val >>= 8; sample_val_L = static_cast<float>(raw_s_val) / 8388608.0f; }
                    else if (bits_per_sample_local == 32) { memcpy(&raw_s_val, left_sample_data_ptr, sizeof(int32_t)); sample_val_L = static_cast<float>(raw_s_val) / 2147483648.0f; }
                }
                float_audio_L[i] = sample_val_L;

                // Process Right Channel (Channel 1) if stereo
                if (is_stereo) {
                    unsigned char* right_sample_data_ptr = current_sample_frame_ptr + bytes_per_single_sample;
                    float sample_val_R = 0.0f;
                    if (_wav_header_main.audio_format == 3) {
                        if (bits_per_sample_local == 32) {
                            memcpy(&sample_val_R, right_sample_data_ptr, sizeof(float));
                        }
                    }
                    else {
                        int32_t raw_s_val = 0;
                        if (bits_per_sample_local == 8) { raw_s_val = *right_sample_data_ptr; sample_val_R = (static_cast<float>(raw_s_val) - 127.5f) / 127.5f; }
                        else if (bits_per_sample_local == 16) { int16_t temp_s16; memcpy(&temp_s16, right_sample_data_ptr, sizeof(int16_t)); sample_val_R = static_cast<float>(temp_s16) / 32768.0f; }
                        else if (bits_per_sample_local == 24) { raw_s_val = (right_sample_data_ptr[2] << 24) | (right_sample_data_ptr[1] << 16) | (right_sample_data_ptr[0] << 8); raw_s_val >>= 8; sample_val_R = static_cast<float>(raw_s_val) / 8388608.0f; }
                        else if (bits_per_sample_local == 32) { memcpy(&raw_s_val, right_sample_data_ptr, sizeof(int32_t)); sample_val_R = static_cast<float>(raw_s_val) / 2147483648.0f; }
                    }
                    float_audio_R[i] = sample_val_R;
                }
            }
            // Else: samples remain 0.0f (zero padding)
        }

        // FFT for Left Channel
        kiss_fftr_cfg fft_cfg_L = kiss_fftr_alloc(N_fft, 0, nullptr, nullptr);
        if (!fft_cfg_L) { /* ... error handling ... */ return; }
        std::vector<kiss_fft_cpx> fft_out_L(N_fft / 2 + 1);
        kiss_fftr(fft_cfg_L, float_audio_L.data(), fft_out_L.data());
        kiss_fft_free(fft_cfg_L);

        // FFT for Right Channel (if stereo)
        kiss_fftr_cfg fft_cfg_R = nullptr;
        std::vector<kiss_fft_cpx> fft_out_R;
        if (is_stereo) {
            fft_cfg_R = kiss_fftr_alloc(N_fft, 0, nullptr, nullptr);
            if (!fft_cfg_R) { /* ... error handling ... */ return; }
            fft_out_R.resize(N_fft / 2 + 1);
            kiss_fftr(fft_cfg_R, float_audio_R.data(), fft_out_R.data());
            kiss_fft_free(fft_cfg_R);
        }

        float* r_ptr = (channels & DD::Image::Chan_Red) ? out_row.writable(DD::Image::Chan_Red) : nullptr;
        float* g_ptr = (channels & DD::Image::Chan_Green) ? out_row.writable(DD::Image::Chan_Green) : nullptr;
        float* b_ptr = (channels & DD::Image::Chan_Blue) ? out_row.writable(DD::Image::Chan_Blue) : nullptr;
        float* a_ptr = (channels & DD::Image::Chan_Alpha) ? out_row.writable(DD::Image::Chan_Alpha) : nullptr;

        int num_fft_bins = N_fft / 2 + 1;

		const float min_db = _min_db_knob_value;
		const float max_db = _max_db_knob_value;

        for (int x_col = x_start_col; x_col < r_end_col; ++x_col) {
            float r_val = 0.0f, g_val = 0.0f, b_val = 0.0f, a_val = 0.0f;

            if (x_col >= 0 && x_col < num_fft_bins) {
                kiss_fft_cpx cpx_L = fft_out_L[x_col];
				if (_calculate_phase) {
					// Calculate phase and magnitude for each FFT bin
                    // Left Channel (or Mono) Processing
                    
					float mag_L_raw = sqrtf(cpx_L.r * cpx_L.r + cpx_L.i * cpx_L.i);  // Vector magnitude
					float phase_L_rad = atan2f(cpx_L.i, cpx_L.r); // Phase in radians, between -pi and pi
					float norm_phase_L = (phase_L_rad + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)); // Normalize phase to [0, 1]

                    float amp_est_L;
					if (x_col == 0 || (N_fft > 0 && x_col == N_fft / 2)) { amp_est_L = mag_L_raw / static_cast<float>(N_fft); } // DC and Nyquist bins: divide by N_fft
					else { amp_est_L = mag_L_raw / (static_cast<float>(N_fft) / 2.0f); }  // Other bins: divide by N_fft/2, the /2 is because the values in the FFT for those bins is apparently scaled to take into account the symetry of the FFT? Not sure, trusting what I read online.

					float db_L = 20.0f * log10f(amp_est_L + 1e-12f);  // Convert amplitude to dB, adding a small epsilon to avoid log10(0).
					float display_db_L = (max_db > min_db) ? (db_L - min_db) / (max_db - min_db) : 0.0f; // Remap dB to [0, 1] range from min_db to max_db

                    r_val = display_db_L;
                    g_val = norm_phase_L;
				}
				else {
					// Use the complex output directly
                    r_val = cpx_L.r;
                    g_val = cpx_L.i;
				}


                if (is_stereo) {
                    kiss_fft_cpx cpx_R = fft_out_R[x_col];
					if (_calculate_phase) {
                        float mag_R_raw = sqrtf(cpx_R.r * cpx_R.r + cpx_R.i * cpx_R.i);
                        float phase_R_rad = atan2f(cpx_R.i, cpx_R.r);
                        float norm_phase_R = (phase_R_rad + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI));

                        float amp_est_R;
                        if (x_col == 0 || (N_fft > 0 && x_col == N_fft / 2)) { amp_est_R = mag_R_raw / static_cast<float>(N_fft); }
                        else { amp_est_R = mag_R_raw / (static_cast<float>(N_fft) / 2.0f); }

                        float db_R = 20.0f * log10f(amp_est_R + 1e-9f);
                        float display_db_R = (max_db > min_db) ? (db_R - min_db) / (max_db - min_db) : 0.0f;

                        b_val = display_db_R;
                        a_val = norm_phase_R;
					}
                    else
					{
						// Use the complex output directly
						b_val = cpx_R.r;
						a_val = cpx_R.i;
					}
                }
                else {
                    // Mono: Blue is 0, Alpha is 1
                    b_val = 0.0f;
                    a_val = 1.0f;
                }
            }
            // else: x_col is outside requested range, values remain default (black)
            // This shouldn't happen if r_end_col <= _actual_image_width_x

            if (r_ptr) r_ptr[x_col] = r_val;
            if (g_ptr) g_ptr[x_col] = g_val;
            if (b_ptr) b_ptr[x_col] = b_val;
            if (a_ptr) a_ptr[x_col] = a_val;
        }
    }

    static const DD::Image::Reader::Description d;
};


static Reader* build(Read* iop, int fd, const unsigned char*, int)
{
    return new WavReader(iop, fd);
}

static bool test(int fd, const unsigned char* block, int length)
{
    if (length < 12) return false;
    return memcmp(block, "RIFF", 4) == 0 && memcmp(block + 8, "WAVE", 4) == 0;
}


static ReaderFormat* buildformat(Read* iop)
{
    return new WavReaderFormat();
}

const DD::Image::Reader::Description WavReader::d("wav\0", "Raw Audio Waveform (WAV)", build, test, buildformat);