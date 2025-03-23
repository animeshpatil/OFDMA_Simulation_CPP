#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <complex>
#include <cstdlib>
#include <windows.h>
#include <random>
#include <utility>
#include <cmath>

constexpr int FFT_SIZE = 64;
constexpr int NUM_ACTIVE = 8;
constexpr int ACTIVE_BIN_SPACING = FFT_SIZE / NUM_ACTIVE; // = 8

// Control codes
constexpr int CTRL_ACCESS_REQUEST = 0; // 00
constexpr int CTRL_DATA_TX        = 1; // 01
constexpr int CTRL_RESPONSE       = 2; // 10
constexpr int CTRL_DEALLOCATE     = 3; // 11

constexpr double NOISE_VARIANCE = 0.001;
constexpr const char* BS_RX_FILE = "rxbuffer_files/bs_rx_waveform.txt";

std::complex<double> qpskModulate(int bit1, int bit2);
std::pair<int,int> qpskDemodulate(const std::complex<double>& sym);

std::vector<std::complex<double>> fft(const std::vector<std::complex<double>>& a);
std::vector<std::complex<double>> ifft(const std::vector<std::complex<double>>& A);

void addAWGN(std::vector<std::complex<double>>& sig, double var);

void writeWaveform(const std::string& filename, const std::vector<std::complex<double>>& wave);
std::vector<std::complex<double>> readWaveform(const std::string& filename);
void clearFile(const std::string& filename);