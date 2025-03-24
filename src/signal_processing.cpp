#define _USE_MATH_DEFINES
#include "signal_processing.h"

std::complex<double> qpskModulate(int bit1, int bit2)
{
    double invSqrt2 = 1.0 / sqrt(2.0);
    if(bit1==0 && bit2==0) return {invSqrt2,  invSqrt2};
    if(bit1==0 && bit2==1) return {invSqrt2, -invSqrt2};
    if(bit1==1 && bit2==0) return {-invSqrt2, invSqrt2};
    return {-invSqrt2, -invSqrt2};
}

std::pair<int,int> qpskDemodulate(const std::complex<double>& sym)
{
    int b1 = (sym.real()<0)?1:0;
    int b2 = (sym.imag()<0)?1:0;
    return {b1,b2};
}

std::vector<std::complex<double>> fft(const std::vector<std::complex<double>>& a)
{
    int n = a.size();
    if (n <= 1) return a;
    
    // Divide: separate a into even and odd indexed elements.
    std::vector<std::complex<double>> a_even(n/2), a_odd(n/2);
    for (int i = 0; i < n/2; i++)
	{
        a_even[i] = a[2*i];
        a_odd[i] = a[2*i + 1];
    }
    
    // Recursively compute FFT on even and odd parts.
    std::vector<std::complex<double>> y_even = fft(a_even);
    std::vector<std::complex<double>> y_odd  = fft(a_odd);
    
    // Combine
    std::vector<std::complex<double>> y(n);
    for (int k = 0; k < n/2; k++)
	{
        std::complex<double> t = std::polar(1.0, -2 * M_PI * k / n) * y_odd[k];
        y[k] = y_even[k] + t;
        y[k + n/2] = y_even[k] - t;
    }
    return y;
}

// Inverse FFT: computes the inverse FFT of a vector of complex<double>
// It works by conjugating the input, computing the FFT, then conjugating the result and dividing by n.
std::vector<std::complex<double>> ifft(const std::vector<std::complex<double>>& A)
{
    int n = A.size();
    std::vector<std::complex<double>> A_conj(n);
    for (int i = 0; i < n; i++)
	{
        A_conj[i] = std::conj(A[i]);
    }
    // Compute FFT of conjugated vector.
    std::vector<std::complex<double>> y = fft(A_conj);
    // Conjugate the result and divide by n.
    for (int i = 0; i < n; i++)
	{
        y[i] = std::conj(y[i]) / static_cast<double>(n);
    }
    return y;
}

void addAWGN(std::vector<std::complex<double>>& sig, double var)
{
    std::default_random_engine gen;
    std::normal_distribution<double> dist(0.0, sqrt(var));
    for(auto &s: sig)
	{
        s += std::complex<double>(dist(gen), dist(gen));
    }
}

void writeWaveform(const std::string& filename, const std::vector<std::complex<double>>& wave)
{
    std::ofstream ofs(filename);
    if(!ofs)
	{
        std::cerr<<"Error writing to "<<filename<<std::endl;
        return;
    }
    for(auto &sm : wave)
	{
        ofs<<sm.real()<<" "<<sm.imag()<<"\n";
    }
    ofs.close();
}

// Read waveform from file
std::vector<std::complex<double>> readWaveform(const std::string& filename)
{
    std::vector<std::complex<double>> wave;
    std::ifstream ifs(filename);
    if (!ifs) return wave;
    double re, im;
    while (ifs >> re >> im)
	{
        wave.push_back({ re, im });
    }
    ifs.close();
    return wave;
}

// Clear file
void clearFile(const std::string& filename)
{
    std::ofstream ofs(filename, std::ofstream::trunc);
    ofs.close();
}