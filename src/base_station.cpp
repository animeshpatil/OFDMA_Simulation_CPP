#define _USE_MATH_DEFINES
#include "signal_processing.h"
#include <map>

using namespace std;

bool usedBins[FREQ_BINS] = { true, true, true, false, false, false, false, false }; // Bin 0 -> CTRL Code, Bin 1 -> DST ID, Bin 2 -> SRC ID
std::map<int, std::pair<int,int>> allocation; // user -> (start_bin, allocated_bins)

std::pair<int,int> allocateBins(int requested)
{
    if (requested < 1) requested = 1;
    if (requested > 3) requested = 3;
    int allocated = 0;
    int start = -1;
	
    for (int i = 1; i < FREQ_BINS; i++)
	{
        if (!usedBins[i])
		{
            bool canAlloc = true;
			// Checking if there are consecutive bins available
            for (int j = i; j < i + requested; j++)
			{
                if (j >= FREQ_BINS || usedBins[j])
				{
                    canAlloc = false;
                    break;
                }
            }
			// If bins are available, allocate
            if (canAlloc)
			{
                start = i;
                allocated = requested;
                for (int j = i; j < i + allocated; j++)
                    usedBins[j] = true;
                break;
            }
        }
    }
	
	// If no consecutive bins available, try and assign lower number of bins
    if (allocated == 0)
	{
        for (int r = requested - 1; r >= 1; r--)
		{
            for (int i = 1; i < FREQ_BINS; i++)
			{
                if (!usedBins[i])
				{
                    bool canAlloc = true;
                    for (int j = i; j < i + r; j++)
					{
                        if (j >= FREQ_BINS || usedBins[j])
						{
                            canAlloc = false;
                            break;
                        }
                    }
                    if (canAlloc)
					{
                        start = i;
                        allocated = r;
                        for (int j = i; j < i + allocated; j++)
                            usedBins[j] = true;
                        goto done;
                    }
                }
            }
        }
    }
	done:
		return std::make_pair(start, allocated);
}

void deallocateBins(int userId)
{
    if (allocation.find(userId) != allocation.end())
	{
        int start = allocation[userId].first;
        int count = allocation[userId].second;
        for (int i = start; i < start + count; i++)
		{
            usedBins[i] = false;
        }
        allocation.erase(userId);
    }
}

int main()
{
    std::cout << "Base station simulation started" << std::endl;
    while (true)
	{
        std::vector<std::complex<double>> rxWave = readWaveform(BS_RX_FILE);
        if (rxWave.size() == FFT_SIZE)
		{
            auto fullFreq = fft(rxWave);	// N = 64 fft
            std::vector<std::complex<double>> active(FREQ_BINS);	// N = 8 bins
			
            for (int i = 0; i < FREQ_BINS; i++)
			{
                active[i] = fullFreq[i * FREQ_BIN_SPACING];
            }

			// Extract CTRL Code
            std::pair<int,int> ctrlBits = qpskDemodulate(active[0]);
            int ctrlVal = ctrlBits.first * 2 + ctrlBits.second;

            if (ctrlVal == CTRL_ACCESS_REQUEST)
			{
                // Extract User ID
                std::pair<int,int> userBits = qpskDemodulate(active[1]);
                int userId = userBits.first * 2 + userBits.second;

				// Extract Requested Bin Count
                std::pair<int,int> reqBits = qpskDemodulate(active[2]);
                int requested = reqBits.first * 2 + reqBits.second;

                std::cout << "Access request from user " << userId << " requesting " << requested << " bins.\n";
                
				// Allocation of bins
                std::pair<int,int> allocRes = allocateBins(requested);
                int start = allocRes.first;
                int count = allocRes.second;

                if (start < 0 || count == 0)
				{
                    std::cout << "Could not allocate bins for user " << userId << std::endl;
                }
				else
				{
                    allocation[userId] = std::make_pair(start, count);
                    std::cout << "Allocated " << count << " bins to user " << userId 
                              << " starting at active bin " << start << std::endl;
                }

                // Response
                std::vector<std::complex<double>> respFull(FFT_SIZE, {0,0});
                std::vector<std::complex<double>> activeResp(FREQ_BINS, {0,0});
				
                // Bin0 -> CTRL_RESPONSE
                activeResp[0] = qpskModulate(1,0);
                // Bin1 -> User ID
                activeResp[1] = qpskModulate((userId>>1)&1, userId&1);
                // Bin2 -> Allocated Count
                activeResp[2] = qpskModulate((count>>1)&1, count&1);
                // [Bin3, Bin4] -> startBin
                int sVal = start & 0xF;
                int hi = (sVal >> 2) & 0x3;
                int lo = sVal & 0x3;
                activeResp[3] = qpskModulate((hi>>1)&1, hi & 1);
                activeResp[4] = qpskModulate((lo>>1)&1, lo & 1);
				
				// No Data in unused bins
                for(int i=5; i<FREQ_BINS; i++)
                    activeResp[i] = qpskModulate(0,0);
				
				// Upsampling to N = 64
                for(int i=0; i<FREQ_BINS; i++)
                    respFull[i * FREQ_BIN_SPACING] = activeResp[i];
				
				// Construct time-domain signal and write to user rx-buffer
                auto respTime = ifft(respFull);
                addAWGN(respTime, NOISE_VARIANCE);
                std::string userRx = "rxbuffer_files/user" + std::to_string(userId) + "_rx_waveform.txt";
                writeWaveform(userRx, respTime);
            }
            else if (ctrlVal == CTRL_DATA_TX)
			{
				// Bin1 -> DST ID
				auto dBits = qpskDemodulate(active[1]);
				int destId = dBits.first * 2 + dBits.second;
				
				// Bin2 -> SRC ID
				auto sBits = qpskDemodulate(active[2]);
				int srcId  = sBits.first * 2 + sBits.second;
				
				// Find sender's allocation
				auto itsrc = allocation.find(srcId);
				if (itsrc == allocation.end())
				{
					std::cout << "Sender " << srcId << " has no allocation\n";
					clearFile(BS_RX_FILE);
					continue;
				}
				int stsrc = itsrc->second.first;   // Sender's start bin
				int cntsrc = itsrc->second.second; // Sender's number of bins

				// Find receiver's allocation
				auto itDst = allocation.find(destId);
				if (itDst == allocation.end())
				{
					std::cout << "Receiver " << destId << " has no allocation, Cannot re-encode data.\n";
					clearFile(BS_RX_FILE);
					continue;
				}
				int stDst = itDst->second.first;   // Receiver's start bin
				int cntDst = itDst->second.second; // Receiver's number of bins

				// Decode data according to Sender's allocated bins
				int payload = 0;
				for (int i = stsrc; i < stsrc + cntsrc; i++)
				{
					auto bits = qpskDemodulate(active[i]);
					int val = bits.first * 2 + bits.second;
					payload = (payload << 2) | val;
				}
				std::cout << "Data from user " << srcId << " to user " << destId
						  << " => decoded payload = " << payload << "\n";

				// Retransmitting according to Receiver's allocated bins
				std::vector<std::complex<double>> respFull(FFT_SIZE, {0,0});
				std::vector<std::complex<double>> activeResp(FREQ_BINS, {0,0});

				// Bin0 -> CTRL_DATA_TX
				activeResp[0] = qpskModulate(0,1);
				// Bin1 -> DST ID
				activeResp[1] = qpskModulate((destId >> 1) & 1, destId & 1);
				// Bin2 -> SRC ID
				activeResp[2] = qpskModulate((srcId >> 1) & 1, srcId & 1);

				//	Retransmit data
				int bitsSender = cntsrc * 2;
				int bitsReceiver = cntDst * 2;
				//	If the sender has more bits than the receiver can store, truncate
				int mask = (1 << bitsReceiver) - 1;
				int reencodedPayload = payload & mask;
				//  Re-encode the payload into the receiver's allocated bins
				for (int i = 0; i < cntDst; i++)
				{
					int shift = 2 * (cntDst - 1 - i);
					int sym = (reencodedPayload >> shift) & 0x3;  // 2 bits
					activeResp[stDst + i] = qpskModulate((sym >> 1) & 1, sym & 1);
				}

				// Upsampling
				for (int i = 0; i < FREQ_BINS; i++)
				{
					respFull[i * FREQ_BIN_SPACING] = activeResp[i];
				}

				// Construct time-domain signal and write to user rx-buffer
				auto newTime = ifft(respFull);
				addAWGN(newTime, NOISE_VARIANCE);
				std::string destRxFile = "rxbuffer_files/user" + std::to_string(destId) + "_rx_waveform.txt";
				writeWaveform(destRxFile, newTime);
				clearFile(BS_RX_FILE);
			}

            else if (ctrlVal == CTRL_DEALLOCATE)
			{
                // Deallocate
                std::pair<int,int> ub = qpskDemodulate(active[1]);
                int uid = ub.first*2 + ub.second;
                std::cout << "Deallocation request from user " << uid << "\n";
                deallocateBins(uid);
                std::cout << "Deallocated bins for user " << uid << "\n";

                // Response
                std::vector<std::complex<double>> respFull(FFT_SIZE, {0,0});
                std::vector<std::complex<double>> activeResp(FREQ_BINS, {0,0});
				
				// Bin0 -> CTRL_RESPONSE
                activeResp[0] = qpskModulate(1,0);
				// Bin1 -> User ID
                activeResp[1] = qpskModulate((uid>>1)&1, uid&1);
				
				// No Data in unused bins
                for(int i=2; i<FREQ_BINS; i++)
				{
                    activeResp[i] = qpskModulate(0,0);
                }

				// Upsampling to N = 64
                for(int i=0; i<FREQ_BINS; i++)
				{
                    respFull[i*FREQ_BIN_SPACING] = activeResp[i];
                }
				
				// Construct time-domain signal and write to user rx-buffer
                auto respTime = ifft(respFull);
                addAWGN(respTime, NOISE_VARIANCE);
                std::string userRx = "rxbuffer_files/user" + std::to_string(uid) + "_rx_waveform.txt";
                writeWaveform(userRx, respTime);
            }
            else
			{
                std::cout << "Unknown control code: " << ctrlVal << std::endl;
            }
            clearFile(BS_RX_FILE);
        }
        Sleep(1000);
    }
    return 0;
}
