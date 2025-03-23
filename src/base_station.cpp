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
            // FFT
            auto fullFreq = fft(rxWave);
            // Extract 8 active subcarriers
            std::vector<std::complex<double>> active(FREQ_BINS);
            for (int i = 0; i < FREQ_BINS; i++)
			{
                active[i] = fullFreq[i * ACTIVE_BIN_SPACING];
            }
            // Decode control code from bin 0
            std::pair<int,int> ctrlBits = qpskDemodulate(active[0]);
            int ctrlVal = ctrlBits.first * 2 + ctrlBits.second;

            if (ctrlVal == CTRL_ACCESS_REQUEST)
			{
                // Access request
                std::pair<int,int> userBits = qpskDemodulate(active[1]);
                int userId = userBits.first * 2 + userBits.second;

                std::pair<int,int> reqBits = qpskDemodulate(active[2]);
                int requested = reqBits.first * 2 + reqBits.second;

                std::cout << "Access request from user " << userId << " requesting " << requested << " bins.\n";
                
                std::pair<int,int> allocRes = allocateBins(requested);
                int st = allocRes.first;
                int cnt = allocRes.second;

                if (st < 0 || cnt == 0)
				{
                    std::cout << "Could not allocate bins for user " << userId << std::endl;
                }
				else
				{
                    allocation[userId] = std::make_pair(st, cnt);
                    std::cout << "Allocated " << cnt << " bins to user " << userId 
                              << " starting at active bin " << st << std::endl;
                }

                // Build response
                std::vector<std::complex<double>> respFull(FFT_SIZE, {0,0});
                std::vector<std::complex<double>> activeResp(FREQ_BINS, {0,0});
                // bin0 => response(10)
                activeResp[0] = qpskModulate(1,0);
                // bin1 => userId
                activeResp[1] = qpskModulate((userId>>1)&1, userId&1);
                // bin2 => allocated count
                activeResp[2] = qpskModulate((cnt>>1)&1, cnt&1);
                // encode start in 4 bits => bin3, bin4
                int sVal = st & 0xF;
                int hi2 = (sVal >> 2) & 0x3;
                int lo2 = sVal & 0x3;
                activeResp[3] = qpskModulate((hi2>>1)&1, hi2 & 1);
                activeResp[4] = qpskModulate((lo2>>1)&1, lo2 & 1);
				
                for(int i=5; i<FREQ_BINS; i++)
                    activeResp[i] = qpskModulate(0,0);
				
                for(int i=0; i<FREQ_BINS; i++)
                    respFull[i * ACTIVE_BIN_SPACING] = activeResp[i];
				
                auto respTime = ifft(respFull);
                addAWGN(respTime, NOISE_VARIANCE);
                std::string userRx = "rxbuffer_files/user" + std::to_string(userId) + "_rx_waveform.txt";
                writeWaveform(userRx, respTime);
            }
            else if (ctrlVal == CTRL_DATA_TX)
			{
				// 1) Decode bin1 => destination user, bin2 => sender user
				auto dBits = qpskDemodulate(active[1]);
				int destId = dBits.first * 2 + dBits.second;
				
				auto sBits = qpskDemodulate(active[2]);
				int sndId  = sBits.first * 2 + sBits.second;
				
				// 2) Look up the sender's allocation
				auto itSnd = allocation.find(sndId);
				if (itSnd == allocation.end())
				{
					std::cout << "Sender " << sndId << " has no allocation\n";
					clearFile(BS_RX_FILE);
					continue;
				}
				int stSnd = itSnd->second.first;   // sender's start bin
				int cntSnd = itSnd->second.second; // sender's number of bins

				// 3) Look up the receiver's allocation
				auto itDst = allocation.find(destId);
				if (itDst == allocation.end())
				{
					std::cout << "Receiver " << destId << " has no allocation, Cannot re-encode data.\n";
					// Optionally, you could auto-allocate bins for the receiver or just drop the data.
					clearFile(BS_RX_FILE);
					continue;
				}
				int stDst = itDst->second.first;   // receiver's start bin
				int cntDst = itDst->second.second; // receiver's number of bins

				// 4) Decode the payload from the sender's allocated bins
				int payload = 0;
				for (int i = stSnd; i < stSnd + cntSnd; i++)
				{
					auto bits = qpskDemodulate(active[i]);
					int val = bits.first * 2 + bits.second;  // each QPSK symbol is 2 bits
					payload = (payload << 2) | val;          // accumulate into 'payload'
				}
				std::cout << "Data from user " << sndId << " to user " << destId
						  << " => decoded payload = " << payload << "\n";

				// 5) Build a brand-new frequency vector for re-encoding into the receiver's bins
				std::vector<std::complex<double>> newFull(FFT_SIZE, {0,0});
				std::vector<std::complex<double>> newActive(FREQ_BINS, {0,0});

				// (a) Bin0 => control code for data (bits=01)
				newActive[0] = qpskModulate(0,1);

				// (b) Bin1 => destination user
				newActive[1] = qpskModulate((destId >> 1) & 1, destId & 1);

				// (c) Bin2 => sender user
				newActive[2] = qpskModulate((sndId >> 1) & 1, sndId & 1);

				// (d) Re-encode the payload into the receiver's allocated bins
				//     Each bin can carry 2 bits, so cntDst bins => 2*cntDst bits.
				//     If the sender had more bits than the receiver can store, we may need to truncate.
				//     If the receiver can store more bits, we can just keep the extra bins as 0.
				int bitsSender = cntSnd * 2;  // how many bits came from the sender
				int bitsReceiver = cntDst * 2;
				// For simplicity, we just store the lower 'bitsReceiver' bits of 'payload'
				// (Truncates if the receiver has fewer bins than the sender)
				int mask = (1 << bitsReceiver) - 1;  // e.g. if bitsReceiver=6 => mask=0x3F
				int reencodedPayload = payload & mask;

				// Place these bits in [stDst .. stDst+cntDst-1]
				for (int i = 0; i < cntDst; i++)
				{
					// shift by 2*(cntDst -1 - i)
					int shift = 2 * (cntDst - 1 - i);
					int sym = (reencodedPayload >> shift) & 0x3;  // 2 bits
					newActive[stDst + i] = qpskModulate((sym >> 1) & 1, sym & 1);
				}

				// (e) Place newActive[] into newFull[]
				for (int i = 0; i < FREQ_BINS; i++)
				{
					newFull[i * ACTIVE_BIN_SPACING] = newActive[i];
				}

				// 6) IFFT, add noise, write to the destination user's RX file
				auto newTime = ifft(newFull);
				addAWGN(newTime, NOISE_VARIANCE);

				std::string destRxFile = "rxbuffer_files/user" + std::to_string(destId) + "_rx_waveform.txt";
				writeWaveform(destRxFile, newTime);

				// 7) Clear the base station RX file after processing
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

                // optional response
                std::vector<std::complex<double>> respFull(FFT_SIZE, {0,0});
                std::vector<std::complex<double>> activeResp(FREQ_BINS, {0,0});
                activeResp[0] = qpskModulate(1,0); // response
                activeResp[1] = qpskModulate((uid>>1)&1, uid&1);
                activeResp[2] = qpskModulate(0,0);
                for(int i=3; i<FREQ_BINS; i++)
				{
                    activeResp[i] = qpskModulate(0,0);
                }

                for(int i=0; i<FREQ_BINS; i++)
				{
                    respFull[i*ACTIVE_BIN_SPACING] = activeResp[i];
                }
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
