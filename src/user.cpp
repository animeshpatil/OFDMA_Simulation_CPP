#define _USE_MATH_DEFINES
#include "signal_processing.h"
#include <queue>

using namespace std;

static int allocCount = 0;
static int allocStart = -1;

int main(int argc, char* argv[])
{
    if(argc<2)
	{
        cerr<<"Usage: user <user_id>"<<endl;
        return 1;
    }
    int userId=atoi(argv[1]);
    if(userId<0 || userId>3)
	{
        cerr<<"User id must be 0-3"<<endl;
        return 1;
    }
    string rxFile="rxbuffer_files/user"+to_string(userId)+"_rx_waveform.txt";
    cout<<"User simulation started. user id="<<userId<<"\n";

	// Message Buffer
    queue<string> msgQueue;

    while(true)
	{
        auto rxWave = readWaveform(rxFile);
        if(rxWave.size()==FFT_SIZE)
		{
            auto fullFreq = fft(rxWave);	// N = 64 fft
            vector<std::complex<double>> active(FREQ_BINS);		// N = 8 bins
			
            for(int i=0; i<FREQ_BINS; i++)
			{
                active[i] = fullFreq[i*FREQ_BIN_SPACING];
            }
			
			// Extract CTRL Code
            std::pair<int,int> ctrlBits = qpskDemodulate(active[0]);
            int ctrlVal = ctrlBits.first*2 + ctrlBits.second;
			
            ostringstream oss;
			
            if(ctrlVal == CTRL_RESPONSE)
			{

                auto cB = qpskDemodulate(active[2]);
                int cnt = cB.first*2 + cB.second;

                auto hiPair = qpskDemodulate(active[3]);
                int hiVal = hiPair.first*2 + hiPair.second;
                auto loPair = qpskDemodulate(active[4]);
                int loVal = loPair.first*2 + loPair.second;
                int s = (hiVal<<2) | loVal;
                
                allocCount = cnt;
                allocStart = s;
                oss<<"Response: allocated "<<cnt<<" bins starting at active bin "<<s;
                msgQueue.push(oss.str());
            }
            else if(ctrlVal == CTRL_DATA_TX)
			{
				// decode data: active[1] => destination, active[2] => sender
				// auto db = qpskDemodulate(active[1]);
				// int destId = db.first * 2 + db.second;
				auto sb = qpskDemodulate(active[2]);
				int sndId = sb.first * 2 + sb.second;

				// Now, decode the payload only from the receiver's allocated bins.
				// allocStart and allocCount should have been updated from a previous response.
				int payload = 0;
				for (int i = allocStart; i < allocStart + allocCount; i++)
				{
					auto bits = qpskDemodulate(active[i]);
					int val = bits.first * 2 + bits.second;  // each symbol gives 2 bits
					payload = (payload << 2) | val;
				}
				oss << "Data from user " << sndId << ", payload=" << payload;
				msgQueue.push(oss.str());
			}

            else if(ctrlVal == CTRL_DEALLOCATE)
			{
                auto ub = qpskDemodulate(active[1]);
                int uid = ub.first*2 + ub.second;
                oss<<"Deallocation command for user "<<uid;
                msgQueue.push(oss.str());
            }
            else
			{
                oss<<"Unknown control code: "<<ctrlVal;
                msgQueue.push(oss.str());
            }
            clearFile(rxFile);
        }

        // menu
        cout<<"\nCommands: req, send, dealloc, read, exit: ";
        string cmd;
        cin>>cmd;
        if(cmd=="exit") break;

        if(cmd=="req")
		{
            int n;
            cout<<"Enter bins requested(1-3): ";
            cin>>n;
            if(n<1)
				n=1;
			if(n>3)
				n=3;
            vector<complex<double>> fullFreq(FFT_SIZE, {0,0});
            vector<complex<double>> activeVec(FREQ_BINS, {0,0});
            // active[0] => 00 => Access Request
            activeVec[0] = qpskModulate(0,0);
            // active[1] => userId
            activeVec[1] = qpskModulate((userId>>1)&1, userId&1);
            // active[2] => requested
            activeVec[2] = qpskModulate((n>>1)&1, n&1);
            for(int i=3; i<FREQ_BINS; i++)
			{
                activeVec[i] = qpskModulate(0,0);
            }
            for(int i=0; i<FREQ_BINS; i++)
			{
                fullFreq[i*FREQ_BIN_SPACING] = activeVec[i];
            }
            auto timeSig = ifft(fullFreq);
            addAWGN(timeSig, NOISE_VARIANCE);
            writeWaveform(BS_RX_FILE, timeSig);
            cout<<"Access request sent.\n";
        }
        else if(cmd=="send")
		{
            if(allocCount<=0)
			{
                cout<<"No bins allocated.\n";
                continue;
            }
            int maxBits = 2*allocCount;
            int maxVal = (1<<maxBits)-1;
            cout<<"You have "<< allocCount <<" bins => can send up to "<< maxBits <<" bits => max int="<<maxVal<<"\n";
            int dst, pay;
            cout << "Destination user(0-3)? ";
            cin >> dst;
            cout << "Payload(0.."<<maxVal<<")? ";
            cin>> pay;
            if(pay<0) pay=0;
            if(pay>maxVal) pay=maxVal;
            
            vector<complex<double>> fullFreq(FFT_SIZE, {0,0});
            vector<complex<double>> activeVec(FREQ_BINS, {0,0});
			
			for(int i=0; i<FREQ_BINS; i++)
			{
                activeVec[i] = qpskModulate(0,0);
            }
			
            // active[0] => 01 => Data Tx
            activeVec[0] = qpskModulate(0,1);
            // active[1] => dest
            activeVec[1] = qpskModulate((dst>>1)&1, dst&1);
            // active[2] => sender
            activeVec[2] = qpskModulate((userId>>1)&1, userId&1);
            // place payload in allocated bins
            for(int i=0; i<allocCount; i++)
			{
                int shift = 2*(allocCount -1 - i);
                int bits = (pay>>shift) & 0x3;
                activeVec[allocStart + i] = qpskModulate((bits>>1)&1, bits&1);
			}
			
            for(int i=0; i<FREQ_BINS; i++)
			{
                fullFreq[i*FREQ_BIN_SPACING] = activeVec[i];
            }
            auto timeSig = ifft(fullFreq);
            addAWGN(timeSig, NOISE_VARIANCE);
            writeWaveform(BS_RX_FILE, timeSig);
            cout<<"Data transmission sent.\n";
        }
        else if(cmd=="dealloc")
		{
            // send a deallocate command => 11
            vector<complex<double>> fullFreq(FFT_SIZE, {0,0});
            vector<complex<double>> activeVec(FREQ_BINS, {0,0});
            activeVec[0] = qpskModulate(1,1); // 11 => Dealloc
            activeVec[1] = qpskModulate((userId>>1)&1, userId&1);
            for(int i=2; i<FREQ_BINS; i++)
			{
                activeVec[i] = qpskModulate(0,0);
            }
            for(int i=0; i<FREQ_BINS; i++)
			{
                fullFreq[i*FREQ_BIN_SPACING] = activeVec[i];
            }
            auto timeSig = ifft(fullFreq);
            addAWGN(timeSig, NOISE_VARIANCE);
            writeWaveform(BS_RX_FILE, timeSig);
            cout<<"Deallocation command sent.\n";
        }
        else if(cmd=="read")
		{
            if(!msgQueue.empty())
			{
                cout<<"Message: "<<msgQueue.front()<<"\n";
                msgQueue.pop();
            }
			else
			{
                cout<<"No messages in queue.\n";
            }
        }
        Sleep(500);
    }
    return 0;
}
