#pragma once

#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Object.h>
#include <EASTL/queue.h>
#include <winbase.h>

using namespace Urho3D;


class GymClient : public Object
{
	URHO3D_OBJECT(GymClient, Object);

public:
	/// Construct.
	explicit GymClient(Context* context) : Object(context)
	{
	}

    enum CommandType {
        CommandType_Action = 0,
        CommandType_Reset,
        CommandType_None
    };



	void Connect()
	{
        if (IsConnected())
        {
            //close
        }



        pipeHandle = CreateFile(
            "\\\\.\\pipe\\GYMPIPE",
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            0,
            NULL
        );

		if (pipeHandle != INVALID_HANDLE_VALUE)
		{
            //send Gym Specs
            Send32(actionSets[0].size());
            Send32(states[0].size());
		}
		else
			URHO3D_LOGINFO("Unable To Connect to Pipe");
	}

    void SetGYMSpec(int nActions, int nStates)
    {
        ends.resize(numGYMS);
        actionSets.resize(numGYMS);
        states.resize(numGYMS);
        rewards.resize(numGYMS);
        for (int g = 0; g < numGYMS; g++)
        {
            actionSets[g].resize(nActions);
            states[g].resize(nStates);
            ends[g] = 0;
        }
    }


    //Read numBytes, block if needed.
    void ReadBytes(uint8_t* data, int numBytes)
    {

        ReadFile(pipeHandle, data, numBytes, NULL, NULL);

    }

    uint32_t Read32()
    {
        uint8_t dataBuff[4];

        ReadBytes(dataBuff, 4);

        return ((dataBuff[0]) & 0x000000FF)
            | ((dataBuff[1] << 8) & 0x0000FF00)
            | ((dataBuff[2] << 16) & 0x00FF0000)
            | ((dataBuff[3] << 24) & 0xFF000000);
    }

    float ReadFloat()
    {
        uint8_t dataBuff[4];

        ReadBytes(dataBuff, 4);

        uint32_t dataAsInt = ((dataBuff[0]) & 0x000000FF)
            | ((dataBuff[1] << 8) & 0x0000FF00)
            | ((dataBuff[2] << 16) & 0x00FF0000)
            | ((dataBuff[3] << 24) & 0xFF000000);

 
        return *(float*)(&dataAsInt);
    }

    void Send32(int val)
    {
        uint8_t dataBuff[4];
        dataBuff[0] = (val) & 0x000000FF; //lsb
        dataBuff[1] = (val) >> 8;
        dataBuff[2] = (val) >> 16;
        dataBuff[3] = (val) >> 24;        //msb



        WriteFile(pipeHandle, dataBuff, 4, NULL, NULL);
    }


    void SendFloat(float val)
    {
        uint32_t dataAsInt = *(uint32_t*)(&val);
        uint8_t dataBuff[4];
        dataBuff[0] = (dataAsInt) & 0x000000FF; //lsb
        dataBuff[1] = (dataAsInt) >> 8;
        dataBuff[2] = (dataAsInt) >> 16;
        dataBuff[3] = (dataAsInt) >> 24;     //msb


        WriteFile(pipeHandle, dataBuff, 4, NULL, NULL);
    }


    //Read the next command
    CommandType GetCommand()
    {
        if (IsConnected())
        {
            lastCommand = (CommandType)Read32();
            if (lastCommand == CommandType_Action)
            {
                doRender = bool(Read32());

                for (int g = 0; g < numGYMS; g++)
                {
                    for (int v = 0; v < actionSets[g].size(); v++)
                    {
                        float val = ReadFloat();
                        actionSets[g][v] = val;
                    }
                }
            }
            else if (lastCommand == CommandType_Reset)
            {
                
                numGYMS = Read32();



                ends.resize(numGYMS);
                actionSets.resize(numGYMS);
                states.resize(numGYMS);
                rewards.resize(numGYMS);

                resetPending = true;
            }

            return lastCommand;
        }
        return CommandType_None;
    }

    void SendResponse()
    {
        if (IsConnected())
        {
            if (lastCommand == CommandType_Action)
            {
                for (int g = 0; g < numGYMS; g++)
                {
                    //state
                    for (int v = 0; v < states[g].size(); v++)
                    {
                        SendFloat(states[g][v]);
                    }

                    //reward
                    SendFloat(rewards[g]);

                    //Ends
                    Send32(ends[g]);
                }

            }
            else if(lastCommand == CommandType_Reset)
            {
                int i = 0;
                for (int g = 0; g < numGYMS; g++)
                {
                    //state
                    for (int v = 0; v < states[g].size(); v++)
                    {
                        SendFloat(states[g][v]);
                        i++;
                    }
                }

                Send32(99999);
                
            }
        }
    }

    bool IsConnected() { return pipeHandle != INVALID_HANDLE_VALUE; }

	void Disconnect()
	{
        DeleteFile("\\\\.\\pipe\\GYMPIPE");
        pipeHandle = nullptr;
	}

    CommandType lastCommand = CommandType_None;
    bool resetPending = false;

    ea::vector<ea::vector<float>> actionSets;
    ea::vector<ea::vector<float>> states;
    ea::vector<float> rewards;
    ea::vector<int> ends;
    int numGYMS = 200;
    bool doRender = false;


    int readBufferLoc = 0;
    HANDLE pipeHandle = INVALID_HANDLE_VALUE;
};

