/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        WatchdogService.h
// Purpose:     A simple watchdog that runs in a detached thread, waiting for new data to be received
//              from the robot.
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.24.24 5:35 PM MT: Comments added
//
// Copyright 2024 Andrew Gallagher
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Experimental class, designed to run periodically in the background and update the GUI.
// As it stands, this does not work due to the inability to access Qt GUI data in a 
// detached thread, but it's a good starting point to gain ideas from.
#ifndef WATCHDOGSERVICE_H
#define WATCHDOGSERVICE_H

#include <mutex>
#include <thread>
#include <unistd.h>
#include <iostream>

// Variables for periodically loading files from the buffer

class WatchdogService {
public:
    // Class constructor
    WatchdogService() {
        startWatchdog();
        std::cout << "Watchdog initialized." << std::endl;
    }

    // Function for the watchdog thread
    void bufferWatchdog() {
        // Sleep time (1 second)
        unsigned int microsecond = 1000000;

        // Spin forever, checking the number of buffers that are currently
        // in the queue
        while (true) {
            usleep(1 * microsecond);

            if ((bufferTally > currentBuffer) && !hasDataToUpdate) {
                std::cout << "Watchdog: New data identified." << std::endl;

                bool tryAgain = true;
                while(tryAgain) {
                    try {
                        if (!freeUpdate.try_lock()) {
                            tryAgain = true;
                            throw 100;
                        } else {
                            tryAgain = false;
                        }
                    } catch (int err) {
                        std::cout << "Error trying to load MapData, mutex is locked" << std::endl;
                    }
                }

                hasDataToUpdate = true;
                //loadBufferToMap();
            }
        }
    }

    // Create the watchdog thread
    void startWatchdog() {

        //watchdog.detach();
    }

    // Stop the watchdog thread
    void stopWatchdog() {

    }

    int bufferTally = 0;
    int currentBuffer = 0;
private:
    std::thread watchdog();
    std::mutex freeUpdate;
    bool hasDataToUpdate = false;
    int threadID = 0;
};

#endif // WATCHDOGSERVICE_H
