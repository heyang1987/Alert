/* Copyright (c) 2007 Johns Hopkins University.
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Razvan Musaloiu-E. <razvanm@cs.jhu.edu>
 * @author Chieh-Jan Mike Liang <cliang4@cs.jhu.edu>
 */

module DelugeP
{
    uses {
        interface Boot;
        interface Leds;
        interface DisseminationValue<DelugeCmd>;
        interface StdControl as DisseminationStdControl;
        interface ObjectTransfer;
        interface SplitControl as RadioSplitControl;
        interface NetProg;
        interface StorageMap[uint8_t volumeId];
        interface DelugeMetadata;
        interface DelugeVolumeManager;
        interface Resource;

        // Interface for NodeStatus Timer
        interface Timer<TMilli> as NodeStatusTimer;

        // Interfaces for Collection (NodeStatus)
        interface RootControl;
        interface StdControl as RoutingControl;
        interface Send as NodeStatusSender;

    }
    provides {
        event void storageReady();
        command void stop();
    }
}

implementation
{
    enum {
        S_IDLE,
        S_PUB,
        S_RECV
    };

    DelugeCmd lastCmd;
    uint8_t state = S_IDLE;

/*    message_t nodeStatusMsg;*/

    event void storageReady()
    {
        call RadioSplitControl.start();
    }

    event void Boot.booted()
    {
        lastCmd.uidhash = DELUGE_INVALID_UID;
    }

    event void RadioSplitControl.startDone(error_t error)
    {
        if (error == SUCCESS) {
            call DisseminationStdControl.start();

            // Start the Collection service (must be done after the radio has been started)
            call RoutingControl.start();

            // Only start the timer if this is a client mote (not BaseStation)
#ifndef DELUGE_BASESTATION

            // Start the NodeStatusTimer after the radio has been started
            // 1024 ticks per second
            // 1 second perodic
            call NodeStatusTimer.startPeriodic(1024);
#endif

            // Only set the BaseStation as the root node for Collection
#ifdef DELUGE_BASESTATION
            call RootControl.setRoot();
#endif

        }
    }

    // Send NodeStatus message (via Collection)
    event void NodeStatusTimer.fired()
    {
        size_t size;

        // Get payload (NodeStatus)
        NodeStatus *newNodeStatus = (NodeStatus *)call NodeStatusSender.getPayload(&nodeStatusMsg, sizeof(NodeStatus));

        // This will only be NULL if NodeStatus is larger than the maximum payload size (e.g. IRIS payload must be <= 20)
        if(newNodeStatus != NULL)
        {

            //call Leds.led2Toggle();

            //Fill in payload
            newNodeStatus->nodeId = TOS_NODE_ID;
            newNodeStatus->groupId = DELUGE_GROUP_ID;
            newNodeStatus->state = state;
            newNodeStatus->appUid = IDENT_UIDHASH;
            newNodeStatus->appTimeStamp = IDENT_TIMESTAMP;

            //Set all the values in the appName byte array to 0
            memset(newNodeStatus->appName, 0, sizeof(newNodeStatus->appName)); 

            //Check size and do not exceed appName size
            if (sizeof(IDENT_APPNAME) > sizeof(newNodeStatus->appName)) 
            {
                size = sizeof(newNodeStatus->appName);
            }
            else
            {
                size = sizeof(IDENT_APPNAME);
            }

            //Copy the array to appName
            memcpy(newNodeStatus->appName, IDENT_APPNAME, size);

            //Send the message
            if(call NodeStatusSender.send(&nodeStatusMsg, sizeof(NodeStatus)) != SUCCESS)
            {
                //Failed - do something with LEDS?
                //call Leds.led0Toggle();
            }

        }
    }

    command void stop()
    {
        call Resource.release();
        if (state == S_RECV) {
            //      printf("erase %d\n", lastCmd.imgNum);
            call DelugeVolumeManager.erase(lastCmd.imgNum);
        }
        call ObjectTransfer.stop();
        state = S_IDLE;
    }

    task void taskRequest()
    {
        signal Resource.granted();
    }

    void request()
    {
        if (call Resource.isOwner()) {
            post taskRequest();
        } else {
            call Resource.request();
        }
    }


    // Checks if the mote's TOS_NODE_ID is set in the nodeIds hash
    bool isNodeIdSet(uint32_t nodeIds) 
    {
        bool returnValue = FALSE;

        if((TOS_NODE_ID) <= 31 && ((((1 << TOS_NODE_ID) & nodeIds)) > 0))
        {
            returnValue = TRUE;
        }

        return returnValue;

    }

    event void DisseminationValue.changed()
    {
        const DelugeCmd *cmd = call DisseminationValue.get();
        //    printf("cmd: %d uidhash: 0x%lx imgNum: %d size: %u\n", cmd->type, cmd->uidhash, cmd->imgNum, cmd->size);
        switch (cmd->type) {
            case DELUGE_CMD_STOP:
                call stop();
                break;
            case DELUGE_CMD_ONLY_DISSEMINATE:
            case DELUGE_CMD_DISSEMINATE_AND_REPROGRAM:
                if (state == S_RECV) {
                    if (cmd->uidhash == lastCmd.uidhash) {
                        if (cmd->imgNum == lastCmd.imgNum) {
                            // Same uidhash, same imgNum, only cmd should be
                            // different.  That will be properly updated by the last
                            // statement from this function.
                            break;
                        }
                    }
                    call stop();
                }
                if (cmd->uidhash != IDENT_UIDHASH) {
                    call DelugeMetadata.read(cmd->imgNum);
                } else {
                    state = S_PUB;
                    request();
                }
                break;
            case DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_NODES:
                if(isNodeIdSet(cmd->nodeIds)) {
                    if (state == S_RECV) {
                        if (cmd->uidhash == lastCmd.uidhash) {
                            if (cmd->imgNum == lastCmd.imgNum) {
                                // Same uidhash, same imgNum, only cmd should be
                                // different.  That will be properly updated by the last
                                // statement from this function.
                                break;
                            }
                        }
                        call stop();
                    }
                    if (cmd->uidhash != IDENT_UIDHASH) {
                        call DelugeMetadata.read(cmd->imgNum);
                    } else {
                        state = S_PUB;
                        request();
                    }
                }
                break;
            case DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_GROUP:
                if(cmd->groupId == DELUGE_GROUP_ID) {
                    if (state == S_RECV) {
                        if (cmd->uidhash == lastCmd.uidhash) {
                            if (cmd->imgNum == lastCmd.imgNum) {
                                // Same uidhash, same imgNum, only cmd should be
                                // different.  That will be properly updated by the last
                                // statement from this function.
                                break;
                            }
                        }
                        call stop();
                    }
                    if (cmd->uidhash != IDENT_UIDHASH) {
                        call DelugeMetadata.read(cmd->imgNum);
                    } else {
                        state = S_PUB;
                        request();
                    }
                }
                break;
            case DELUGE_CMD_UPDATE_GROUP:
                if(isNodeIdSet(cmd->nodeIds)) {
                    if(cmd->groupId != DELUGE_GROUP_ID) {
                        call stop();

                        DELUGE_GROUP_ID = cmd->groupId;
                        call NetProg.reboot();
                    }
                }
                break;
        }
        lastCmd = *cmd;
        //    printf("lastCmd: %d uidhash: 0x%lx\n", lastCmd.type, lastCmd.uidhash);
    }

    event void ObjectTransfer.receiveDone(error_t error)
    {
        call Leds.set(LEDS_LED1 | LEDS_LED2);
        state = S_IDLE;

        if (error == SUCCESS) {
            switch (lastCmd.type) {
                case DELUGE_CMD_ONLY_DISSEMINATE:
                    state = S_PUB;
                    request();
                    break;
                case DELUGE_CMD_DISSEMINATE_AND_REPROGRAM:
                case DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_NODES:
                case DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_GROUP:
                    call NetProg.programImageAndReboot(call StorageMap.getPhysicalAddress[lastCmd.imgNum](0));
                    break;
            }
        } else {
            call DelugeVolumeManager.erase(lastCmd.imgNum);
        }
    }

    event void DelugeMetadata.readDone(uint8_t imgNum, DelugeIdent* ident, error_t error)
    {
        //    printf("readDone 0x%lx imgNum: %d size: %lu\n", lastCmd.uidhash, lastCmd.imgNum, lastCmd.size);
        if (ident->uidhash == lastCmd.uidhash) {
            if (lastCmd.type == DELUGE_CMD_DISSEMINATE_AND_REPROGRAM ||
                lastCmd.type == DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_NODES ||
                lastCmd.type == DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_GROUP) {
                call NetProg.programImageAndReboot(call StorageMap.getPhysicalAddress[imgNum](0));
            } else {
                // We already have the image so we'll go ahead and start publishing.
                state = S_PUB;
                request();
            }
        } else {
            state = S_RECV;
            request();
        }
    }

    event void Resource.granted()
    {
        switch (state) {
            case S_PUB:
                //      printf("start pub 0x%lx imgNum: %d size: %u\n", lastCmd.uidhash, lastCmd.imgNum, lastCmd.size);
                call ObjectTransfer.publish(lastCmd.uidhash, lastCmd.size, lastCmd.imgNum);
                break;
            case S_RECV:
                call ObjectTransfer.receive(lastCmd.uidhash, lastCmd.size, lastCmd.imgNum);
                break;
        }
    }
/*
    event void NodeStatusSender.sendDone(message_t *msg, error_t ok) { 
        //call Leds.led1Toggle();
    }
*/
    event void DelugeVolumeManager.eraseDone(uint8_t imgNum) {}
    event void RadioSplitControl.stopDone(error_t error) {}
    default async void command Leds.set(uint8_t val) {}
}
