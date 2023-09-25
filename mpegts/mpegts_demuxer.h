#pragma once

// Prefixes used
// m class member
// p pointer (*)
// r reference (&)
// l local scope

#include "ts_packet.h"
#include "simple_buffer.h"

#include <cstdint>
#include <memory>
#include <map>
#include <set>
#include <functional>
#include <mutex>

class MpegTsDemuxer {
public:
    MpegTsDemuxer();

    virtual ~MpegTsDemuxer();

    uint8_t decode(SimpleBuffer &rIn);

    std::function<void(EsFrame *pEs)> esOutCallback = nullptr;
    std::function<void(uint64_t lPcr)> pcrOutCallback = nullptr;

    typedef struct {
        PMTHeader mPmtHeader;
        bool mPmtIsValid = false;
        int mPcrId;
        int mProgNo;
        SimpleBuffer mPmtBuf;
        int mCurrentSection = 0;
        std::set<int> epids;
    } pmtInfoT;

    // stream, pid
    //    std::map<uint8_t, int> mStreamPidMap;

    // pmtPid, pmtInfo
    std::map<int, pmtInfoT> mPmtMap;

    // PAT
    PATHeader mPatHeader;
    bool mPatIsValid = false;

private:
    // pid, Elementary data frame
    std::map<int, std::shared_ptr<EsFrame>> mEsFrames;
    SimpleBuffer mRestData;
    SimpleBuffer mPatBuf;
};
