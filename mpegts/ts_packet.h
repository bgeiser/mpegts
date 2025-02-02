#pragma once

// Prefixes used
// m class member
// p pointer (*)
// r reference (&)
// l local scope

#include <cstdint>
#include <memory>
#include <vector>
#include <string>

class SimpleBuffer;

class EsFrame {
public:
    EsFrame();

    EsFrame(uint8_t lSt, uint16_t pmtPid);

    virtual ~EsFrame() {};

    bool empty();

    void reset();

    std::shared_ptr<SimpleBuffer> mData;
    uint64_t mPts;
    uint64_t mDts;
    uint64_t mPcr;
    uint8_t mRandomAccess;
    uint8_t mStreamType;
    uint8_t mStreamId;
    uint16_t mPid;
    uint16_t mExpectedPesPacketLength;
    uint16_t mExpectedPayloadLength;
    bool mCompleted;
    bool mBroken;

    uint16_t mPmtPid;
};

class TsHeader {
public:
    TsHeader();

    virtual ~TsHeader();

    void encode(SimpleBuffer &rSb);

    void decode(SimpleBuffer &rSb);

    uint8_t mSyncByte;                      // 8 bits
    uint8_t mTransportErrorIndicator;      // 1 bit
    uint8_t mPayloadUnitStartIndicator;   // 1 bit
    uint8_t mTransportPriority;             // 1 bit
    uint16_t mPid;                           // 13 bits
    uint8_t mTransportScramblingControl;   // 2 bits
    uint8_t mAdaptationFieldControl;       // 2 bits
    uint8_t mContinuityCounter;             // 4 bits
};

class PATHeader {
public:
    PATHeader();

    virtual ~PATHeader();

    void encode(SimpleBuffer &rSb);

    void decode(SimpleBuffer &rSb);

    uint16_t getSectionLength(SimpleBuffer &rSb);

    void print();

    uint8_t mTableId;                       // 8 bits
    uint8_t mSectionSyntaxIndicator;       // 1 bit
    uint8_t mB0;                             // 1 bit
    uint8_t mReserved0;                      // 2 bits
    uint16_t mSectionLength;                // 12 bits
    uint16_t mTransportStreamId;           // 16 bits
    uint8_t mReserved1;                      // 2 bits
    uint8_t mVersionNumber;                 // 5 bits
    uint8_t mCurrentNextIndicator;         // 1 bit
    uint8_t mSectionNumber;                 // 8 bits
    uint8_t mLastSectionNumber;            // 8 bits
};

class PMTElementInfo {
public:
    PMTElementInfo();

    PMTElementInfo(uint8_t lSt, uint16_t lPid);

    virtual ~PMTElementInfo();

    void encode(SimpleBuffer &rSb);

    void decode(SimpleBuffer &rSb);

    uint16_t size();

    void print();

    uint8_t mStreamType;                    // 8 bits
    uint8_t mReserved0;                      // 3 bits
    uint16_t mElementaryPid;                // 13 bits
    uint8_t mReserved1;                      // 4 bits
    uint16_t mEsInfoLength;                // 12 bits
    std::string mEsInfo;
};

class PMTHeader {
public:
    PMTHeader();

    virtual ~PMTHeader();

    void encode(SimpleBuffer &rSb);

    void decode(SimpleBuffer &rSb);

    uint16_t getSectionLength(SimpleBuffer &rSb);

    uint16_t size();

    void print();

    uint8_t mTableId;                       // 8 bits
    uint8_t mSectionSyntaxIndicator;       // 1 bit
    uint8_t mB0;                             // 1 bit
    uint8_t mReserved0;                      // 2 bits
    uint16_t mSectionLength;                // 12 bits
    uint16_t mProgramNumber;                // 16 bits
    uint8_t mReserved1;                      // 2 bits
    uint8_t mVersionNumber;                 // 5 bits
    uint8_t mCurrentNextIndicator;         // 1 bit
    uint8_t mSectionNumber;                 // 8 bits
    uint8_t mLastSectionNumber;            // 8 bits
    uint8_t mReserved2;                      // 3 bits
    uint16_t mPcrPid;                       // 13 bits
    uint8_t mReserved3;                      // 4 bits
    uint16_t mProgramInfoLength;           // 12 bits
    std::vector<std::shared_ptr<PMTElementInfo>> mInfos;
};

class AdaptationFieldHeader {
public:
    AdaptationFieldHeader();

    virtual ~AdaptationFieldHeader();

    void encode(SimpleBuffer &rSb);

    void decode(SimpleBuffer &rSb);

    uint8_t mAdaptationFieldLength;                // 8 bits
    uint8_t mAdaptationFieldExtensionFlag;        // 1 bit
    uint8_t mTransportPrivateDataFlag;            // 1 bit
    uint8_t mSplicingPointFlag;                    // 1 bit
    uint8_t mOpcrFlag;                              // 1 bit
    uint8_t mPcrFlag;                               // 1 bit
    uint8_t mElementaryStreamPriorityIndicator;   // 1 bit
    uint8_t mRandomAccessIndicator;                // 1 bit
    uint8_t mDiscontinuityIndicator;                // 1 bit
};

class PESHeader {
public:
    PESHeader();

    virtual ~PESHeader();

    void encode(SimpleBuffer &rSb);

    void decode(SimpleBuffer &rSb);

    uint32_t mPacketStartCode;             // 24 bits
    uint8_t mStreamId;                      // 8 bits
    uint16_t mPesPacketLength;             // 16 bits
    uint8_t mOriginalOrCopy;               // 1 bit
    uint8_t mCopyright;                      // 1 bit
    uint8_t mDataAlignmentIndicator;       // 1 bit
    uint8_t mPesPriority;                   // 1 bit
    uint8_t mPesScramblingControl;         // 2 bits
    uint8_t mMarkerBits;                    // 2 bits
    uint8_t mPesExtFlag;                   // 1 bit
    uint8_t mPesCrcFlag;                   // 1 bit
    uint8_t mAddCopyInfoFlag;             // 1 bit
    uint8_t mDsmTrickModeFlag;            // 1 bit
    uint8_t mEsRateFlag;                   // 1 bit
    uint8_t mEscrFlag;                      // 1 bit
    uint8_t mPtsDtsFlags;                  // 2 bits
    uint8_t mHeaderDataLength;             // 8 bits
};
