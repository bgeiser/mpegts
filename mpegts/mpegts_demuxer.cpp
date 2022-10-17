#include "mpegts_demuxer.h"
#include "common.h"
#include <iostream>

MpegTsDemuxer::MpegTsDemuxer() {
}

MpegTsDemuxer::~MpegTsDemuxer() {
}

uint8_t MpegTsDemuxer::decode(SimpleBuffer &rIn) {
    if (mRestData.size()) {
        rIn.prepend(mRestData.data(),mRestData.size());
        mRestData.clear();
    }
    while ((rIn.size() - rIn.pos()) >= 188 ) {
        int lPos = rIn.pos();
        TsHeader lTsHeader;
        lTsHeader.decode(rIn);

        // found pat & get pmt pid
        if (!mPatIsValid && lTsHeader.mPid == 0) {
            if (lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mAdaptionOnly ||
                lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadAdaptionBoth) {
                AdaptationFieldHeader lAdaptionField;
                lAdaptionField.decode(rIn);
                rIn.skip(lAdaptionField.mAdaptationFieldLength > 0 ? (lAdaptionField.mAdaptationFieldLength - 1) : 0);
            }
            if (lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadOnly ||
                lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadAdaptionBoth) {
                if (lTsHeader.mPayloadUnitStartIndicator == 0x01)
                    {
                        uint8_t lPointField = rIn.read1Byte();
                        mPatBuf.clear();
                        mPatBuf.append(rIn.data()+rIn.pos(), rIn.size() - rIn.pos());
                    }
                else if(mPatBuf.size())
                    mPatBuf.append(rIn.data()+rIn.pos(), rIn.size() - rIn.pos());
                if (mPatBuf.size() >= mPatHeader.getSectionLength(mPatBuf)) {
                    mPatHeader.decode(mPatBuf);
                    auto start = rIn.pos();
                    while(mPatBuf.pos() < start + mPatHeader.mSectionLength - 8) {
                        auto progNo = mPatBuf.read2Bytes();
                        auto pmtPid = mPatBuf.read2Bytes() & 0x1fff;
                        mPmtMap[pmtPid] = { {}, false, -1, progNo, {} };
                        // std::cout<<"Prog "<<progNo<<std::endl;
                        // std::cout<<"PMT PID "<<pmtPid<<std::endl;
                    }
                    mPatBuf.clear();
                    mPatIsValid = true;
#ifdef TSDEBUG
                    mPatHeader.print();
#endif
                }
            }
        }

        // found pmt
        if (mPmtMap.find(lTsHeader.mPid) != mPmtMap.end()) {
            auto &pmtInfo = mPmtMap.at(lTsHeader.mPid);
            if (!pmtInfo.mPmtIsValid) {
                if (lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mAdaptionOnly ||
                    lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadAdaptionBoth) {
                    AdaptationFieldHeader lAdaptionField;
                    lAdaptionField.decode(rIn);
                    rIn.skip(lAdaptionField.mAdaptationFieldLength > 0 ? (lAdaptionField.mAdaptationFieldLength - 1) : 0);
                }
                if (lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadOnly ||
                    lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadAdaptionBoth) {
                    if (lTsHeader.mPayloadUnitStartIndicator == 0x01)
                        {
                            uint8_t lPointField = rIn.read1Byte();
                            pmtInfo.mPmtBuf.clear();
                            pmtInfo.mPmtBuf.append(rIn.data()+rIn.pos(), rIn.size() - rIn.pos());
                        }
                    else if(pmtInfo.mPmtBuf.size())
                        pmtInfo.mPmtBuf.append(rIn.data()+rIn.pos(), rIn.size() - rIn.pos());
                    if (pmtInfo.mPmtBuf.size() >= pmtInfo.mPmtHeader.getSectionLength(pmtInfo.mPmtBuf)) {
                        pmtInfo.mPmtHeader.decode(pmtInfo.mPmtBuf);
                        pmtInfo.mPmtBuf.clear();
                        pmtInfo.mPcrId = pmtInfo.mPmtHeader.mPcrPid;
                        if(pmtInfo.mPmtHeader.mSectionNumber == pmtInfo.mCurrentSection)
                            {
                                for (size_t lI = 0; lI < pmtInfo.mPmtHeader.mInfos.size(); lI++) {
                                    auto &info = pmtInfo.mPmtHeader.mInfos[lI];
                                    if(mEsFrames.find(info->mElementaryPid) == mEsFrames.end()) {
                                        mEsFrames[info->mElementaryPid] = std::shared_ptr<EsFrame>(new EsFrame(info->mStreamType));
                                        //                                mStreamPidMap[info->mStreamType] = info->mElementaryPid;
                                    }
                                }
                                pmtInfo.mCurrentSection++;
                            }
                        if(pmtInfo.mCurrentSection > pmtInfo.mPmtHeader.mLastSectionNumber)
                            pmtInfo.mPmtIsValid = true;
#ifdef TSDEBUG
                        pmtInfo.mPmtHeader.print();
#endif
                    }
                }
            }
        }

        if (mEsFrames.find(lTsHeader.mPid) != mEsFrames.end()) {
            // uint8_t lPcrFlag = 0;
            uint64_t lPcr = 0;
            uint8_t lRandomAccessIndicator = 0;
            if (lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mAdaptionOnly ||
                lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadAdaptionBoth) {
                AdaptationFieldHeader lAdaptionField;
                lAdaptionField.decode(rIn);
                lRandomAccessIndicator = lAdaptionField.mRandomAccessIndicator;
                int lAdaptFieldLength = lAdaptionField.mAdaptationFieldLength;
                if (lAdaptionField.mPcrFlag == 1) {
                    lPcr = readPcr(rIn);

                    if (pcrOutCallback) {
                        pcrOutCallback(lPcr);
                    }

                    // just adjust buffer pos
                    lAdaptFieldLength -= 6;
                }
                rIn.skip(lAdaptFieldLength > 0 ? (lAdaptFieldLength - 1) : 0);
            }

            if (lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadOnly ||
                lTsHeader.mAdaptationFieldControl == MpegTsAdaptationFieldType::mPayloadAdaptionBoth) {
                PESHeader lPesHeader;
                if (lTsHeader.mPayloadUnitStartIndicator == 0x01) {

                    mEsFrames[lTsHeader.mPid]->mRandomAccess = lRandomAccessIndicator;

                    if (mEsFrames[lTsHeader.mPid]->mCompleted) {
                        mEsFrames[lTsHeader.mPid]->reset();
                    } else if (mEsFrames[lTsHeader.mPid]->mData->size() && !mEsFrames[lTsHeader.mPid]->mCompleted) {
                        //Its a broken frame deliver that as broken
                        if (esOutCallback) {
                            EsFrame *lEsFrame = mEsFrames[lTsHeader.mPid].get();
                            lEsFrame -> mBroken = true;
                            lEsFrame -> mPid = lTsHeader.mPid;
                            esOutCallback(lEsFrame);
                        }

                        mEsFrames[lTsHeader.mPid]->reset();
                    }

                    lPesHeader.decode(rIn);
                    mEsFrames[lTsHeader.mPid]->mStreamId = lPesHeader.mStreamId;
                    mEsFrames[lTsHeader.mPid]->mExpectedPesPacketLength = lPesHeader.mPesPacketLength;
                    if (lPesHeader.mPtsDtsFlags == 0x02) {
                        mEsFrames[lTsHeader.mPid]->mPts = mEsFrames[lTsHeader.mPid]->mDts = readPts(rIn);
                    } else if (lPesHeader.mPtsDtsFlags == 0x03) {
                        mEsFrames[lTsHeader.mPid]->mPts = readPts(rIn);
                        mEsFrames[lTsHeader.mPid]->mDts = readPts(rIn);
                    }
                    if (lPesHeader.mPesPacketLength != 0) {

                        int payloadLength = lPesHeader.mPesPacketLength - 3 -
                            lPesHeader.mHeaderDataLength;

                        mEsFrames[lTsHeader.mPid]->mExpectedPayloadLength = payloadLength;

                        if (payloadLength + rIn.pos() > 188 || payloadLength < 0) {
                            mEsFrames[lTsHeader.mPid]->mData->append(rIn.data() + rIn.pos(),
                                                                     188 - (rIn.pos() - lPos));
                        } else {
                            mEsFrames[lTsHeader.mPid]->mData->append(rIn.data() + rIn.pos(),
                                                                     lPesHeader.mPesPacketLength - 3 -
                                                                     lPesHeader.mHeaderDataLength);
                        }

                        if(mEsFrames[lTsHeader.mPid]->mData->size() == payloadLength) {
                            mEsFrames[lTsHeader.mPid]->mCompleted = true;
                            mEsFrames[lTsHeader.mPid]->mPid = lTsHeader.mPid;
                            EsFrame *lEsFrame = mEsFrames[lTsHeader.mPid].get();
                            if (esOutCallback) {
                                esOutCallback(lEsFrame);
                            }
                            mEsFrames[lTsHeader.mPid]->reset();
                        }

                        rIn.skip(188 - (rIn.pos() - lPos));
                        continue;
                    }
                }

                if (mEsFrames[lTsHeader.mPid]->mExpectedPesPacketLength != 0 &&
                    mEsFrames[lTsHeader.mPid]->mData->size() + 188 - (rIn.pos() - lPos) >
                    mEsFrames[lTsHeader.mPid]->mExpectedPesPacketLength) {

                    uint8_t *dataPosition = rIn.data() + rIn.pos();
                    int size = mEsFrames[lTsHeader.mPid]->mExpectedPesPacketLength - mEsFrames[lTsHeader.mPid]->mData->size();
                    mEsFrames[lTsHeader.mPid]->mData->append(dataPosition,size);
                } else {
                    mEsFrames[lTsHeader.mPid]->mData->append(rIn.data() + rIn.pos(), 188 - (rIn.pos() - lPos));
                }

                //Enough data to deliver?
                if (mEsFrames[lTsHeader.mPid]->mExpectedPayloadLength == mEsFrames[lTsHeader.mPid]->mData->size()) {
                    mEsFrames[lTsHeader.mPid]->mCompleted = true;
                    mEsFrames[lTsHeader.mPid]->mPid = lTsHeader.mPid;
                    EsFrame *lEsFrame = mEsFrames[lTsHeader.mPid].get();
                    if (esOutCallback) {
                        esOutCallback(lEsFrame);
                    }
                    mEsFrames[lTsHeader.mPid]->reset();
                }

            }
        }
#if 0
        else if (mPcrId != 0 && mPcrId == lTsHeader.mPid) {
            AdaptationFieldHeader lAdaptField;
            lAdaptField.decode(rIn);
            uint64_t lPcr = readPcr(rIn);
            if (pcrOutCallback) {
                pcrOutCallback(lPcr);
            }
        }
#endif

        rIn.skip(188 - (rIn.pos() - lPos));
    }

    if (rIn.size()-rIn.pos()) {
        mRestData.append(rIn.data()+rIn.pos(),rIn.size()-rIn.pos());
    }

    rIn.clear();
    return 0;
}
