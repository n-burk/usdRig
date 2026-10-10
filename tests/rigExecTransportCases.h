// Included by the existing Format executable inside its test namespace.
void TestTransportCases()
{
    std::vector<uint8_t> raw(16384, 42), encoded, repeat;
    std::string error;
    transport::Stats encodedStats;
    CHECK(transport::Encode(raw.data(), raw.size(), &encoded, &error, &encodedStats));
    CHECK(!encoded.empty());
    CHECK(transport::Encode(raw.data(), raw.size(), &repeat, &error));
    CHECK(encoded == repeat);
    transport::Buffer decoded;
    CHECK(transport::Decode(encoded.data(), encoded.size(), &decoded, &error));
    CHECK(decoded.size == raw.size());
    CHECK(!std::memcmp(decoded.data.get(), raw.data(), raw.size()));
    CHECK(encodedStats.sdkActive == 0);
    for (size_t call = 1; call <= encodedStats.sdkCalls; ++call) {
        std::vector<uint8_t> retained{1,2,3}; transport::Stats stats;
        CHECK(!transport::Encode(raw.data(), raw.size(), &retained, &error, &stats, {call,false}));
        CHECK(retained == std::vector<uint8_t>({1,2,3}));
        CHECK(stats.sdkActive == 0);
    }
    transport::Stats decodeStats;
    CHECK(transport::Decode(encoded.data(), encoded.size(), &decoded, &error, &decodeStats));
    for(size_t call=1;call<=decodeStats.sdkCalls;++call) {
        auto *before=decoded.data.get();transport::Stats stats;
        CHECK(!transport::Decode(encoded.data(),encoded.size(),&decoded,&error,&stats,{call,false}));
        CHECK(decoded.data.get()==before);CHECK(stats.sdkActive==0);
    }
    CHECK(!transport::Decode(encoded.data(),encoded.size(),&decoded,&error,nullptr,{0,true}));
    CHECK(!transport::Encode(raw.data(),raw.size(),&repeat,&error,nullptr,{0,true}));
    auto retained=repeat;
    CHECK(!transport::Encode(raw.data(),raw.size(),&repeat,&error,nullptr,{0,false,2}));
    CHECK(repeat==retained);
    for(size_t field : {size_t(8),size_t(12),size_t(16),size_t(24),size_t(32),size_t(33),size_t(37),size_t(40),size_t(44)}) {
        auto bad=encoded;bad[field]^=0x80;transport::Stats stats;
        CHECK(!transport::Decode(bad.data(),bad.size(),&decoded,&error,&stats));CHECK(stats.sdkActive==0);
    }
    auto trailing=encoded;trailing.push_back(0);
    CHECK(!transport::Decode(trailing.data(),trailing.size(),&decoded,&error));
    std::vector<std::future<bool>> jobs;
    for(int i=0;i<4;++i) jobs.emplace_back(std::async(std::launch::async,[&raw,&encoded,i] {
        std::string why;std::vector<uint8_t> own;transport::Buffer result;
        if(!transport::Encode(raw.data(),raw.size(),&own,&why)||own!=encoded)return false;
        transport::Stats stats;
        bool ok=transport::Decode(own.data(),own.size(),&result,&why,&stats,{size_t(i&1),false});
        return stats.sdkActive==0 && (i&1 ? !ok : ok&&result.size==raw.size()&&!std::memcmp(result.data.get(),raw.data(),raw.size()));
    }));
    for(auto &job:jobs)CHECK(job.get());
    // Real semantic rejection enters through a checksum-valid transport,
    // not the validated writer (which would reject before transport decode).
    auto file=_MinimalFile();file.formatVersion=0;
    file.names.push_back(std::string(16384, 'x'));
    flatbuffers::FlatBufferBuilder builder;
    fb::FinishFileBuffer(builder,fb::File::Pack(builder,&file));
    CHECK(transport::Encode(builder.GetBufferPointer(),builder.GetSize(),&encoded,&error));
    CHECK(!encoded.empty());
    std::unique_ptr<fb::RigExecWireFile> opened;
    CHECK(!RigExecFormatOpen(encoded.data(),encoded.size(),&opened,&error));
    CHECK(!opened);
}
