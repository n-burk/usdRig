// Format-9 head ownership, reader binding and memo rejection cases.
// Included after the USD-free format fixture helpers.

static_assert(uint8_t(fb::StepKind::PropertyRevision) == 19 &&
                  uint8_t(fb::StepKind::RestCompose) == 20 &&
                  uint8_t(fb::StepKind::LadderCompose) == 21 &&
                  uint8_t(fb::StepKind::SkinTopology) == 22 &&
                  uint8_t(fb::SlotDomain::Rest) == 27 &&
                  uint8_t(fb::SlotDomain::Ladder) == 28 &&
                  uint8_t(fb::SlotDomain::SkinTopology) == 29 &&
                  uint8_t(fb::PropertyCandidateKind::SlotOnly) == 0 &&
                  uint8_t(fb::PropertyCandidateKind::ChainFinal) == 1 &&
                  uint8_t(fb::PropertyCandidateKind::PhasedRecord) == 2,
              "format-9 head and candidate identities are frozen");

RigExecWirePropertyInputCandidate
_HeadCandidate(uint32_t slot, fb::PropertyCandidateKind kind,
               int32_t version, bool raw = true)
{
    RigExecWirePropertyInputCandidate candidate;
    candidate.slot = slot;
    candidate.kind = uint8_t(kind);
    candidate.version = version;
    candidate.raw = raw;
    return candidate;
}

RigExecWireFile
_HeadPropertyFile()
{
    RigExecWireFile f = _MinimalFile();
    f.names = {"", "Rig", "A", "B", "C", "value"};
    for (uint32_t name = 2; name <= 4; ++name) {
        const auto prim = uint32_t(f.paths.size());
        f.paths.emplace_back(1, name, PathKind::Prim);
        f.paths.emplace_back(prim, 5, PathKind::Property);
    }
    f.values.resize(3);
    f.values[1].tag = InputTag::Bool;
    f.values[1].bits = 1;
    f.values[2].tag = InputTag::Float;
    f.values[2].bits = _Bits(1.0f);
    const auto listed = uint8_t(fb::InputSlotFlags::Listed);
    f.inputs = {fb::InputSlot(3, 2, 0, -1, InputTag::Float, listed),
                fb::InputSlot(5, 2, -1, 0, InputTag::Float, listed),
                fb::InputSlot(7, 2, -1, -1, InputTag::Float, 0)};
    f.listedInputs = 2;
    fb::RigExecWirePropertyChain chain;
    chain.target = 0;
    chain.valueType = fb::PropertyValueType::Float;
    chain.versionBase = 0;
    fb::RigExecWirePropertyRevision revision;
    const auto read = [](InputTag tag, uint32_t constant) {
        auto result = std::make_unique<RigExecWireInput>();
        result->tag = tag;
        result->constant = constant;
        result->mode = ReadMode::Pinned;
        return result;
    };
    revision.enabled = read(InputTag::Bool, 1);
    revision.defaultWeight = read(InputTag::Float, 2);
    revision.value = read(InputTag::Float, 2);
    revision.min = read(InputTag::Float, 2);
    revision.max = read(InputTag::Float, 2);
    revision.value->walk = {2};
    revision.value->propertyCandidates = {
        _HeadCandidate(2, fb::PropertyCandidateKind::SlotOnly, -1)};
    chain.revisions.push_back(std::move(revision));
    f.propertyChains.push_back(std::move(chain));
    fb::RigExecWirePhasedConsumer consumer;
    consumer.chain = 0;
    consumer.consumer = 1;
    consumer.consumerType = fb::PropertyValueType::Float;
    consumer.applied = 1;
    consumer.hops = {1};
    consumer.version = 2;
    f.phasedConsumers.push_back(std::move(consumer));
    f.pose->hasPropertyChains = true;
    f.pose->composeGroups.resize(1);

    f.steps.resize(3);
    auto &base = f.steps[0];
    base.kind = fb::StepKind::PropertyRevision;
    base.object = 0;
    base.part = 0;
    base.isHead = true;
    base.writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 0, 1)};
    base.headInputSlots = {0};
    base.succs = {1};
    auto &part = f.steps[1];
    part.kind = fb::StepKind::PropertyRevision;
    part.object = 0;
    part.isHead = true;
    part.part = 1;
    part.reads = {fb::SlotRange(fb::SlotDomain::PropertyResult, 0, 1)};
    part.writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 1, 3)};
    part.headInputSlots = {2};
    part.preds = {0};
    part.succs = {2};
    auto &region = f.steps[2];
    region.kind = fb::StepKind::ComposeSubtree;
    region.object = 0;
    region.preds = {1};
    for (auto &step : f.steps) step.cluster = 0;
    fb::RigExecWireCluster cluster;
    cluster.members = {0, 1, 2};
    f.clustering->clusters.push_back(std::move(cluster));
    f.clustering->clusterOf = {0, 0, 0};
    fb::RigExecWireClusterSet set;
    set.clusters = 1;
    set.words = {1};
    set.words = {0};
    *f.cones->always = set;
    *f.cones->poseClusters = set;

    fb::RigExecWirePathRead row;
    row.path = 5;
    row.headFallback = true;
    row.read = std::make_unique<RigExecWireInput>();
    row.read->tag = InputTag::Float;
    row.read->mode = ReadMode::Resolved;
    row.read->constant = 2;
    row.read->walk = {1, 0, 2};
    row.read->propertyCandidates = {
        _HeadCandidate(1, fb::PropertyCandidateKind::PhasedRecord, 2),
        _HeadCandidate(0, fb::PropertyCandidateKind::ChainFinal, 1),
        _HeadCandidate(2, fb::PropertyCandidateKind::SlotOnly, -1)};
    f.geometry->pathReads.push_back(std::move(row));
    _DeclareFixtureStageFrames(f);
    return f;
}

RigExecWireFile
_HeadComposeFile()
{
    auto f = _MinimalFile();
    f.names = {"", "Rig", "Parent", "Child", "XYZ"};
    f.paths.emplace_back(1, 2, PathKind::Prim);
    f.paths.emplace_back(2, 3, PathKind::Prim);
    f.paths.emplace_back(0, 4, PathKind::Token);
    f.values.resize(5);
    f.values[4].tag = InputTag::Vec3d;
    f.values[4].vec3d = std::make_unique<RigExecWireVec3d>();
    f.values[1].tag = InputTag::Matrix4d;
    f.values[1].matrix = std::make_unique<RigExecWireMatrix4d>();
    for (size_t i = 0; i < 4; ++i) (*f.values[1].matrix)[i * 4 + i] = 1;
    f.values[3].tag = InputTag::Token;
    f.values[3].bits = 4;
    auto &meta = *f.slotMeta;
    meta.paths = {2, 3};
    meta.publicationRoles = {0, 0};
    meta.slotKind = {fb::SlotKind::FirstFramePose, fb::SlotKind::FirstFramePose};
    meta.providerActive = {1,1};
    meta.parent = {-1, 0};
    meta.propParent = {-1, 0};
    meta.needFinal = {0, 1};
    meta.needBase = {0, 0};
    auto &constants = *f.constants;
    const auto matrix = *f.values[1].matrix;
    constants.restM = constants.selfD = constants.parentDinv =
        constants.restRoundTrip = constants.defaultRoundTrip =
        constants.posedAuthoredM = {matrix, matrix};
    constants.restPts.resize(2);
    constants.restFrames.resize(2);
    constants.rotOrder = {4, 4};
    constants.posedAuthored = {0, 0};
    constants.noScaleAvars = {0, 0};
    constants.avarConstants.resize(22);
    constants.rotationSign = {0, 0};
    const auto input = [](InputTag tag, uint32_t constant) {
        RigExecWireInput result;
        result.tag = tag;
        result.constant = constant;
        return result;
    };
    for (size_t slot = 0; slot < 2; ++slot) {
        fb::RigExecWireLadder ladder;
        ladder.restSpace = std::make_unique<RigExecWireInput>(input(InputTag::Matrix4d, 1));
        ladder.defaultSpace = std::make_unique<RigExecWireInput>(input(InputTag::Matrix4d, 1));
        ladder.posedSpace = std::make_unique<RigExecWireInput>(input(InputTag::Matrix4d, 1));
        ladder.restAvars.assign(6, input(InputTag::Double, 2));
        ladder.defaultAvars.assign(6, input(InputTag::Double, 2));
        ladder.rotationOrder = std::make_unique<RigExecWireInput>(input(InputTag::Token, 3));
        for (auto *read : {&ladder.parentSpace,&ladder.parentDefaultSpace,&ladder.avarDefaultSpace,&ladder.posedDefaultSpace,&ladder.interveningSpace}) *read=std::make_unique<RigExecWireInput>(input(InputTag::Matrix4d,1));
        ladder.rotationSign=std::make_unique<RigExecWireInput>(input(InputTag::Vec3d,4));
        f.pose->ladders.push_back(std::move(ladder));
        fb::RigExecWireComposeGroup group;
        group.begin = int32_t(slot);
        group.end = int32_t(slot + 1);
        if (slot) group.parentSlots = {0};
        f.pose->composeGroups.push_back(std::move(group));
    }
    f.pose->restChainVaries = {0, 0};
    f.steps.resize(6);
    for (size_t slot = 0; slot < 2; ++slot) {
        const auto &body = f.pose->ladders[slot];
        auto &rest = f.steps[slot * 2];
        rest.kind = fb::StepKind::RestCompose;
        rest.object = int32_t(slot);
        rest.part = 0;
        rest.isHead = true;
        rest.writes = {fb::SlotRange(fb::SlotDomain::Rest, uint32_t(slot), uint32_t(slot + 1))};
        rest.headInputReads = body.restAvars;
        rest.headInputReads.push_back(*body.restSpace);
        rest.headInputReads.push_back(*body.interveningSpace);
        auto &ladder = f.steps[slot * 2 + 1];
        ladder.kind = fb::StepKind::LadderCompose;
        ladder.object = int32_t(slot);
        ladder.part = 1;
        ladder.isHead = true;
        ladder.reads = {fb::SlotRange(fb::SlotDomain::Rest, uint32_t(slot), uint32_t(slot + 1))};
        ladder.writes = {fb::SlotRange(fb::SlotDomain::Ladder, uint32_t(slot), uint32_t(slot + 1))};
        ladder.headInputReads = {*body.posedSpace, *body.defaultSpace,*body.parentSpace,*body.parentDefaultSpace,*body.avarDefaultSpace,*body.posedDefaultSpace,*body.rotationSign};
        ladder.headInputReads.insert(ladder.headInputReads.end(), body.defaultAvars.begin(), body.defaultAvars.end());
        ladder.headInputReads.push_back(*body.rotationOrder);
    }
    f.steps[0].succs = {1, 2, 3};
    f.steps[1].preds = {0};
    f.steps[1].succs = {3, 4};
    f.steps[2].reads = {fb::SlotRange(fb::SlotDomain::Rest, 0, 1)};
    f.steps[2].preds = {0};
    f.steps[2].succs = {3, 5};
    f.steps[3].reads = {fb::SlotRange(fb::SlotDomain::Rest, 0, 2),
                       fb::SlotRange(fb::SlotDomain::Ladder, 0, 1)};
    f.steps[3].preds = {0, 1, 2};
    f.steps[3].succs = {4};
    auto &compose = f.steps[4];
    compose.kind = fb::StepKind::ComposeSubtree;
    compose.object = 1;
    compose.preds = {1, 3};
    compose.succs = {5};
    compose.reads = {fb::SlotRange(fb::SlotDomain::Ladder, 0, 2)};
    compose.writes = {fb::SlotRange(fb::SlotDomain::PoseFin, 1, 2),
                      fb::SlotRange(fb::SlotDomain::PoseBase,1,2)};
    auto &provider = f.steps[5];
    provider.kind = fb::StepKind::ProviderMatrix;
    provider.object = 1;
    provider.part = 1;
    provider.preds = {2, 4};
    provider.reads = {fb::SlotRange(fb::SlotDomain::Rest, 1, 2),
                      fb::SlotRange(fb::SlotDomain::PoseFin, 1, 2),
                      fb::SlotRange(fb::SlotDomain::PoseBase,1,2)};
    provider.writes = {fb::SlotRange(fb::SlotDomain::FinalMatrix, 1, 2)};
    for (auto &step : f.steps) step.cluster = 0;
    fb::RigExecWireCluster cluster;
    cluster.members = {0, 1, 2, 3, 4, 5};
    f.clustering->clusters.push_back(std::move(cluster));
    f.clustering->clusterOf.assign(6, 0);
    fb::RigExecWireClusterSet set;
    set.clusters = 1;
    set.words = {1};
    *f.cones->poseClusters = set;
    set.words = {0};
    *f.cones->always = set;
    f.cones->avarCluster = {0, 0};
    _DeclareFixtureStageFrames(f);
    return f;
}

void
TestSpaceCheckpointDeclarations()
{
    _context="space checkpoint declarations";
    auto file=_HeadComposeFile();
    fb::RigExecWireSpaceCheckpoint checkpoint;
    checkpoint.key="test:unswitched";
    checkpoint.recompose={1};
    file.pose->spaceCheckpoints.push_back(checkpoint);
    fb::RigExecWireStep avars;
    avars.kind=fb::StepKind::AvarInputs; avars.object=1; avars.cluster=0;
    avars.succs={7}; avars.writes={fb::SlotRange(fb::SlotDomain::Avars,1,2)};
    file.steps.push_back(avars);
    fb::RigExecWireStep context;
    context.kind=fb::StepKind::SpaceCheckpoint; context.object=0; context.cluster=0;
    context.preds={3,6};
    context.reads={fb::SlotRange(fb::SlotDomain::Avars,1,2),fb::SlotRange(fb::SlotDomain::Ladder,1,2)};
    context.writes={fb::SlotRange(fb::SlotDomain::SwitchFrame,0,1)};
    file.steps.push_back(context); file.steps[3].succs.push_back(7);
    file.clustering->clusters[0].members.insert(file.clustering->clusters[0].members.end(),{6,7});
    file.clustering->clusterOf.insert(file.clustering->clusterOf.end(),{0,0});
    _DeclareFixtureStageFrames(file);
    std::string why; std::vector<uint8_t> bytes;
    CHECK(_Write(file,&bytes,&why));
    if(!why.empty()) std::printf("  checkpoint fixture: %s\n",why.c_str());
    CHECK(_Open(bytes,&why));
    const auto refuses=[&](RigExecWireFile bad,const char *needle) {
        why.clear(); CHECK(!RigExecFormatValidate(bad,&why)); CHECK(why.find(needle)!=std::string::npos);
        why.clear(); CHECK(!_Open(_PackUnchecked(bad),&why)); CHECK(why.find(needle)!=std::string::npos);
    };
    auto bad=file; bad.steps[7].reads.erase(bad.steps[7].reads.begin());
    refuses(std::move(bad),"body SSA missing read Avars[1]");
    bad=file; bad.pose->spaceCheckpoints[0].anchor=2;
    refuses(std::move(bad),"anchor");
    bad=file; bad.steps[7].writes.clear();
    refuses(std::move(bad),"body SSA missing write SwitchFrame[0]");
}

RigExecWireFile
_PoseBodyFile()
{
    auto f=_HeadComposeFile();
    const auto value=[&](InputTag tag) {
        fb::RigExecWireValue constant; constant.tag=tag;
        if(tag==InputTag::Vec3d) constant.vec3d=std::make_unique<RigExecWireVec3d>();
        f.values.push_back(std::move(constant));
        RigExecWireInput read; read.tag=tag; read.constant=uint32_t(f.values.size()-1);
        return std::make_unique<RigExecWireInput>(read);
    };
    fb::RigExecWireConstraint constraint;
    constraint.path=3; constraint.target=1; constraint.targetSlots={1}; constraint.sources={1};
    for(auto *read:{&constraint.enabled,&constraint.affectX,&constraint.affectY,&constraint.affectZ,
        &constraint.tX,&constraint.tY,&constraint.tZ,&constraint.rX,&constraint.rY,&constraint.rZ,
        &constraint.sX,&constraint.sY,&constraint.sZ}) *read=value(InputTag::Bool);
    constraint.defaultWeight=value(InputTag::Float);
    for(auto *read:{&constraint.offset,&constraint.aimVector,&constraint.upVector,&constraint.rotationOffset,
        &constraint.worldUpVector,&constraint.poleVector}) *read=value(InputTag::Vec3d);
    constraint.twistDegrees=value(InputTag::Double);
    f.pose->constraints.push_back(std::move(constraint));
    fb::RigExecWireWalkStep walk; walk.index=0; f.pose->walkSteps.push_back(std::move(walk));
    fb::RigExecWireCommit commit;
    commit.moverPath=3; commit.slots={1}; commit.slotReads={1}; commit.slotWrites={3};
    commit.targetRead=1; commit.targetReads={1}; commit.sourceReads={1};
    f.pose->commits.push_back(std::move(commit));
    auto provider=std::move(f.steps[5]);
    auto &body=f.steps[5]; body=fb::RigExecWireStep{};
    body.kind=fb::StepKind::Constraint; body.object=0; body.cluster=0;
    body.preds={2,4}; body.succs={6};
    body.reads={fb::SlotRange(fb::SlotDomain::PoseFin,1,2),fb::SlotRange(fb::SlotDomain::Rest,1,2)};
    body.writes={fb::SlotRange(fb::SlotDomain::PoseFin,3,4),fb::SlotRange(fb::SlotDomain::CommitTable,0,1)};
    provider.preds={2,4,5};
    provider.reads={fb::SlotRange(fb::SlotDomain::Rest,1,2),fb::SlotRange(fb::SlotDomain::PoseBase,1,2),
        fb::SlotRange(fb::SlotDomain::PoseFin,3,4)};
    f.steps[2].succs.push_back(6);
    f.steps[4].succs={5,6};
    f.steps.push_back(std::move(provider));
    f.clustering->clusterOf.push_back(0);
    f.clustering->clusters[0].members.push_back(6);
    _DeclareFixtureStageFrames(f);
    return f;
}

void
TestPoseBodyDeclarations()
{
    _context="pose body SSA declarations";
    auto valid=_PoseBodyFile(); std::string why;
    CHECK(RigExecFormatValidate(valid,&why));
    if(!why.empty()) std::printf("  pose body fixture: %s\n",why.c_str());
    std::vector<uint8_t> bytes;
    CHECK(_Write(valid,&bytes,&why));
    CHECK(_Open(bytes,&why));
    const auto refuses=[&](RigExecWireFile file,const std::string &needle) {
        why.clear(); CHECK(!RigExecFormatValidate(file,&why));
        CHECK(why.find("Constraint")!=std::string::npos && why.find(needle)!=std::string::npos);
        why.clear(); CHECK(!_Open(_PackUnchecked(file),&why)); CHECK(why.find(needle)!=std::string::npos);
    };
    auto changed=valid;
    changed.steps[5].object=int32_t(valid.pose->commits.size());
    refuses(std::move(changed),"commit body object out of range");
    changed=valid;
    changed.pose->commits[0].sourceReads[0]=0; // In-range, but not the declared body source.
    refuses(std::move(changed),"body SSA missing read PoseFin[0]");
    changed=valid;
    changed.steps[5].reads.erase(changed.steps[5].reads.begin());
    refuses(std::move(changed),"body SSA missing read PoseFin[1]");
    changed=valid;
    changed.steps[5].writes.erase(changed.steps[5].writes.begin());
    refuses(std::move(changed),"body SSA missing write PoseFin[3]");
}

RigExecWireFile
_HeadDoubleFile(bool cycle)
{
    auto f = _HeadPropertyFile();
    f.names.push_back("D");
    const auto prim = uint32_t(f.paths.size());
    f.paths.emplace_back(1, uint32_t(f.names.size() - 1), PathKind::Prim);
    f.paths.emplace_back(prim, 5, PathKind::Property);
    fb::RigExecWireValue value;
    value.tag = InputTag::Double;
    value.bits = _Bits(1.0);
    f.values.push_back(std::move(value));
    f.inputs.emplace_back(prim + 1, 3, -1, -1, InputTag::Double,
                          uint8_t(fb::InputSlotFlags::HasValue));
    auto &read = *f.geometry->pathReads[0].read;
    read.walk = {1, 0, 3, 2};
    read.propertyCandidates = {
        _HeadCandidate(1, fb::PropertyCandidateKind::PhasedRecord, 2, false),
        _HeadCandidate(0, fb::PropertyCandidateKind::ChainFinal, 1, false),
        _HeadCandidate(3, fb::PropertyCandidateKind::SlotOnly, -1, false)};
    read.doubleCandidates = {
        _HeadCandidate(3, fb::PropertyCandidateKind::SlotOnly, -1),
        _HeadCandidate(2, fb::PropertyCandidateKind::SlotOnly, -1)};
    if (cycle) {
        read.doubleCandidates.push_back(
            _HeadCandidate(1, fb::PropertyCandidateKind::PhasedRecord, 2));
        read.doubleCandidates.push_back(
            _HeadCandidate(0, fb::PropertyCandidateKind::ChainFinal, 1));
    }
    _DeclareFixtureStageFrames(f);
    return f;
}

void
TestHeadDoubleFormatCases()
{
    std::string why;
    for (const bool cycle : {false, true}) {
        _context = cycle ? "head format: fresh Double cycle" :
                           "head format: Double suffix";
        const auto f = _HeadDoubleFile(cycle);
        CHECK(RigExecFormatValidate(f, &why));
        if (!why.empty()) std::printf("  Double fixture refused: %s\n", why.c_str());
        std::vector<uint8_t> bytes, again;
        CHECK(_Write(f, &bytes, &why));
        const auto opened = _Open(bytes, &why);
        CHECK(opened);
        if (opened) {
            CHECK(_Write(*opened, &again) && again == bytes);
            CHECK(opened->geometry->pathReads[0].read->doubleCandidates.size() ==
                  (cycle ? 4u : 2u));
        }
    }
    int cases = 0;
    const auto expect = [&](const char *name, const char *fragment,
                            const std::function<void(RigExecWireInput &)> &mutate) {
        _context = std::string("head format: ") + name;
        auto f = _HeadDoubleFile(true);
        mutate(*f.geometry->pathReads[0].read);
        ++cases;
        why.clear();
        const bool accepted = RigExecFormatValidate(f, &why);
        CHECK(!accepted && _Contains(why, fragment));
        const auto validation = why;
        CHECK(!_Open(_PackUnchecked(f), &why));
        CHECK(why == "invalid .rigexec: " + validation);
        if (accepted || !_Contains(validation, fragment))
            std::printf("  got '%s', expected '%s'\n", validation.c_str(), fragment);
    };
    expect("Double wrong first hop", "does not start at its double hop", [](auto &r) {
        std::swap(r.doubleCandidates[0], r.doubleCandidates[1]);
    });
    expect("Double omitted suffix", "omits its walk suffix", [](auto &r) {
        r.doubleCandidates.resize(1);
    });
    expect("Double changed suffix", "differs from its walk suffix", [](auto &r) {
        std::swap(r.doubleCandidates[1], r.doubleCandidates[2]);
    });
    expect("Double reversed cycle prefix", "out of walk order", [](auto &r) {
        std::swap(r.doubleCandidates[2], r.doubleCandidates[3]);
    });
    expect("Double normal raw leakage", "raw flag disagrees", [](auto &r) {
        r.propertyCandidates[0].raw = true;
    });
    expect("Double tail suppresses raw", "raw flag disagrees", [](auto &r) {
        r.doubleCandidates[1].raw = false;
    });
    std::printf("head format: two Double segments round-tripped, %d defects refused\n", cases);
}

RigExecWireFile
_HeadEnvelopeFile()
{
    auto f = _HeadPropertyFile();
    const auto token = [&](const char *name) {
        f.names.push_back(name);
        const auto path = uint32_t(f.paths.size());
        f.paths.emplace_back(0, uint32_t(f.names.size() - 1), PathKind::Token);
        return path;
    };
    const auto scalar = token("RigExecStaticWeight");
    const auto combine = token("RigExecCombineWeight");
    token("RigExecSphereWeight");
    for (size_t i = 0; i < 3; ++i) {
        fb::RigExecWireWeightObject weight;
        weight.path = uint32_t(2 + i * 2);
        weight.type = i == 2 ? combine : scalar;
        weight.envelopeOnly = false;
        for (auto *read : {&weight.defaultWeight, &weight.driver, &weight.scale,
                          &weight.bias, &weight.strength, &weight.invert,
                          &weight.falloffMin, &weight.falloffMax,
                          &weight.scaleXPos, &weight.scaleYPos, &weight.scaleZPos,
                          &weight.scaleXNeg, &weight.scaleYNeg, &weight.scaleZNeg,
                          &weight.scaleX, &weight.scaleY, &weight.scaleZ,
                          &weight.extentU, &weight.extentV}) {
            *read = std::make_unique<RigExecWireInput>();
            (*read)->tag = InputTag::Float;
            (*read)->mode = ReadMode::Baked;
            (*read)->constant = 2;
        }
        if (i == 2) {
            weight.base = 0;
            weight.inputs = {1};
        }
        f.geometry->weightObjects.push_back(std::move(weight));
    }
    f.propertyChains[0].revisions[0].envelope = 2;
    f.propertyChains[0].revisions[0].weightField = 0;
    fb::RigExecWireWeightField field;
    field.form=fb::WeightFieldForm::EnvelopeProperty;
    field.object=2; field.consumer=0; field.part=1;
    field.placementPhase=fb::WeightFieldPlacementPhase::Base;
    for(size_t object=0;object<3;++object) {
        const auto &weight=f.geometry->weightObjects[object];
        const RigExecWireInput *members[]={weight.defaultWeight.get(),weight.driver.get(),weight.scale.get(),weight.bias.get(),
            weight.strength.get(),weight.invert.get(),weight.falloffMin.get(),weight.falloffMax.get(),weight.scaleX.get(),weight.scaleY.get(),weight.scaleZ.get(),
            weight.scaleXPos.get(),weight.scaleYPos.get(),weight.scaleZPos.get(),weight.scaleXNeg.get(),weight.scaleYNeg.get(),weight.scaleZNeg.get(),weight.extentU.get(),weight.extentV.get()};
        for(size_t member=0;member<19;++member) {
            field.scalarObjects.push_back(int32_t(object));
            field.scalarMembers.push_back(fb::WeightFieldScalarMember(member));
            field.scalarReads.push_back(*members[member]);
            field.scalarReads.back().mode = ReadMode::Resolved;
        }
    }
    f.geometry->weightFields.push_back(std::move(field));
    auto base=f.steps[0],revision=f.steps[1],compose=f.steps[2];
    f.steps.clear(); base.preds.clear();base.succs={5};f.steps.push_back(base);
    for(int object=0;object<3;++object) {
        fb::RigExecWireStep packet;packet.kind=fb::StepKind::WeightPacket;packet.object=object;
        packet.writes={fb::SlotRange(fb::SlotDomain::WeightPacket,uint32_t(object),uint32_t(object+1))};
        packet.succs={4};
        if(object==2) { packet.reads={fb::SlotRange(fb::SlotDomain::WeightPacket,0,2)};packet.preds={1,2}; }
        f.steps.push_back(std::move(packet));
    }
    f.steps[1].succs={3,4};f.steps[2].succs={3,4};
    fb::RigExecWireStep producer;producer.kind=fb::StepKind::WeightField;producer.object=0;producer.isHead=true;
    producer.reads={fb::SlotRange(fb::SlotDomain::WeightPacket,0,3)};
    producer.writes={fb::SlotRange(fb::SlotDomain::WeightField,0,1)};producer.preds={1,2,3};producer.succs={5};
    f.steps.push_back(std::move(producer));
    revision.headAlwaysRuns=false;revision.preds={0,4};revision.succs={6};
    revision.reads.push_back(fb::SlotRange(fb::SlotDomain::WeightField,0,1));f.steps.push_back(std::move(revision));
    compose.preds={5};compose.succs.clear();f.steps.push_back(std::move(compose));
    auto &cluster=f.clustering->clusters[0];cluster.members={0,1,2,3,4,5,6};
    f.clustering->clusterOf.assign(7,0);
    for(auto &step:f.steps) step.cluster=0;
    _DeclareFixtureStageFrames(f);
    return f;
}

void
TestHeadEnvelopeFormatCases()
{
    _context = "head format: scalar property envelope closure";
    const auto valid = _HeadEnvelopeFile();
    std::string why;
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) std::printf("  envelope fixture refused: %s\n", why.c_str());
    if (!RigExecFormatValidate(valid, &why)) return;
    std::vector<uint8_t> bytes, again;
    CHECK(_Write(valid, &bytes, &why));
    const auto opened = _Open(bytes, &why);
    CHECK(opened);
    if (opened) CHECK(_Write(*opened, &again) && again == bytes);
    if (!opened) return;
    {
        auto wrongContext=valid;
        wrongContext.geometry->weightFields[0].availableChains={0};
        CHECK(!RigExecFormatValidate(wrongContext,&why));
        CHECK(_Contains(why,"publication context differs from its consumer"));
        const auto reason=why;
        CHECK(!_Open(_PackUnchecked(wrongContext),&why));
        CHECK(why=="invalid .rigexec: "+reason);
    }
    // Selected scalar wrappers are required even for signed zero; table
    // scalar defaults would silently erase its sign during serialization.
    auto scalarFile=_HeadEnvelopeFile();
    scalarFile.providerProgram=std::make_unique<fb::RigExecWireProviderProgram>();
    scalarFile.providerProgram->valueKeys={"scalar-double","scalar-float"};
    fb::RigExecWireProviderValue d;
    d.kind=fb::ProviderValueKind::Double;
    d.scalarDouble=std::make_unique<double>(-0.0); d.count=1; d.initialized=true;
    fb::RigExecWireProviderValue f;
    f.kind=fb::ProviderValueKind::Float;
    f.scalarFloat=std::make_unique<float>(-0.0f); f.count=1; f.initialized=true;
    scalarFile.providerProgram->defaults.push_back(std::move(d));
    scalarFile.providerProgram->defaults.push_back(std::move(f));
    CHECK(RigExecFormatValidate(scalarFile,&why));
    const auto scalarOpen=_Open(_PackUnchecked(scalarFile),&why);
    CHECK(scalarOpen && scalarOpen->providerProgram);
    if(scalarOpen && scalarOpen->providerProgram) {
        const auto &values=scalarOpen->providerProgram->defaults;
        CHECK(values.size()==2);
        if(values.size()==2) {
            CHECK(values[0].scalarDouble && std::signbit(*values[0].scalarDouble));
            CHECK(values[1].scalarFloat && std::signbit(*values[1].scalarFloat));
        }
    }
    scalarFile.providerProgram->defaults[0].scalarDouble.reset();
    CHECK(!RigExecFormatValidate(scalarFile,&why));
    scalarFile.providerProgram->defaults[0].scalarDouble=std::make_unique<double>(-0.0);
    scalarFile.providerProgram->defaults[1].scalarDouble=std::make_unique<double>(0.0);
    CHECK(!RigExecFormatValidate(scalarFile,&why));
    int cases=0;
    for(size_t object=0;object<3;++object) for(auto phase:{fb::WeightFieldPlacementPhase::Base,fb::WeightFieldPlacementPhase::Final}) {
        _context="S9 NoEquivalent: volume envelope structural admission";
        auto f=_HeadEnvelopeFile();
        f.geometry->weightObjects[object].type=uint32_t(f.paths.size()-1);
        f.geometry->weightFields[0].placementPhase=phase;
        CHECK(RigExecFormatValidate(f,&why));
        CHECK(_Open(_PackUnchecked(f),&why));
        ++cases;
        auto invalid=f;
        invalid.geometry->weightFields[0].placementPhase=fb::WeightFieldPlacementPhase(255);
        CHECK(!RigExecFormatValidate(invalid,&why) && _Contains(why,"invalid form"));
        invalid=f;
        invalid.geometry->weightFields[0].consumer=1;
        CHECK(!RigExecFormatValidate(invalid,&why) && _Contains(why,"consumer"));
        invalid=f;
        invalid.steps[4].writes.clear();
        CHECK(!RigExecFormatValidate(invalid,&why) && _Contains(why,"WeightField"));
    }
    std::printf("S9 NoEquivalent: %d volume envelope shapes admitted; ownership/phase/declaration negatives retained\n",cases);

}

void
TestHeadComposeFormatCases()
{
    _context = "head format: compose parent and child";
    auto valid = _HeadComposeFile();
    std::string why;
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) std::printf("  compose fixture refused: %s\n", why.c_str());
    std::vector<uint8_t> bytes;
    CHECK(_Write(valid, &bytes, &why));
    const auto opened = _Open(bytes, &why);
    CHECK(opened);
    if (opened) {
        std::vector<uint8_t> again;
        CHECK(_Write(*opened, &again) && again == bytes);
        CHECK(opened->steps[2].headInputReads.size() == 8);
        CHECK(opened->steps[3].headInputReads.size() == 14);
    }
    int cases = 0;
    const auto expect = [&](const char *name, const char *fragment, size_t step,
                            const std::function<void(RigExecWireFile &)> &mutate) {
        _context = std::string("head format: ") + name;
        auto f = _HeadComposeFile();
        mutate(f);
        ++cases;
        why.clear();
        const bool accepted = RigExecFormatValidate(f, &why);
        const auto label = "step " + std::to_string(step) + " (" +
                           RigExecFormatStepLabel(f, step) + ")";
        CHECK(!accepted && _Contains(why, fragment) && _Contains(why, label));
        const auto validation = why;
        CHECK(!_Open(_PackUnchecked(f), &why));
        CHECK(why == "invalid .rigexec: " + validation);
        if (accepted || !_Contains(validation, fragment))
            std::printf("  got '%s', expected '%s'\n", validation.c_str(), fragment);
    };
    expect("RestCompose wrong part", "not its group range", 2, [](auto &f) {
        f.steps[2].part = 1;
    });
    expect("LadderCompose wrong part", "not its group range", 3, [](auto &f) {
        f.steps[3].part = 0;
    });
    expect("RestCompose wrong group output", "not its group range", 2, [](auto &f) {
        f.steps[2].writes = {fb::SlotRange(fb::SlotDomain::Rest, 1, 1)};
    });
    expect("compose memo omitted", "memo does not cover exactly", 2, [](auto &f) {
        f.steps[2].headInputReads.pop_back();
    });
    expect("compose memo changed", "binding order or value differs", 2, [](auto &f) {
        f.steps[2].headInputReads[0].constant = 0;
    });
    expect("compose varying flag", "binding-varying flag differs", 3, [](auto &f) {
        f.steps[3].headVaryingLeaves = true;
    });
    expect("provider Rest omitted", "missing Rest[1]", 5, [](auto &f) {
        f.steps[5].reads.erase(f.steps[5].reads.begin());
    });
    expect("ComposeSubtree parent Ladder omitted", "missing Ladder[0]", 4, [](auto &f) {
        f.steps[4].reads = {fb::SlotRange(fb::SlotDomain::Ladder, 1, 2)};
    });
    expect("ComposeSubtree own Ladder omitted", "missing Ladder[1]", 4, [](auto &f) {
        f.steps[4].reads = {fb::SlotRange(fb::SlotDomain::Ladder, 0, 1)};
    });
    expect("RestCompose parent Rest omitted", "missing Rest[0]", 2, [](auto &f) {
        f.steps[2].reads.clear();
    });
    expect("LadderCompose own Rest omitted", "missing Rest[1]", 3, [](auto &f) {
        f.steps[3].reads[0] = fb::SlotRange(fb::SlotDomain::Rest, 0, 1);
    });
    expect("LadderCompose parent Rest omitted", "missing Rest[0]", 3, [](auto &f) {
        f.steps[3].reads[0] = fb::SlotRange(fb::SlotDomain::Rest, 1, 2);
    });
    expect("LadderCompose parent Ladder omitted", "missing Ladder[0]", 3, [](auto &f) {
        f.steps[3].reads.pop_back();
    });
    std::printf("head format: %d isolated compose/body defects refused\n", cases);
}

RigExecWireFile
_HeadTopologyFile()
{
    auto f = _ArrayFile();
    const size_t region = _RegionBegin(f);
    const size_t topology = region - 1;
    auto &reader = f.steps[region];
    reader.kind = fb::StepKind::RevisionStatic;
    reader.object = 0;
    reader.isSource = true;
    reader.reads = {fb::SlotRange(fb::SlotDomain::SkinTopology, 0, 1)};
    reader.writes = {fb::SlotRange(fb::SlotDomain::RevisionPacket, 0, 1)};
    reader.preds = {int32_t(topology)};
    f.steps[topology].succs = {int32_t(region)};
    _DeclareFixtureStageFrames(f);
    return f;
}

RigExecWireFile
_HeadChunkFile()
{
    auto f = _ArrayFile();
    _RegionStep(f, 0).writes.push_back(fb::SlotRange(fb::SlotDomain::PoseBase, 0, 1));
    auto &r = f.geometry->chains[0].revisions[0];
    r.op = uint8_t(fb::RevisionOp::Skin);
    r.influenceSlots = {0, 0};
    r.chunked = true;
    r.chunks.resize(2);
    r.chunks[0].begin = 0;
    r.chunks[0].end = 1;
    r.chunks[0].key = {0};
    r.chunks[1].begin = 1;
    r.chunks[1].end = 2;
    r.chunks[1].key = {0, 1};
    r.partitionElementSize = 2;
    r.partitionIndexCount = 4;
    r.partitionPointCount = 2;
    r.partitionProducerMin = r.partitionProducerMax = 1;
    r.partitionDistinctReads = 1;
    r.partitionProducerSets.resize(2);
    for (auto &set : r.partitionProducerSets)
        set.values = {fb::SlotRange(fb::SlotDomain::BaseMatrix, 0, 1)};
    f.geometry->revisionChunkCount = {2};
    f.geometry->chainChunkEnd = {2};
    f.slotMeta->needBase[0] = 1;
    fb::RigExecWireStep provider;
    provider.kind = fb::StepKind::ProviderMatrix;
    provider.object = 0;
    provider.part = 0;
    provider.reads = {fb::SlotRange(fb::SlotDomain::Rest, 0, 1),
                      fb::SlotRange(fb::SlotDomain::PoseBase, 0, 1)};
    provider.writes = {fb::SlotRange(fb::SlotDomain::BaseMatrix, 0, 1)};
    _AppendStep(f, std::move(provider));
    for (int part = 0; part < 2; ++part) {
        fb::RigExecWireStep chunk;
        chunk.kind = fb::StepKind::RevisionChunk;
        chunk.object = 0;
        chunk.part = part;
        chunk.reads = {fb::SlotRange(fb::SlotDomain::BaseMatrix, 0, 1),
                       fb::SlotRange(fb::SlotDomain::SkinTopology, 0, 1)};
        chunk.writes = {fb::SlotRange(fb::SlotDomain::RevisionOut,
                                      uint32_t(part), uint32_t(part + 1))};
        _AppendStep(f, std::move(chunk));
    }
    _DeclareFixtureStageFrames(f);
    return f;
}

void
TestHeadChunkFormatCases()
{
    _context = "head format: partition keys and aliased influence providers";
    const auto valid = _HeadChunkFile();
    std::string why;
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) std::printf("  chunk fixture refused: %s\n", why.c_str());
    if (!RigExecFormatValidate(valid, &why)) return;
    std::vector<uint8_t> bytes, again;
    CHECK(_Write(valid, &bytes, &why));
    const auto opened = _Open(bytes, &why);
    CHECK(opened);
    if (opened) CHECK(_Write(*opened, &again) && again == bytes);
    if (!opened) return;
    int cases = 0;
    const auto expect = [&](const char *name, const char *fragment,
                            const std::function<void(RigExecWireFile &)> &mutate) {
        _context = std::string("head format: ") + name;
        auto f = _HeadChunkFile();
        mutate(f);
        ++cases;
        const bool accepted = RigExecFormatValidate(f, &why);
        CHECK(!accepted && _Contains(why, fragment));
        const size_t step = f.steps.size() - 1;
        CHECK(_Contains(why, "step " + std::to_string(step) + " (" +
                              RigExecFormatStepLabel(f, step) + ")"));
        const auto validation = why;
        CHECK(!_Open(_PackUnchecked(f), &why));
        CHECK(why == "invalid .rigexec: " + validation);
        if (accepted || !_Contains(validation, fragment))
            std::printf("  got '%s', expected '%s'\n", validation.c_str(), fragment);
    };
    expect("partition key omits influence", "chunk key differs from its partition layout", [](auto &f) {
        f.geometry->chains[0].revisions[0].chunks[1].key = {0};
    });
    expect("chunk matrix omitted", "chunk matrix reads differ from its influence key", [](auto &f) {
        f.steps.back().reads.erase(f.steps.back().reads.begin());
    });
    expect("chunk wrong matrix phase", "chunk matrix reads differ from its influence key", [](auto &f) {
        f.steps.back().reads[0] = fb::SlotRange(fb::SlotDomain::FinalMatrix, 0, 1);
    });
    std::printf("head format: partition and alias reads round-tripped, %d defects refused\n", cases);
}

void
TestHeadTopologyFormatCases()
{
    _context = "head format: fixed topology and source reader";
    auto valid = _HeadTopologyFile();
    std::string why;
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) std::printf("  topology fixture refused: %s\n", why.c_str());
    if (!RigExecFormatValidate(valid, &why)) return;
    std::vector<uint8_t> bytes;
    CHECK(_Write(valid, &bytes, &why));
    CHECK(_Open(bytes, &why));
    int cases = 0;
    const auto expect = [&](const char *name, const char *fragment,
                            const std::function<void(RigExecWireFile &)> &mutate,
                            bool reader = false) {
        _context = std::string("head format: ") + name;
        auto f = _HeadTopologyFile();
        mutate(f);
        ++cases;
        why.clear();
        const bool accepted = RigExecFormatValidate(f, &why);
        const size_t step = _RegionBegin(f) - (reader ? 0 : 1);
        const auto label = "step " + std::to_string(step) + " (" +
                           RigExecFormatStepLabel(f, step) + ")";
        CHECK(!accepted && _Contains(why, fragment) && _Contains(why, label));
        const auto validation = why;
        CHECK(!_Open(_PackUnchecked(f), &why));
        CHECK(why == "invalid .rigexec: " + validation);
        if (accepted || !_Contains(validation, fragment))
            std::printf("  got '%s', expected '%s'\n", validation.c_str(), fragment);
    };
    expect("topology wrong part", "topology output ownership", [](auto &f) {
        f.steps[_RegionBegin(f) - 1].part = 1;
    });
    expect("topology writes foreign domain", "wrong head domain writer kind", [](auto &f) {
        f.steps[_RegionBegin(f) - 1].writes = {fb::SlotRange(fb::SlotDomain::Rest, 0, 1)};
    });
    expect("topology memo omitted", "topology memo", [](auto &f) {
        f.steps[_RegionBegin(f) - 1].headInputSlots.pop_back();
    });
    expect("topology memo has unrelated input", "topology memo", [](auto &f) {
        auto &slots = f.steps[_RegionBegin(f) - 1].headInputSlots;
        slots.insert(slots.begin(), 0);
    });
    expect("topology reader omitted", "missing SkinTopology[0]", [](auto &f) {
        f.steps[_RegionBegin(f)].reads.clear();
    }, true);
    expect("topology producer absent", "no earlier producer", [](auto &f) {
        const auto head = int32_t(_RegionBegin(f) - 1);
        f.steps.erase(f.steps.begin() + head);
        f.clustering->clusterOf.erase(f.clustering->clusterOf.begin() + head);
        for (auto &step : f.steps) {
            for (auto *edges : {&step.preds, &step.succs}) {
                edges->erase(std::remove(edges->begin(), edges->end(), head), edges->end());
                for (auto &id : *edges) if (id > head) --id;
            }
        }
        for (auto &cluster : f.clustering->clusters) {
            auto &members = cluster.members;
            members.erase(std::remove(members.begin(), members.end(), head), members.end());
            for (auto &id : members) if (id > head) --id;
        }
        for (auto *indices : {&f.cones->varyingSteps, &f.cones->overrideSteps}) {
            indices->erase(std::remove(indices->begin(), indices->end(), head), indices->end());
            for (auto &id : *indices) if (id > head) --id;
        }
    }, true);
    std::printf("head format: %d isolated topology defects refused\n", cases);
}

void
TestHeadFormatCases()
{
    _context = "head format: valid property prefix and private candidate";
    const auto valid = _HeadPropertyFile();
    std::string why;
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) std::printf("  head fixture refused: %s\n", why.c_str());
    std::vector<uint8_t> bytes;
    if (!_Write(valid, &bytes, &why)) {
        CHECK(false);
        return;
    }
    const auto opened = _Open(bytes, &why);
    CHECK(opened);
    if (opened) {
        std::vector<uint8_t> again;
        CHECK(_Write(*opened, &again) && again == bytes);
        CHECK(opened->steps[1].isHead);
        CHECK(opened->steps[1].headInputSlots == std::vector<uint32_t>{2});
        CHECK(opened->propertyChains[0].versionBase == 0);
        CHECK(opened->phasedConsumers[0].version == 2);
        CHECK(opened->geometry->pathReads[0].read->propertyCandidates[2].slot == 2);
        CHECK(opened->geometry->pathReads[0].read->propertyCandidates[2].version == -1);
    }
    int cases = 0;
    const auto expect = [&](const char *name, const char *fragment,
                            const std::function<void(RigExecWireFile &)> &mutate,
                            int namedStep = -1) {
        _context = std::string("head format: ") + name;
        auto f = _HeadPropertyFile();
        mutate(f);
        ++cases;
        why.clear();
        const bool accepted = RigExecFormatValidate(f, &why);
        CHECK(!accepted && _Contains(why, fragment));
        if (namedStep >= 0) {
            CHECK(_Contains(why, "step " + std::to_string(namedStep) + " (" +
                                  RigExecFormatStepLabel(f, size_t(namedStep)) + ")"));
        }
        const auto validation = why;
        CHECK(!_Open(_PackUnchecked(f), &why));
        CHECK(why == "invalid .rigexec: " + validation);
        if (accepted || !_Contains(validation, fragment))
            std::printf("  got '%s', expected '%s'\n", validation.c_str(), fragment);
    };
    expect("body category mismatch", "head category differs from its body kind", [](auto &f) {
        f.steps[0].isHead = false;
    }, 0);
    expect("head kind on region", "head category differs from its body kind", [](auto &f) {
        f.steps[2].kind = fb::StepKind::PropertyRevision;
    }, 2);
    expect("head source", "never a source", [](auto &f) {
        f.steps[0].isSource = true;
    }, 0);
    expect("head external", "never a source", [](auto &f) {
        f.steps[0].externalReads = true;
    }, 0);
    expect("head depends on later producer", "depends on later step", [](auto &f) {
        f.steps[1].preds = {2};
    });
    expect("wrong property ownership", "ownership differs", [](auto &f) {
        f.steps[1].writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 1, 2)};
    }, 1);
    expect("duplicate version writer", "duplicate head domain writer", [](auto &f) {
        f.steps[1].writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 0, 3)};
    }, 1);
    expect("missing part predecessor", "declare predecessor version", [](auto &f) {
        f.steps[1].reads.clear();
    }, 1);
    expect("out of range property writer", "head write out of range", [](auto &f) {
        f.steps[1].writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 1, 4)};
    }, 1);
    expect("invalid property part", "invalid property part", [](auto &f) {
        f.steps[1].part = 2;
    }, 1);
    expect("noncanonical versions", "noncanonical version_base", [](auto &f) {
        f.propertyChains[0].versionBase = 1;
        f.geometry->pathReads[0].read->propertyCandidates[1].version = 2;
    });
    expect("noncanonical record", "noncanonical version", [](auto &f) {
        f.phasedConsumers[0].version = 3;
        f.geometry->pathReads[0].read->propertyCandidates[0].version = 3;
    });
    expect("private candidate past inputs", "malformed property candidate", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates[2].slot = 3;
    });
    expect("unknown candidate enum", "malformed property candidate", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates[2].kind = 3;
    });
    expect("slot-only version", "malformed property candidate", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates[2].version = 0;
    });
    expect("wrong chain final", "not its chain final", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates[1].version = 0;
    });
    expect("wrong record version", "not its phased record", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates[0].version = 1;
    });
    expect("candidate order", "out of walk order", [](auto &f) {
        auto &list = f.geometry->pathReads[0].read->propertyCandidates;
        std::swap(list[0], list[1]);
    });
    expect("candidate raw permission", "raw flag disagrees", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates[0].raw = false;
    });
    expect("missing crossing", "lacks its property candidate", [](auto &f) {
        f.geometry->pathReads[0].read->propertyCandidates.erase(
            f.geometry->pathReads[0].read->propertyCandidates.begin());
    });
    expect("own fallback outside head", "not its head slot", [](auto &f) {
        f.geometry->pathReads[0].read->rawFallbackSlot = 2;
    });
    expect("memo omitted", "memo does not cover exactly", [](auto &f) {
        f.steps[1].headInputSlots.clear();
    }, 1);
    expect("memo duplicate", "not sorted unique", [](auto &f) {
        f.steps[1].headInputSlots = {2, 2};
    }, 1);
    expect("memo out of range", "memo slot out of range", [](auto &f) {
        f.steps[1].headInputSlots = {3};
    }, 1);
    expect("volatile flag without envelope", "volatile property memo flag", [](auto &f) {
        f.steps[1].headAlwaysRuns = true;
    }, 1);
    expect("raw head marked varying binding", "binding-varying flag", [](auto &f) {
        f.steps[1].headVaryingLeaves = true;
    }, 1);
    expect("ordinary body marked head", "head category differs from its body kind", [](auto &f) {
        f.steps[2].isHead = true;
    }, 2);
    std::printf("head format: %d isolated property/binding/memo defects refused\n", cases);
}
