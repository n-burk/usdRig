#ifndef RIGEXEC_INPUT_ACTIONS_ADAPTERS_H
#define RIGEXEC_INPUT_ACTIONS_ADAPTERS_H

#include "rigExec/bakedProgram.h"
#include "rigExec/goldenPose.h"
#include "rigExec/inputReplay.h"
#include "rigExec/rigEvaluator.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

/// This adapter is valid for a program with the owning evaluator's authored
/// inputs and options, without direct program overrides. Its successful run
/// gains an actual native counterpart visit, independently judged by original
/// transcript replay. Current numerical output never becomes reference input.
inline bool RigExecInputActionsRunProgram(
    rigExec::RigExecBakedProgram &program,
    rigExec::RigExecRigEvaluator &owner,
    pxr::UsdTimeCode time,rigExec::RigExecRigPose *pose)
{
    const char *mode=std::getenv("RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES");
    if(!mode||std::string(mode)!="1")return program.Run(time,pose);
    bool okay=false;
    {
        rigExec::RigExecInputReplayComparisonScope scope(
            "direct program Run with authored-input native counterpart",true);
        okay=program.Run(time,pose);
    }
    // Epoch-binding refusal has no mathematical native equivalent. The
    // caller retains its explicit false/empty diagnostic assertions.
    if(!okay)return false;
    const auto native=owner.Evaluate(time);
    std::vector<rigExec::RigExecGoldenValue> programValues,nativeValues;
    std::string error;
    if(!pose||!native.valid||
       !rigExec::RigExecEncodeGoldenPose(*pose,&programValues,&error)||
       !rigExec::RigExecEncodeGoldenPose(native,&nativeValues,&error)||
       rigExec::RigExecGoldenVisit("equivalent",0,*pose,programValues)!=
       rigExec::RigExecGoldenVisit("equivalent",0,native,nativeValues)) {
        std::fprintf(stderr,"direct program/native input-adapter comparison failed for %s: %s\n",
            owner.GetRigPath().GetText(),error.c_str());
        return false;
    }
    return true;
}
#endif
