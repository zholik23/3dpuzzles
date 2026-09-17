#pragma once
//
// IritSolid - turns a MeshData into something IRIT's booleans will accept.
//
// Internal header: it pulls in the IRIT C headers. Shared by the joint code and
// the cage-vs-model intersection, so the conversion rules exist in one place
// rather than in two copies that drift.
//
#include "MeshData.h"

extern "C" {
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/ip_cnvrt.h"
#include "inc_irit/bool_lib.h"
}

namespace IritSolid {

double signedVolume(const MeshData &m);

bool isClosed(const MeshData &m);

enum class Winding {
    Outward,
    Inward
};

IritPrsrObjectStruct *fromMesh(const MeshData &m, Winding w);

void orientConsistently(MeshData *m);

void weldClose(MeshData *m, double eps = 0.0);

void closeLists(IritPrsrObjectStruct *o);

}
