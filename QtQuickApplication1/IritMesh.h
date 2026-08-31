#pragma once
//
// IritMesh - turns IRIT geometry into a MeshData.
//
// Internal header: it pulls in the IRIT C headers, so only .cpp files that
// actually talk to IRIT should include it. CadLoader uses it for whole files;
// PuzzleDivider uses it for one extracted sub-trivariate at a time.
//
#include "MeshData.h"

#include <QString>
#include <QStringList>

extern "C" {
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/attribut.h"
#include "inc_irit/ip_cnvrt.h"
#include "inc_irit/geom_lib.h"
#include "inc_irit/misc_lib.h"
}

namespace IritMesh {

// Tessellates freeform geometry, triangulates polygons, and harvests the lot
// into `out`. CONSUMES `objs` - it is freed here, valid or not, and the caller
// must not touch it afterwards.
//
// `fineNess` is IRIT's polygonal approximation control (20 is its default; a
// higher number means more polygons).
//
// Returns false with `error` set if IRIT raised a fatal error along the way.
// Succeeding with an empty mesh is possible and is NOT an error - check
// out->isEmpty() when that matters.
bool tessellate(IritPrsrObjectStruct *objs,
                MeshData            *out,
                double               fineNess,
                QString             *error);

// Names the object types in a chain, for diagnostics. Read-only; call it
// BEFORE tessellate(), which consumes the objects.
QStringList inventory(const IritPrsrObjectStruct *objs);

} // namespace IritMesh
