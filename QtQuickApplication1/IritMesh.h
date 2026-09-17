#pragma once
//
// IritMesh - turns IRIT geometry into a MeshData.
//
// Internal header: it pulls in the IRIT C headers, so only .cpp files that
// actually talk to IRIT should include it.
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

bool tessellate(IritPrsrObjectStruct *objs,
                MeshData            *out,
                double               fineNess,
                QString             *error);

QStringList inventory(const IritPrsrObjectStruct *objs);

}
