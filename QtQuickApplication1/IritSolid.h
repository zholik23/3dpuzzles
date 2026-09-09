#pragma once
//
// IritSolid - turning a MeshData into something IRIT's booleans will accept.
//
// Internal header: it pulls in the IRIT C headers, so only .cpp files that talk
// to IRIT should include it.
//
// Lifted out of IritJoint.cpp so the cage-vs-model intersection can use the
// same conversion. Both operations are "boolean a MeshData against another
// solid", and the two requirements below were expensive enough to find once
// that having a second, drifting copy of them would be a liability.
//
#include "MeshData.h"

extern "C" {
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/allocate.h"
#include "inc_irit/ip_cnvrt.h"   // IritPrsrOpenPolysToClosed
#include "inc_irit/bool_lib.h"
}

namespace IritSolid {

// Signed volume by the divergence theorem: positive when the mesh is wound
// outward, negative when wound inward. Only meaningful for a closed mesh.
double signedVolume(const MeshData &m);

// True when every directed edge has its opposite twin - i.e. the mesh is
// watertight. IRIT's booleans on an open shell produce garbage rather than an
// error, so this is worth checking before handing one over.
bool isClosed(const MeshData &m);

// Which way the polygon planes should face once handed to IRIT.
//
// MEASURED, not assumed. IRIT builds a polygon's plane from its vertex order,
// and that plane is how the booleans decide inside from outside - but the sign
// is the OPPOSITE of the right-hand rule signedVolume() uses. Feeding IRIT a
// conventionally outward-wound solid inverts both containment tests, and an
// intersection then silently computes a union: AND of a bounding box with an
// inscribed sphere returned the box (64000, every box polygon, no sphere
// polygon) instead of the sphere. Inverting the winding returned the sphere.
//
// So Inward is what the booleans want. Outward is kept because it is what the
// pre-existing joint code was built and tuned against; the two call sites now
// say which they mean instead of sharing one silent default.
enum class Winding {
    Outward,   // right-hand rule: signedVolume() > 0
    Inward     // what IRIT's booleans need
};

// MeshData to an IRIT polygon object, ready for a boolean. Returns NULL if
// nothing usable survived.
//
// Two things are not optional and neither is obvious:
//   * every polygon needs its plane equation (IritPrsrUpdatePolyPlane), since
//     that is how the booleans tell inside from outside;
//   * every vertex list has to be CLOSED back on itself, or IRIT rejects the
//     operand with "Vertex list must be circular for proper adjacencies".
// The mesh's own winding is measured and corrected to `w`, so the caller does
// not have to know how the mesh happened to arrive.
IritPrsrObjectStruct *fromMesh(const MeshData &m, Winding w);

// Makes every triangle agree with its neighbours about which way is out, then
// flips the whole thing if it ended up inside-out.
//
// Needed because a trivariate's boundary comes back as SURFACES, not as an
// oriented solid: IRIT tessellates each of the six faces on its own terms, so
// neighbouring faces can disagree. A boolean decides inside from outside using
// the polygon planes, and mixed winding makes that meaningless - the symptom is
// a piece whose measured volume is about half what its box should be, and an
// intersection that comes back several times too large.
//
// A global flip cannot fix mixed winding, only uniformly-inverted winding, so
// this has to run before fromMesh's flip, not instead of it.
void orientConsistently(MeshData *m);

// Merges vertices that are within `eps` of each other (default: a whisker of
// the bounding-box diagonal), rewrites the triangle indices onto the survivors
// and drops any triangle that collapsed in the process.
//
// MeshData::finalize welds on EXACT float equality, which is enough for a mesh
// read from a file but not for one that came out of the tessellator: IRIT
// evaluates each boundary surface of a trivariate on its own, so a box corner
// reached from the u face and from the v face differs in the last few bits.
// The box then has no shared edges at all - six loose quads - and anything that
// reasons about adjacency (orientConsistently, isClosed, a boolean) is working
// on a mesh that is not actually a solid.
void weldClose(MeshData *m, double eps = 0.0);

// Re-closes the vertex lists of an object. Objects coming out of the
// tessellator, a transform copy, or a previous boolean can all be open, so this
// is applied defensively to both operands before every operation.
void closeLists(IritPrsrObjectStruct *o);

} // namespace IritSolid
